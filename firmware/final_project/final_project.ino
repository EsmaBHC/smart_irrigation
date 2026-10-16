/*
  Smart Agriculture IoT - ESP32
  Sensors : AHT20 (temp + humidity), BMP280 (pressure), capacitive soil sensor
  Outputs : irrigation pump (automatic), 2 LEDs (manual via MQTT)
  Network : WiFi + MQTT (authenticated) + small local web page

  Requires a "secrets.h" file next to this sketch (see secrets.example.h):
    WIFI_SSID, WIFI_PASSWORD, MQTT_BROKER, MQTT_PORT, MQTT_USER, MQTT_PASS
*/

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>

#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include <PubSubClient.h>

#include "secrets.h"

// ================= TOPICS =================

const char* TOPIC_TEMP         = "agriculture/esma/sensors/temp";
const char* TOPIC_HUMIDITY     = "agriculture/esma/sensors/humidity";
const char* TOPIC_SOIL         = "agriculture/esma/sensors/soil";
const char* TOPIC_PRESSURE     = "agriculture/esma/sensors/pressure";

const char* TOPIC_STATUS       = "agriculture/esma/status";
const char* TOPIC_ALERT        = "agriculture/esma/alerts";
const char* TOPIC_INFO         = "agriculture/esma/info";
const char* TOPIC_AVAILABILITY = "agriculture/esma/availability";   // online / offline (Last Will)

// States published by the ESP32
const char* TOPIC_IRRIGATION   = "agriculture/esma/actuators/irrigation";
const char* TOPIC_TEMP_LED     = "agriculture/esma/actuators/temperature_led";
const char* TOPIC_HUMIDITY_LED = "agriculture/esma/actuators/humidity_led";

// Commands received by the ESP32 (ON / OFF)
const char* TOPIC_TEMP_LED_CMD     = "agriculture/esma/actuators/temperature_led/cmd";
const char* TOPIC_HUMIDITY_LED_CMD = "agriculture/esma/actuators/humidity_led/cmd";

// ================= PINS =================

#define SDA_PIN 11
#define SCL_PIN 12

#define SOIL_PIN 7

#define IRRIGATION_PIN   4
#define TEMP_LED_PIN     5
#define HUMIDITY_LED_PIN 6

// ================= SOIL CALIBRATION =================

#define SOIL_AIR   2215    // raw value in dry air  (0 %)
#define SOIL_WATER 270     // raw value in water    (100 %)

// ================= THRESHOLDS =================

#define SOIL_DRY_LIMIT 50
#define SOIL_WET_LIMIT 80

#define TEMP_HIGH_LIMIT 30.0

#define HUMIDITY_LOW_LIMIT  35.0
#define HUMIDITY_HIGH_LIMIT 80.0

// ================= PUMP SAFETY =================

const unsigned long PUMP_MAX_RUN_MS   = 5UL * 60UL * 1000UL;   // stop after 5 min of continuous run
const unsigned long PUMP_COOLDOWN_MS  = 2UL * 60UL * 1000UL;   // then wait 2 min before restarting

// ================= ALERT BITS =================

#define ALERT_SOIL_DRY      (1 << 0)
#define ALERT_TEMP_HIGH     (1 << 1)
#define ALERT_HUMIDITY      (1 << 2)
#define ALERT_SENSOR_FAULT  (1 << 3)
#define ALERT_PUMP_TIMEOUT  (1 << 4)

// ================= OBJECTS =================

WebServer server(80);

Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// ================= VALUES =================

float temperature = NAN;
float humidity    = NAN;
float pressure    = NAN;     // hPa
int   soilPercent = 0;

bool ahtOk  = false;
bool bmpOk  = false;
bool soilOk = false;

// ================= STATES =================

bool irrigationActive = false;   // automatic
bool temperatureLedOn = false;   // manual (Node-RED)
bool humidityLedOn    = false;   // manual (Node-RED)

unsigned long pumpStartedAt = 0;
unsigned long pumpLockUntil = 0;

uint8_t lastAlertMask = 0;

String systemStatus = "STARTING";

// ================= TIMERS =================

unsigned long lastSensorRead    = 0;
unsigned long lastMQTTPublish   = 0;
unsigned long lastMQTTReconnect = 0;
unsigned long lastWiFiAttempt   = 0;

const unsigned long SENSOR_INTERVAL         = 2000;
const unsigned long MQTT_INTERVAL           = 5000;
const unsigned long MQTT_RECONNECT_INTERVAL = 5000;
const unsigned long WIFI_RETRY_INTERVAL     = 10000;

// ================= HELPERS =================

void publishMsg(const char* topic, const char* payload, bool retained = false) {
  if (mqttClient.connected()) {
    mqttClient.publish(topic, payload, retained);
  }
}

String numOrNull(float v, int decimals) {
  return isnan(v) ? String("null") : String(v, decimals);
}

// ================= READ SENSORS =================

int readSoilRaw() {
  // Average 16 samples to reduce ADC noise
  long sum = 0;
  for (int i = 0; i < 16; i++) {
    sum += analogRead(SOIL_PIN);
    delay(2);
  }
  return (int)(sum / 16);
}

void readSensors() {

  // --- AHT20 ---
  if (ahtOk) {
    sensors_event_t humidityEvent;
    sensors_event_t temperatureEvent;
    aht.getEvent(&humidityEvent, &temperatureEvent);

    float t = temperatureEvent.temperature;
    float h = humidityEvent.relative_humidity;

    if (isnan(t) || isnan(h) || h < 0 || h > 100) {
      temperature = NAN;
      humidity = NAN;
    } else {
      temperature = t;
      humidity = h;
    }
  } else {
    temperature = NAN;
    humidity = NAN;
  }

  // --- BMP280 ---
  if (bmpOk) {
    float p = bmp.readPressure() / 100.0;   // Pa -> hPa
    pressure = (isnan(p) || p < 300 || p > 1100) ? NAN : p;
  }

  // --- Soil ---
  int soilRaw = readSoilRaw();

  // A raw value of 0 or full scale means the sensor is unplugged or shorted
  soilOk = (soilRaw > 50 && soilRaw < 4090);

  if (soilOk) {
    soilPercent = map(soilRaw, SOIL_AIR, SOIL_WATER, 0, 100);
    soilPercent = constrain(soilPercent, 0, 100);
  }
}

// ================= IRRIGATION (AUTOMATIC, WITH SAFETY) =================

void controlIrrigation() {

  unsigned long now = millis();

  if (!soilOk) {
    // Never irrigate on invalid data
    irrigationActive = false;
  }
  else if (irrigationActive) {

    if (soilPercent > SOIL_WET_LIMIT) {
      irrigationActive = false;                         // soil wet enough
    }
    else if (now - pumpStartedAt >= PUMP_MAX_RUN_MS) {
      irrigationActive = false;                         // safety timeout
      pumpLockUntil = now + PUMP_COOLDOWN_MS;
      Serial.println("PUMP SAFETY: max run time reached, cooldown started");
    }
  }
  else {

    bool lockedOut = ((long)(pumpLockUntil - now) > 0);

    if (soilPercent < SOIL_DRY_LIMIT && !lockedOut) {
      irrigationActive = true;
      pumpStartedAt = now;
    }
  }

  digitalWrite(IRRIGATION_PIN, irrigationActive ? HIGH : LOW);
}

// ================= ALERTS =================

uint8_t computeAlertMask() {

  uint8_t mask = 0;

  if (!soilOk || !ahtOk || isnan(temperature) || isnan(humidity)) {
    mask |= ALERT_SENSOR_FAULT;
  }
  if (soilOk && soilPercent < SOIL_DRY_LIMIT) {
    mask |= ALERT_SOIL_DRY;
  }
  if (!isnan(temperature) && temperature > TEMP_HIGH_LIMIT) {
    mask |= ALERT_TEMP_HIGH;
  }
  if (!isnan(humidity) && (humidity < HUMIDITY_LOW_LIMIT || humidity > HUMIDITY_HIGH_LIMIT)) {
    mask |= ALERT_HUMIDITY;
  }
  if (!irrigationActive && (long)(pumpLockUntil - millis()) > 0) {
    mask |= ALERT_PUMP_TIMEOUT;
  }

  return mask;
}

// Publish an alert only when it newly appears (no spam)
void publishAlerts() {

  if (!mqttClient.connected()) {
    return;
  }

  uint8_t mask = computeAlertMask();
  uint8_t newAlerts = mask & ~lastAlertMask;

  if (newAlerts & ALERT_SENSOR_FAULT) publishMsg(TOPIC_ALERT, "SENSOR_FAULT");
  if (newAlerts & ALERT_SOIL_DRY)     publishMsg(TOPIC_ALERT, "SOIL_DRY");
  if (newAlerts & ALERT_TEMP_HIGH)    publishMsg(TOPIC_ALERT, "TEMPERATURE_HIGH");
  if (newAlerts & ALERT_HUMIDITY)     publishMsg(TOPIC_ALERT, "HUMIDITY_ABNORMAL");
  if (newAlerts & ALERT_PUMP_TIMEOUT) publishMsg(TOPIC_ALERT, "PUMP_TIMEOUT");

  lastAlertMask = mask;
}

// ================= SYSTEM STATUS =================

void updateSystemStatus() {

  uint8_t mask = computeAlertMask();

  if (mask & ALERT_SENSOR_FAULT)         systemStatus = "SENSOR_FAULT";
  else if (mask & ALERT_SOIL_DRY)        systemStatus = "SOIL_DRY";
  else if (mask & ALERT_TEMP_HIGH)       systemStatus = "TOO_HOT";
  else if (!isnan(humidity) && humidity < HUMIDITY_LOW_LIMIT)  systemStatus = "LOW_HUMIDITY";
  else if (!isnan(humidity) && humidity > HUMIDITY_HIGH_LIMIT) systemStatus = "HIGH_HUMIDITY";
  else                                   systemStatus = "NORMAL";
}

// ================= AUTOMATION =================

void updateAutomation() {
  controlIrrigation();     // only the pump is automatic
  updateSystemStatus();
}

// ================= MQTT CALLBACK =================

void mqttCallback(char* topic, byte* payload, unsigned int length) {

  String msg = "";

  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }

  msg.trim();
  msg.toUpperCase();

  Serial.print("MQTT received [");
  Serial.print(topic);
  Serial.print("] : ");
  Serial.println(msg);

  bool newState;

  if (msg == "ON" || msg == "TRUE" || msg == "1") {
    newState = true;
  }
  else if (msg == "OFF" || msg == "FALSE" || msg == "0") {
    newState = false;
  }
  else {
    return;   // unknown message, ignore
  }

  if (strcmp(topic, TOPIC_TEMP_LED_CMD) == 0) {

    temperatureLedOn = newState;
    digitalWrite(TEMP_LED_PIN, temperatureLedOn ? HIGH : LOW);
    publishMsg(TOPIC_TEMP_LED, temperatureLedOn ? "ON" : "OFF");
  }
  else if (strcmp(topic, TOPIC_HUMIDITY_LED_CMD) == 0) {

    humidityLedOn = newState;
    digitalWrite(HUMIDITY_LED_PIN, humidityLedOn ? HIGH : LOW);
    publishMsg(TOPIC_HUMIDITY_LED, humidityLedOn ? "ON" : "OFF");
  }
}

// ================= WIFI =================

void connectWiFi(unsigned long timeoutMs) {

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long start = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi CONNECTED!");
    Serial.print("ESP32 IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi not connected yet, will keep retrying in background.");
  }
}

void maintainWiFi() {

  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  unsigned long now = millis();

  if (now - lastWiFiAttempt < WIFI_RETRY_INTERVAL) {
    return;
  }

  lastWiFiAttempt = now;

  Serial.println("WiFi lost, reconnecting...");
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

// ================= MQTT CONNECTION =================

void reconnectMQTT() {

  if (mqttClient.connected() || WiFi.status() != WL_CONNECTED) {
    return;
  }

  unsigned long now = millis();

  if (now - lastMQTTReconnect < MQTT_RECONNECT_INTERVAL) {
    return;
  }

  lastMQTTReconnect = now;

  Serial.print("Connecting to Mosquitto... ");

  String clientId = "agriculture-esma-";
  clientId += WiFi.macAddress();

  // Last Will: if the ESP32 disappears, the broker publishes "offline" (retained)
  bool ok = mqttClient.connect(clientId.c_str(),
                               MQTT_USER, MQTT_PASS,
                               TOPIC_AVAILABILITY, 1, true, "offline");

  if (ok) {

    Serial.println("CONNECTED!");

    mqttClient.subscribe(TOPIC_TEMP_LED_CMD);
    mqttClient.subscribe(TOPIC_HUMIDITY_LED_CMD);

    publishMsg(TOPIC_AVAILABILITY, "online", true);
    publishMsg(TOPIC_INFO, "ESP32 Agriculture System connected");

    publishMsg(TOPIC_TEMP_LED, temperatureLedOn ? "ON" : "OFF");
    publishMsg(TOPIC_HUMIDITY_LED, humidityLedOn ? "ON" : "OFF");

    lastAlertMask = 0;   // re-announce active alerts after a reconnect
  }
  else {

    // 4 = bad credentials, 5 = not authorised
    Serial.print("FAILED, state = ");
    Serial.println(mqttClient.state());
  }
}

// ================= PUBLISH =================

void publishSensorData() {

  if (!mqttClient.connected()) {
    return;
  }

  char buffer[16];

  if (!isnan(temperature)) {
    snprintf(buffer, sizeof(buffer), "%.2f", temperature);
    publishMsg(TOPIC_TEMP, buffer);
  }

  if (!isnan(humidity)) {
    snprintf(buffer, sizeof(buffer), "%.2f", humidity);
    publishMsg(TOPIC_HUMIDITY, buffer);
  }

  if (!isnan(pressure)) {
    snprintf(buffer, sizeof(buffer), "%.1f", pressure);
    publishMsg(TOPIC_PRESSURE, buffer);
  }

  if (soilOk) {
    snprintf(buffer, sizeof(buffer), "%d", soilPercent);
    publishMsg(TOPIC_SOIL, buffer);
  }

  publishMsg(TOPIC_STATUS, systemStatus.c_str());

  publishMsg(TOPIC_IRRIGATION, irrigationActive ? "ON" : "OFF");
  publishMsg(TOPIC_TEMP_LED, temperatureLedOn ? "ON" : "OFF");
  publishMsg(TOPIC_HUMIDITY_LED, humidityLedOn ? "ON" : "OFF");
}

// ================= WEB DASHBOARD =================

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Smart Agriculture</title>
<style>
body { font-family: Arial; background: #eef5ee; text-align: center; margin: 0; padding: 20px; }
.container { max-width: 700px; margin: auto; }
h1 { color: #245c32; }
.card { background: white; margin: 15px; padding: 20px; border-radius: 15px; box-shadow: 0 3px 10px rgba(0,0,0,0.12); }
.value { font-size: 32px; font-weight: bold; }
.status { font-size: 24px; font-weight: bold; }
</style>
</head>
<body>
<div class="container">
<h1>Smart Agriculture IoT</h1>
<div class="card"><h2>Temperature</h2><div class="value"><span id="temperature">--</span> &deg;C</div></div>
<div class="card"><h2>Air Humidity</h2><div class="value"><span id="humidity">--</span> %</div></div>
<div class="card"><h2>Pressure</h2><div class="value"><span id="pressure">--</span> hPa</div></div>
<div class="card"><h2>Soil Moisture</h2><div class="value"><span id="soil">--</span> %</div></div>
<div class="card"><h2>System Status</h2><div class="status"><span id="status">--</span></div></div>
<div class="card"><h2>Irrigation</h2><div class="status"><span id="irrigation">--</span></div></div>
<div class="card"><h2>Temperature LED</h2><div class="status"><span id="tempLed">--</span></div></div>
<div class="card"><h2>Humidity LED</h2><div class="status"><span id="humidityLed">--</span></div></div>
<div class="card"><h2>MQTT</h2><div class="status"><span id="mqtt">--</span></div></div>
</div>
<script>
function fmt(v, d) { return (v === null || v === undefined) ? "--" : v.toFixed(d); }
function updateData() {
  fetch('/data')
    .then(r => r.json())
    .then(data => {
      document.getElementById("temperature").innerText = fmt(data.temperature, 1);
      document.getElementById("humidity").innerText = fmt(data.humidity, 1);
      document.getElementById("pressure").innerText = fmt(data.pressure, 1);
      document.getElementById("soil").innerText = data.soil === null ? "--" : data.soil;
      document.getElementById("status").innerText = data.status;
      document.getElementById("irrigation").innerText = data.irrigation ? "ON" : "OFF";
      document.getElementById("tempLed").innerText = data.temperatureLed ? "ON" : "OFF";
      document.getElementById("humidityLed").innerText = data.humidityLed ? "ON" : "OFF";
      document.getElementById("mqtt").innerText = data.mqtt ? "CONNECTED" : "DISCONNECTED";
    })
    .catch(e => console.log("Error:", e));
}
setInterval(updateData, 2000);
updateData();
</script>
</body>
</html>
)rawliteral";

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleData() {

  String json = "{";

  json += "\"temperature\":" + numOrNull(temperature, 1);
  json += ",\"humidity\":" + numOrNull(humidity, 1);
  json += ",\"pressure\":" + numOrNull(pressure, 1);
  json += ",\"soil\":" + (soilOk ? String(soilPercent) : String("null"));
  json += ",\"status\":\"" + systemStatus + "\"";
  json += ",\"irrigation\":" + String(irrigationActive ? "true" : "false");
  json += ",\"temperatureLed\":" + String(temperatureLedOn ? "true" : "false");
  json += ",\"humidityLed\":" + String(humidityLedOn ? "true" : "false");
  json += ",\"mqtt\":" + String(mqttClient.connected() ? "true" : "false");

  json += "}";

  server.send(200, "application/json", json);
}

// ================= SETUP =================

void setup() {

  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("==================================");
  Serial.println("     SMART AGRICULTURE IoT");
  Serial.println("==================================");

  pinMode(SOIL_PIN, INPUT);
  pinMode(IRRIGATION_PIN, OUTPUT);
  pinMode(TEMP_LED_PIN, OUTPUT);
  pinMode(HUMIDITY_LED_PIN, OUTPUT);

  digitalWrite(IRRIGATION_PIN, LOW);
  digitalWrite(TEMP_LED_PIN, LOW);
  digitalWrite(HUMIDITY_LED_PIN, LOW);

  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.println("Initializing AHT20...");
  ahtOk = aht.begin();
  Serial.println(ahtOk ? "AHT20 OK!" : "ERROR: AHT20 not found!");

  Serial.println("Initializing BMP280...");
  bmpOk = bmp.begin(0x77) || bmp.begin(0x76);
  Serial.println(bmpOk ? "BMP280 OK!" : "BMP280 not found (pressure disabled)");

  Serial.println();
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);

  connectWiFi(20000);   // 20 s max, never blocks forever

  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();

  Serial.println("Web server started!");

  readSensors();
  updateAutomation();

  Serial.println();
  Serial.println("SYSTEM READY!");
}

// ================= LOOP =================

void loop() {

  server.handleClient();
  maintainWiFi();
  reconnectMQTT();
  mqttClient.loop();

  unsigned long now = millis();

  if (now - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = now;
    readSensors();
    updateAutomation();
    publishAlerts();
  }

  if (now - lastMQTTPublish >= MQTT_INTERVAL) {
    lastMQTTPublish = now;
    publishSensorData();
  }
}
