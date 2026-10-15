#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <secrets.example.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include <PubSubClient.h>

// ================= WIFI =================


// ================= MQTT =================

// IP du PC Mosquitto (SANS espace)


// ================= TOPICS =================

const char* TOPIC_TEMP         = "agriculture/esma/sensors/temp";
const char* TOPIC_HUMIDITY     = "agriculture/esma/sensors/humidity";
const char* TOPIC_SOIL         = "agriculture/esma/sensors/soil";

const char* TOPIC_STATUS       = "agriculture/esma/status";
const char* TOPIC_ALERT        = "agriculture/esma/alerts";
const char* TOPIC_INFO         = "agriculture/esma/info";

// Etats publies par l'ESP32
const char* TOPIC_IRRIGATION   = "agriculture/esma/actuators/irrigation";
const char* TOPIC_TEMP_LED     = "agriculture/esma/actuators/temperature_led";
const char* TOPIC_HUMIDITY_LED = "agriculture/esma/actuators/humidity_led";

// Commandes recues par l'ESP32 (ON / OFF)
const char* TOPIC_TEMP_LED_CMD     = "agriculture/esma/actuators/temperature_led/cmd";
const char* TOPIC_HUMIDITY_LED_CMD = "agriculture/esma/actuators/humidity_led/cmd";

// ================= PINS =================

#define SDA_PIN 11
#define SCL_PIN 12

#define SOIL_PIN 7

#define IRRIGATION_PIN 4
#define TEMP_LED_PIN 5
#define HUMIDITY_LED_PIN 6

// ================= SOIL CALIBRATION =================

#define SOIL_AIR 2215
#define SOIL_WATER 270

// ================= THRESHOLDS =================

#define SOIL_DRY_LIMIT 50
#define SOIL_WET_LIMIT 80

#define TEMP_HIGH_LIMIT 30.0

#define HUMIDITY_LOW_LIMIT 35.0
#define HUMIDITY_HIGH_LIMIT 80.0

// ================= OBJECTS =================

WebServer server(80);

Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// ================= VALUES =================

float temperature = 0.0;
float humidity = 0.0;
int soilPercent = 0;

// ================= STATES =================

bool irrigationActive = false;   // automatique

bool temperatureLedOn = false;   // manuel (Node-RED)
bool humidityLedOn = false;      // manuel (Node-RED)

String systemStatus = "STARTING";

// ================= TIMERS =================

unsigned long lastSensorRead = 0;
unsigned long lastMQTTPublish = 0;
unsigned long lastMQTTReconnect = 0;

const unsigned long SENSOR_INTERVAL = 2000;
const unsigned long MQTT_INTERVAL = 5000;
const unsigned long MQTT_RECONNECT_INTERVAL = 5000;

// ================= READ SENSORS =================

void readSensors() {

  sensors_event_t humidityEvent;
  sensors_event_t temperatureEvent;

  aht.getEvent(&humidityEvent, &temperatureEvent);

  temperature = temperatureEvent.temperature;
  humidity = humidityEvent.relative_humidity;

  int soilRaw = analogRead(SOIL_PIN);

  soilPercent = map(soilRaw, SOIL_AIR, SOIL_WATER, 0, 100);
  soilPercent = constrain(soilPercent, 0, 100);
}

// ================= IRRIGATION (AUTOMATIQUE) =================

void controlIrrigation() {

  // Hysteresis :
  // Sol < 50%  -> pompe ON
  // Sol > 80%  -> pompe OFF
  // Entre les deux -> etat precedent

  if (soilPercent < SOIL_DRY_LIMIT) {
    irrigationActive = true;
  }
  else if (soilPercent > SOIL_WET_LIMIT) {
    irrigationActive = false;
  }

  digitalWrite(IRRIGATION_PIN, irrigationActive ? HIGH : LOW);
}

// ================= SYSTEM STATUS (selon les mesures) =================

void updateSystemStatus() {

  if (soilPercent < SOIL_DRY_LIMIT) {
    systemStatus = "SOIL_DRY";
  }
  else if (temperature > TEMP_HIGH_LIMIT) {
    systemStatus = "TOO_HOT";
  }
  else if (humidity < HUMIDITY_LOW_LIMIT) {
    systemStatus = "LOW_HUMIDITY";
  }
  else if (humidity > HUMIDITY_HIGH_LIMIT) {
    systemStatus = "HIGH_HUMIDITY";
  }
  else {
    systemStatus = "NORMAL";
  }
}

// ================= AUTOMATION =================

void updateAutomation() {

  controlIrrigation();     // seule la pompe est automatique
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
    return;   // message inconnu, on ignore
  }

  // ----- LED TEMPERATURE -----
  if (String(topic) == TOPIC_TEMP_LED_CMD) {

    temperatureLedOn = newState;
    digitalWrite(TEMP_LED_PIN, temperatureLedOn ? HIGH : LOW);

    mqttClient.publish(TOPIC_TEMP_LED, temperatureLedOn ? "ON" : "OFF");
  }

  // ----- LED HUMIDITE -----
  else if (String(topic) == TOPIC_HUMIDITY_LED_CMD) {

    humidityLedOn = newState;
    digitalWrite(HUMIDITY_LED_PIN, humidityLedOn ? HIGH : LOW);

    mqttClient.publish(TOPIC_HUMIDITY_LED, humidityLedOn ? "ON" : "OFF");
  }
}

// ================= MQTT CONNECTION =================

void reconnectMQTT() {

  if (mqttClient.connected()) {
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

 if (mqttClient.connect(clientId.c_str(), MQTT_USER, MQTT_PASS)) { {

    Serial.println("CONNECTED!");

    mqttClient.subscribe(TOPIC_TEMP_LED_CMD);
    mqttClient.subscribe(TOPIC_HUMIDITY_LED_CMD);

    mqttClient.publish(TOPIC_INFO, "ESP32 Agriculture System connected");
  }
  else {

    Serial.print("FAILED, state = ");
    Serial.println(mqttClient.state());
  }
}

// ================= PUBLISH =================

void publishSensorData() {

  if (!mqttClient.connected()) {
    return;
  }

  char tempBuffer[10];
  char humidityBuffer[10];
  char soilBuffer[10];

  snprintf(tempBuffer, sizeof(tempBuffer), "%.2f", temperature);
  snprintf(humidityBuffer, sizeof(humidityBuffer), "%.2f", humidity);
  snprintf(soilBuffer, sizeof(soilBuffer), "%d", soilPercent);

  // Capteurs
  mqttClient.publish(TOPIC_TEMP, tempBuffer);
  mqttClient.publish(TOPIC_HUMIDITY, humidityBuffer);
  mqttClient.publish(TOPIC_SOIL, soilBuffer);

  // Statut
  mqttClient.publish(TOPIC_STATUS, systemStatus.c_str());

  // Pompe (etat affiche dans Node-RED)
  mqttClient.publish(TOPIC_IRRIGATION, irrigationActive ? "ON" : "OFF");

  // LEDs (etat reel)
  mqttClient.publish(TOPIC_TEMP_LED, temperatureLedOn ? "ON" : "OFF");
  mqttClient.publish(TOPIC_HUMIDITY_LED, humidityLedOn ? "ON" : "OFF");

  // Alertes (selon les mesures)
  if (soilPercent < SOIL_DRY_LIMIT) {
    mqttClient.publish(TOPIC_ALERT, "SOIL_DRY");
  }
  else if (temperature > TEMP_HIGH_LIMIT) {
    mqttClient.publish(TOPIC_ALERT, "TEMPERATURE_HIGH");
  }
  else if (humidity < HUMIDITY_LOW_LIMIT || humidity > HUMIDITY_HIGH_LIMIT) {
    mqttClient.publish(TOPIC_ALERT, "HUMIDITY_ABNORMAL");
  }
}

// ================= WEB DASHBOARD =================

void handleRoot() {

  String html = R"rawliteral(

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

<h1>🌱 Smart Agriculture IoT</h1>

<div class="card"><h2>🌡 Temperature</h2>
<div class="value"><span id="temperature">--</span> °C</div></div>

<div class="card"><h2>💧 Air Humidity</h2>
<div class="value"><span id="humidity">--</span> %</div></div>

<div class="card"><h2>🌱 Soil Moisture</h2>
<div class="value"><span id="soil">--</span> %</div></div>

<div class="card"><h2>System Status</h2>
<div class="status"><span id="status">--</span></div></div>

<div class="card"><h2>🚿 Irrigation</h2>
<div class="status"><span id="irrigation">--</span></div></div>

<div class="card"><h2>Temperature LED</h2>
<div class="status"><span id="tempLed">--</span></div></div>

<div class="card"><h2>Humidity LED</h2>
<div class="status"><span id="humidityLed">--</span></div></div>

</div>

<script>
function updateData() {
  fetch('/data')
    .then(response => response.json())
    .then(data => {
      document.getElementById("temperature").innerText = data.temperature.toFixed(1);
      document.getElementById("humidity").innerText = data.humidity.toFixed(1);
      document.getElementById("soil").innerText = data.soil;
      document.getElementById("status").innerText = data.status;
      document.getElementById("irrigation").innerText = data.irrigation ? "ON" : "OFF";
      document.getElementById("tempLed").innerText = data.temperatureLed ? "ON" : "OFF";
      document.getElementById("humidityLed").innerText = data.humidityLed ? "ON" : "OFF";
    })
    .catch(error => { console.log("Error:", error); });
}
setInterval(updateData, 2000);
updateData();
</script>

</body>
</html>

)rawliteral";

  server.send(200, "text/html", html);
}

// ================= JSON =================

void handleData() {

  String json = "{";

  json += "\"temperature\":" + String(temperature, 1);
  json += ",\"humidity\":" + String(humidity, 1);
  json += ",\"soil\":" + String(soilPercent);
  json += ",\"status\":\"" + systemStatus + "\"";
  json += ",\"irrigation\":" + String(irrigationActive ? "true" : "false");
  json += ",\"temperatureLed\":" + String(temperatureLedOn ? "true" : "false");
  json += ",\"humidityLed\":" + String(humidityLedOn ? "true" : "false");

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
  if (!aht.begin()) {
    Serial.println("ERROR: AHT20 not found!");
  } else {
    Serial.println("AHT20 OK!");
  }

  Serial.println("Initializing BMP280...");
  if (!bmp.begin(0x77)) {
    Serial.println("BMP280 not found");
  } else {
    Serial.println("BMP280 OK!");
  }

  Serial.println();
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi CONNECTED!");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();

  Serial.println("Web server started!");

  reconnectMQTT();

  Serial.println();
  Serial.println("SYSTEM READY!");
}

// ================= LOOP =================

void loop() {

  server.handleClient();
  mqttClient.loop();
  reconnectMQTT();

  unsigned long now = millis();

  if (now - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = now;
    readSensors();
    updateAutomation();
  }

  if (now - lastMQTTPublish >= MQTT_INTERVAL) {
    lastMQTTPublish = now;
    publishSensorData();
  }
}