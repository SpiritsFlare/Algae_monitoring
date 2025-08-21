// --- Standard & Library Includes ---
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Arduino.h>
#include "config.h"
#include <SparkFun_SCD30_Arduino_Library.h>

// --- Pin Definitions ---
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5

// --- I2C Addresses ---
#define SCD30_DEFAULT_ADDRESS 0x61
#define TCA9548A_ADDRESS 0x70

// --- Global Variables ---
float CO2_upload[6];
float Temp_upload[6];
unsigned long Time_upload;

unsigned long lastMillis = 0;
const unsigned long MEASURE_INTERVAL = 30000;
unsigned long lastMeasureTime = 0;

int sensorErrors[6] = {0, 0, 0, 0, 0, 0};
int consecutiveMQTTErrors = 0;
unsigned long lastSuccessfulReading = 0;
const int MAX_SENSOR_ERRORS = 5;
const int MAX_MQTT_ERRORS = 10;
const unsigned long WATCHDOG_TIMEOUT = 120000; 
float prevCO2[6] = { -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f};
bool validReadingAvailable[6] = {false, false, false, false, false, false};
unsigned long totalMQTTErrors = 0;

unsigned long lastI2CReset = 0;
const unsigned long I2C_RESET_INTERVAL = 86400000; // 30 min

unsigned long lastWatchdogFeed = 0;
unsigned long lastLoopTime = 0;

unsigned long lastMemoryCheck = 0;
const unsigned long MEMORY_CHECK_INTERVAL = 60000; 
int startupFreeHeap = 0;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
SCD30 airSensor[6];
StaticJsonDocument<2048> jsonDoc;

// --- Watchdog functions ---
void feedWatchdog() { lastWatchdogFeed = millis(); }

void checkSystemFreeze() {
  unsigned long currentTime = millis();
  if (currentTime - lastLoopTime > 180000) rp2040.restart(); // 3 min freeze
  if (currentTime - lastWatchdogFeed > 60000) rp2040.restart(); // missed feed
}

bool tcaSelect(uint8_t i) {
    if (i > 7) return false; // only 0–7 valid
    Wire.beginTransmission(TCA9548A_ADDRESS);
    Wire.write(1 << i);
    return (Wire.endTransmission() == 0);
}

// --- Memory health ---
void checkMemoryHealth() {
  unsigned long currentTime = millis();
  if (currentTime - lastMemoryCheck >= MEMORY_CHECK_INTERVAL) {
    lastMemoryCheck = currentTime;
    int currentHeap = rp2040.getFreeHeap();
    int heapUsed = startupFreeHeap - currentHeap;

    if (currentHeap < 3000) rp2040.restart();   // Critical
    else if (currentHeap < 6000) { /* warning only */ }

    if (heapUsed > 50000) rp2040.restart();     // Leak
  }
}

// --- I2C recovery ---
void recoverI2C() {
  Wire.end();
  delay(100);
  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();
  Wire.setTimeout(1000);
  Wire.setClock(100000);
  delay(1000);
  Wire.beginTransmission(TCA9548A_ADDRESS);
  delay(1000);
  Wire.write(0);
  Wire.endTransmission();
  delay(20);
}

// --- Health check ---
void systemHealthCheck() {
  unsigned long currentTime = millis();
  if (currentTime - lastSuccessfulReading > WATCHDOG_TIMEOUT) {
    recoverI2C();
    for (int i = 0; i < 6; i++) sensorErrors[i] = 0;
    lastSuccessfulReading = currentTime;
    delay(30000);
    if (millis() - lastSuccessfulReading > WATCHDOG_TIMEOUT + 30000) {
      rp2040.restart();
    }
  }
  if (currentTime - lastI2CReset > I2C_RESET_INTERVAL) {
    recoverI2C();
    lastI2CReset = currentTime;
  }
  if (consecutiveMQTTErrors >= MAX_MQTT_ERRORS) {
    mqttClient.disconnect();
    delay(5000);
    if (!mqttClient.connected() && !mqttClient.connect("Pi_Pico_Algae", ACCESS_TOKEN, NULL)) {
      rp2040.restart();
    }
    consecutiveMQTTErrors = 0;
  }
}

// --- MQTT ---
bool connectToMQTT() {
  static int totalMQTTFailures = 0;
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    unsigned long wifiStart = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 8000) {
      delay(500);
      yield();
    }
    if (WiFi.status() != WL_CONNECTED) {
      totalMQTTFailures++;
      if (totalMQTTFailures > 20) rp2040.restart();
      return false;
    }
  }
  mqttClient.setServer(THINGSBOARD_CLIENT, THINGSBOARD_PORT);
  if (mqttClient.connect("Pi_Pico_Algae", ACCESS_TOKEN, NULL)) {
    consecutiveMQTTErrors = 0;
    totalMQTTFailures = 0;
    return true;
  } else {
    consecutiveMQTTErrors++;
    totalMQTTErrors++;
    if (totalMQTTFailures > 50) rp2040.restart();
    delay(2000);
    return false;
  }
}

bool sendSensorData() {
  if (WiFi.status() != WL_CONNECTED) return false;
  if (!mqttClient.connected() && !connectToMQTT()) return false;

  jsonDoc.clear();
  bool hasValidData = false;
  for (int i = 0; i < 6; i++) {
    if (validReadingAvailable[i] && sensorErrors[i] < MAX_SENSOR_ERRORS && CO2_upload[i] > 0) {
      char co2Key[20], tempKey[20];
      sprintf(co2Key, "co2_sensor%d", i + 1);
      sprintf(tempKey, "temp_sensor%d", i + 1);
      jsonDoc[co2Key] = CO2_upload[i];
      jsonDoc[tempKey] = Temp_upload[i];
      hasValidData = true;
    }
  }
  jsonDoc["runtime_seconds"] = millis() / 1000;

  if (!hasValidData) return false;
  static char payload[2048];
  size_t payloadSize = serializeJson(jsonDoc, payload, sizeof(payload));
  bool success = mqttClient.publish("v1/devices/me/telemetry", payload);
  if (!success) consecutiveMQTTErrors++;
  return success;
}

// --- Sensor read ---
bool readSensorWithRetry(int sensorIndex, int maxRetries = 1) {  // reduced retries
  for (int attempt = 0; attempt < maxRetries; attempt++) {
    if (!tcaSelect(sensorIndex + 1)) continue;
    Wire.beginTransmission(SCD30_DEFAULT_ADDRESS);
    if (Wire.endTransmission() != 0) continue;
    unsigned long waitStart = millis();
    while (!airSensor[sensorIndex].dataAvailable() && millis() - waitStart < 5000) {
      delay(50);
      yield();
    }
    if (!airSensor[sensorIndex].dataAvailable()) continue;

    float co2Reading = airSensor[sensorIndex].getCO2();
    float tempReading = airSensor[sensorIndex].getTemperature();
    if (co2Reading <= 0 || co2Reading > 10000) continue;
    if (tempReading < -20.0 || tempReading > 60.0) continue;

    if (prevCO2[sensorIndex] >= 0 && fabs(co2Reading - prevCO2[sensorIndex]) > 500) {
      if (attempt < maxRetries - 1) { delay(1000); continue; }
    }
    CO2_upload[sensorIndex] = co2Reading;
    Temp_upload[sensorIndex] = tempReading;
    prevCO2[sensorIndex] = co2Reading;
    return true;
  }
  return false;
}

void readAndUploadData() {
  unsigned long readingStartTime = millis();
  bool anySuccessfulReading = false;
  Time_upload = millis() / 1000;

  for (int i = 0; i < 6; i++) validReadingAvailable[i] = false;
  for (int i = 0; i < 6; i++) {
    if (millis() - readingStartTime > 25000) break;
    if (sensorErrors[i] >= MAX_SENSOR_ERRORS) continue;
    if (readSensorWithRetry(i)) {
      validReadingAvailable[i] = true;
      sensorErrors[i] = 0;
      anySuccessfulReading = true;
    } else {
      sensorErrors[i]++;
    }
    delay(100);
    yield();
  }
  if (anySuccessfulReading) {
    if (sendSensorData()) lastSuccessfulReading = millis();
  }
  feedWatchdog();
}

// --- Setup ---
void setup() {
  startupFreeHeap = rp2040.getFreeHeap();
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  mqttClient.setBufferSize(2048);
  mqttClient.setKeepAlive(60);

  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();
  Wire.setTimeout(1000);
  Wire.setClock(100000);

  const int MAX_INIT_RETRIES = 3; // Number of initialization retries
  const int RETRY_DELAY_MS = 500; // Delay between retries

  for (int i = 0; i < 6; i++) {
    bool initialized = false;
    for (int attempt = 0; attempt < MAX_INIT_RETRIES && !initialized; attempt++) {
      if (tcaSelect(i + 1)) { // Select TCA9548A channel
        if (airSensor[i].begin()) {
          airSensor[i].setMeasurementInterval(2);
          initialized = true;
          // Optional: Log success if Serial is available
          // Serial.printf("Sensor %d initialized on attempt %d\n", i + 1, attempt + 1);
        } else {
          // Optional: Log failure if Serial is available
          // Serial.printf("Sensor %d failed to initialize on attempt %d\n", i + 1, attempt + 1);
        }
      }
      if (!initialized && attempt < MAX_INIT_RETRIES - 1) {
        delay(RETRY_DELAY_MS); // Wait before retrying
      }
    }
    if (!initialized) {
      sensorErrors[i] = MAX_SENSOR_ERRORS; // Mark sensor as failed
      // Optional: Log permanent failure
      // Serial.printf("Sensor %d marked as failed after %d attempts\n", i + 1, MAX_INIT_RETRIES);
    }
  }

  connectToMQTT();
  lastMeasureTime = millis();
  feedWatchdog();
}

// --- Main Loop ---
void loop() {
  unsigned long currentTime = millis();
  lastLoopTime = currentTime;
  checkSystemFreeze();
  checkMemoryHealth();
  systemHealthCheck();
  if (currentTime - lastMeasureTime >= MEASURE_INTERVAL) {
    lastMeasureTime = currentTime;
    readAndUploadData();
  }
  if (mqttClient.connected()) mqttClient.loop();
  feedWatchdog();
  yield();
  delay(100);
}
