#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <FS.h>
#include <Wire.h>
#include <RTClib.h>
#include <ArduinoJson.h>
#include <Adafruit_BME280.h>

#define RTC_SDA D6  // Using D6 as SDA (GPIO12)
#define RTC_SCL D5  // Using D5 as SCL (GPIO14)

// Wi-Fi and MQTT credentials
const char* ssid = "";
const char* password = "";

// Emqx
const char* mqtt_server = "....emqxsl.com";
const char* mqtt_user = "";
const char* mqtt_pass = "";

// NTP Server settings
const char *ntp_server = "pool.ntp.org";     // Default NTP server
const long gmt_offset_sec = 7 * 3600;            // GMT offset in seconds (GMT+7 for Jakarta)
const int daylight_offset_sec = 0;        // Daylight saving time offset in seconds

// WiFi and MQTT client initialization
BearSSL::WiFiClientSecure espClient;
PubSubClient client(espClient);

// SSL certificate for MQTT broker
// Load DigiCert Global Root CA ca_cert, which is used by EMQX Serverless Deployment
static const char ca_cert[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDrzCCApegAwIBAgIQCDvgVpBCRrGhdWrJWZHHSjANBgkqhkiG9w0BAQUFADBh
MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3
d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBD
QTAeFw0wNjExMTAwMDAwMDBaFw0zMTExMTAwMDAwMDBaMGExCzAJBgNVBAYTAlVT
MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j
b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IENBMIIBIjANBgkqhkiG
9w0BAQEFAAOCAQ8AMIIBCgKCAQEA4jvhEXLeqKTTo1eqUKKPC3eQyaKl7hLOllsB
CSDMAZOnTjC3U/dDxGkAV53ijSLdhwZAAIEJzs4bg7/fzTtxRuLWZscFs3YnFo97
nh6Vfe63SKMI2tavegw5BmV/Sl0fvBf4q77uKNd0f3p4mVmFaG5cIzJLv07A6Fpt
43C/dxC//AH2hdmoRBBYMql1GNXRor5H4idq9Joz+EkIYIvUX7Q6hL+hqkpMfT7P
T19sdl6gSzeRntwi5m3OFBqOasv+zbMUZBfHWymeMr/y7vrTC0LUq7dBMtoM1O/4
gdW7jVg/tRvoSSiicNoxBN33shbyTApOB6jtSj1etX+jkMOvJwIDAQABo2MwYTAO
BgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUA95QNVbR
TLtm8KPiGxvDl7I90VUwHwYDVR0jBBgwFoAUA95QNVbRTLtm8KPiGxvDl7I90VUw
DQYJKoZIhvcNAQEFBQADggEBAMucN6pIExIK+t1EnE9SsPTfrgT1eXkIoyQY/Esr
hMAtudXH/vTBH1jLuG2cenTnmCmrEbXjcKChzUyImZOMkXDiqw8cvpOp/2PV5Adg
06O/nVsJ8dWO41P0jmP6P6fbtGbfYmbW0W5BjfIttep3Sp+dWOIrWcBAI+0tKIJF
PnlUkiaY4IBIqDfv8NZ5YBberOgOzW6sRBc4L0na4UU+Krk2U886UAb3LujEV0ls
YSEY1QSteDwsOoBrp+uvFRTp2InBuThs4pFsiv9kuXclVzDAGySj4dzp30d8tbQk
CAUw7C29C79Fv1C5qfPrmAESrciIxpg0X40KPMbp1ZWVbd4=
-----END CERTIFICATE-----
)EOF";

RTC_DS3231 rtc;

Adafruit_BME280 bme; // I2C, Instantiate the BME280 sensor object

String serial_number = "1101";
bool offlineDataSent = false;
unsigned long lastReconnectAttempt = 0;

float P0_calibrated;

void setup() {
  Serial.begin(9600);
  
  // Initialize LED pins
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Wi-Fi setup
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  WiFi.begin(ssid, password);
  
  // EMQX
  client.setServer(mqtt_server, 8883);

  SPIFFS.begin();

  // Initialize I2C on custom pins
  Wire.begin(RTC_SDA, RTC_SCL);

  // Initialize RTC DS3231
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  Serial.println("RTC is online!");

  // Initialize BME280 sensor
  if (!bme.begin(0x76)) {  // Address 0x76 is common; some modules use 0x77.
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  // Optional: Configure sensor settings
  bme.setSampling(
    Adafruit_BME280::MODE_NORMAL,
    Adafruit_BME280::SAMPLING_X2,   // Temperature
    Adafruit_BME280::SAMPLING_X16,  // Pressure
    Adafruit_BME280::SAMPLING_X1,   // Humidity
    Adafruit_BME280::FILTER_X16, 
    Adafruit_BME280::STANDBY_MS_500
  );

  while (!Serial) {}
  // Wait for and read a full line
  while (Serial.available() == 0) {
    delay(3000);
    Serial.println("Enter true elevation in meters:");
  }

  String line = Serial.readStringUntil('\n');

  float P_meas = bme.readPressure() / 100.0F;  // current pressure in hPa
  Serial.print("Measured P: ");
  Serial.println(P_meas);

  float trueAltitude = line.toFloat();
  float entered = trueAltitude;
  Serial.print("True elevation: ");
  Serial.println(entered);

  P0_calibrated = P_meas / pow(1.0 - trueAltitude/44330.0, 1/0.1903);
  Serial.print("Calibrated P0: ");
  Serial.println(P0_calibrated);     // prints “Calibrated P0: 1006.30”
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("No WiFi");
    if (!reconnectWiFi()) {
      offlineMode();
    }
  }

  Serial.println("WiFi connected");
  syncTime();  // X.509 validation requires synchronization time
  if (!client.connected()) {
    reconnectMQTT();
    delay(2000);
  }
  
  while (client.connected()) {
    client.loop();
    publishOnline();
    delay(3000);
  }
}

bool syncTime() {
  configTime(gmt_offset_sec, daylight_offset_sec, ntp_server);
  Serial.println("Waiting for NTP time sync: ");

  const int maxRetries = 15;    // try for up to ~15 seconds
  int retryCount = 0;
  struct tm timeinfo;
  // Loop until getLocalTime succeeds or we hit maxRetries
  while (!getLocalTime(&timeinfo) && retryCount < maxRetries) {
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    Serial.print(".");
    retryCount++;
  }

  if (retryCount >= maxRetries) {
    Serial.println("[!] NTP sync failed!");
    return false;
  }

  Serial.println("synchronized: ");
  Serial.print(asctime(&timeinfo));

  DateTime currentTime(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                     timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  rtc.adjust(currentTime);
  Serial.print("RTC time: ");
  Serial.println(currentTime.timestamp());
  return true;
}

bool reconnectWiFi() {
  WiFi.begin(ssid, password);
  unsigned long startTime = millis();
  unsigned long lastDotTime = startTime;
  const unsigned long dotIntervalMs = 500;  // one dot every 500 ms
  Serial.println("Connecting to Wi-Fi...");
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000) {
    unsigned long now = millis();
    if (now - lastDotTime >= dotIntervalMs) {
      Serial.print(".");
      lastDotTime = now;
    }
    yield();  // keep background tasks alive
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Wi-Fi connected!");
    return true;
  }

  Serial.println("Wi-Fi connection failed.");
  return false;
}

void sendOfflineData() {
  // Check if the file exists and if offline data has not already been sent
  if (!SPIFFS.exists("/data.txt") || offlineDataSent) return;

  File file = SPIFFS.open("/data.txt", "r"); // Open file in read mode
  if (!file) {
    Serial.println("Failed to open file SPIFFS for reading.");
    return;
  }

  Serial.println("Sending saved data to MQTT");
  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      client.publish("luckyharvi/offline", line.c_str());
    } else {
      Serial.println("No data to send");
    }
  }
  file.close();
  SPIFFS.remove("/data.txt");
  offlineDataSent = true;
  Serial.println("Old data removed.");
}

void reconnectMQTT() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("No WiFi");
  }
  Serial.println("Connecting to MQTT...");
  BearSSL::X509List serverTrustedCA(ca_cert);
  espClient.setTrustAnchors(&serverTrustedCA);
  if (client.connect("ESP8266Lucky", mqtt_user, mqtt_pass)) {
    Serial.print("MQTT connected! ");
    digitalWrite(LED_BUILTIN, LOW);
    client.publish("luckyharvi/status/1101", "online");
    sendOfflineData();
  } else {
    Serial.println("MQTT connection failed.");
  }
}

void publishOnline() {
  // Read sensor values from BME280
  float pressure = bme.readPressure() / 100.0F; // Convert Pa to hPa 
  float temperature = bme.readTemperature(); // Read temperature in Celsius
  float humidity = bme.readHumidity(); // Read humidity in %
  float altitude = bme.readAltitude(P0_calibrated) * 100.0F; // Calculate altitude (in m) and convert to centimeters

  // Create JSON object
  StaticJsonDocument<200> jsonDoc;
  jsonDoc["pressure"] = pressure; // Mapped to pressure reading
  jsonDoc["distance"] = altitude; // Mapped to altitude reading (cm)
  jsonDoc["humidity"] = humidity;
  jsonDoc["temperature"] = temperature;

  char buffer[128];
  serializeJson(jsonDoc, buffer);

  client.publish("luckyharvi/online", buffer);
  Serial.print("Published: ");
  Serial.println(buffer);
}

void offlineMode() {
  Serial.println("Failed to connect WiFi, running offline mode.");

  while (WiFi.status() != WL_CONNECTED) {
    DateTime now = rtc.now();  // Get the current time from DS3231
    // Get the Unix timestamp (numeric value)
    unsigned long timestamp = now.unixtime();

    // Read sensor values from BME280
    float pressure = bme.readPressure() / 100.0F; // Convert Pa to hPa 
    float temperature = bme.readTemperature(); // Read temperature in Celsius
    float humidity = bme.readHumidity(); // Read humidiy in %
    float altitude = bme.readAltitude(P0_calibrated) * 100.0F; // Calculate altitude (in m) and convert to centimeters
    
    // JSON data
    String jsonData = "{\"timestamp\": " + String(timestamp) +
                 ", \"pressure\": " + String(pressure, 2) +
                 ", \"distance\": " + String(altitude, 2) +
                 ", \"humidity\": " + String(humidity, 2) +
                 ", \"temperature\": " + String(temperature, 2) + "}";

    saveDataToSPIFFS(jsonData);
    Serial.println("Saved offline: " + jsonData);
    delay(5000);
  }

  offlineDataSent = false;
}

void saveDataToSPIFFS(String data) {
  if (data.length() == 0) {
    Serial.println("[ERROR] No data to save.");
    return;
  }

  // Open file in append mode for adding new data
  File file = SPIFFS.open("/data.txt", "a");
  if (!file) {
    Serial.println("Failed to open SPIFFS for writing.");
    return;
  }

  file.println(data);
  file.close();
}
