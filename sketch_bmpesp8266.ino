#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <FS.h>
#include <Wire.h>
#include <RTClib.h>
#include <ArduinoJson.h>
#include <time.h>
#include <Adafruit_BMP280.h>

#define RTC_SDA D6  // Using D6 as SDA (GPIO12)
#define RTC_SCL D5  // Using D5 as SCL (GPIO14)

// Wi-Fi and MQTT credentials
const char* ssid = "ACER";
const char* password = "bucabuca";

// Emqx
const char* mqtt_server = "f225b61b.ala.asia-southeast1.emqxsl.com";
const char* mqtt_user = "luckyharvi";
const char* mqtt_pass = "12345";

// NTP Server settings
const char *ntp_server = "pool.ntp.org";     // Default NTP server
// const char* ntp_server = "cn.pool.ntp.org"; // Recommended NTP server for users in China
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

Adafruit_BMP280 bmp; // I2C, Instantiate the BMP280 sensor object

String serial_number = "1101";
bool offlineDataSent = false;
unsigned long lastReconnectAttempt = 0;

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

  // Initialize BMP280 sensor
  if (!bmp.begin(0x76)) {  // Address 0x76 is common; some modules use 0x77.
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }

  // Optional: configure BMP280 settings if desired
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,   // temperature oversampling
                  Adafruit_BMP280::SAMPLING_X16,  // pressure oversampling
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_BUILTIN, HIGH);
    reconnectWiFi();
    offlineMode();
  }

  if (WiFi.status() == WL_CONNECTED) {
    syncTime();  // X.509 validation requires synchronization time
    if (!client.connected()) {
      reconnectMQTT();
      delay(2000);
    }
    client.loop();
    while (client.connected()) {
      publishOnline();
      delay(3000);
    }
  }  
}

void syncTime() {
  configTime(gmt_offset_sec, daylight_offset_sec, ntp_server);
  Serial.print("Waiting for NTP time sync: ");
  while (time(nullptr) < 8 * 3600 * 2) {
      digitalWrite(LED_BUILTIN, LOW);
      delay(1000);
      digitalWrite(LED_BUILTIN, HIGH);
      Serial.print(".");
  }
  Serial.println("Time synchronized");
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
      Serial.print("Current time: ");
      Serial.println(asctime(&timeinfo));
      DateTime currentTime(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                         timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
      rtc.adjust(currentTime);
      DateTime now = rtc.now();
      Serial.print("RTC time: ");
      Serial.println(now.timestamp());
  } else {
      Serial.println("Failed to obtain local time");
  }
}

void reconnectWiFi() {
  Serial.print("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000) {
    Serial.print(".");
    yield(); // Allow background tasks (prevents WDT reset)
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWi-Fi connected!");
  } else {
    Serial.println("\nWi-Fi connection failed.");
  }
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
  Serial.print("Connecting to MQTT...");
  BearSSL::X509List serverTrustedCA(ca_cert);
  espClient.setTrustAnchors(&serverTrustedCA);
  if (client.connect("ESP8266Lucky", mqtt_user, mqtt_pass)) {
    Serial.println("\nMQTT connected!");
    digitalWrite(LED_BUILTIN, LOW);
    client.publish("luckyharvi/status/1101", "online");
    sendOfflineData();
  } else {
    Serial.println("\nMQTT connection failed.");
  }
}

void publishOnline() {
  // Read sensor values from BMP280
  float pressure = bmp.readPressure() / 100.0F; // Convert Pa to hPa 
  float temperature = bmp.readTemperature(); // Read temperature in Celsius
  float altitude = bmp.readAltitude(1013.25) * 100.0F; // Calculate altitude (in m) and convert to centimeters

  // Create JSON object
  StaticJsonDocument<200> jsonDoc;
  jsonDoc["pressure"] = pressure; // Mapped to pressure reading
  jsonDoc["distance"] = altitude; // Mapped to altitude reading (cm)
  jsonDoc["temperature"] = temperature;

  char buffer[256];
  serializeJson(jsonDoc, buffer);

  client.publish("luckyharvi/online", buffer);
  Serial.print("Published: ");
  Serial.println(buffer);
}

void offlineMode() {
  Serial.println("\nFailed to connect WiFi. Running offline mode.");

  while (WiFi.status() != WL_CONNECTED) {
    DateTime now = rtc.now();  // Get the current time from DS3231
    // Get the Unix timestamp (numeric value)
    unsigned long timestamp = now.unixtime();

    // Read sensor values from BMP280
    float pressure = bmp.readPressure() / 100.0F; // Convert Pa to hPa 
    float temperature = bmp.readTemperature(); // Read temperature in Celsius
    float altitude = bmp.readAltitude(1013.25) * 100.0F; // Calculate altitude (in m) and convert to centimeters
    
    // JSON data
    String jsonData = "{\"timestamp\": " + String(timestamp) +
                 ", \"pressure\": " + String(pressure, 2) +
                 ", \"distance\": " + String(altitude, 2) +
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
