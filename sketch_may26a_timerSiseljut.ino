#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <FS.h>
#include <Wire.h>
#include <RTClib.h>
#include <ArduinoJson.h>
#include <Adafruit_BME280.h>
#include "HX711.h"
#include <ArduinoOTA.h>

#define RTC_SDA D6  // Using D6 as SDA (GPIO12)
#define RTC_SCL D5  // Using D5 as SCL (GPIO14)
// Pin HX710B definitions
#define DOUT D2  
#define SCK  D1  
#define MOSFET_PIN D7
// Wi-Fi and MQTT credentials
const char* ssid = "SISELJUT";
const char* password = "setubabakan";

// Emqx
const char* mqtt_server = "";
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

bool offlineDataSent = false;

HX711 scale;

// known calibration points:
const long RAW_PTS[] = { 1260000, 1520000, 1790000, 1970000 };
const float CM_PTS[]  = {      0.0,      5.0,     10.0,     15.0 };
const int  N_PTS     = 4;

// After 15 cm (RAW ≥ 1970000), every +270000 raw → +5 cm
const long  RAW_STEP = 270000;
const float CM_STEP  = 5.0;

unsigned long lastPublishTime = 0;
const unsigned long PUBLISH_INTERVAL = 3000UL; // Publish data every 3 seconds

void setup() {
  Serial.begin(9600);
  
  // Initialize LED pins
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  pinMode(MOSFET_PIN, OUTPUT);
  digitalWrite(MOSFET_PIN, LOW); // Ensure MOSFET is OFF at startup

  // Wi-Fi setup
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  WiFi.mode(WIFI_STA);
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

  scale.begin(DOUT, SCK);
  scale.set_scale();  // you can still apply a scale factor here if needed
  scale.tare();
  Serial.println("HX711 → cm converter ready");

  ArduinoOTA.setPassword("kelompok2");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {  // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("No WiFi");
    if (!reconnectWiFi()) {
      offlineMode();
    }
  }

  if (WiFi.status() == WL_CONNECTED){
    Serial.println("WiFi connected");
    ArduinoOTA.begin();
    ArduinoOTA.handle();
    syncTime();  // X.509 validation requires synchronization time
    if (!client.connected()) {
      reconnectMQTT();
      delay(2000);
      WiFi.mode(WIFI_STA);
      WiFi.begin(ssid, password);
    }
  }

  while (client.connected()) {
    client.loop();
    unsigned long currentMillisLoop = millis(); // For non-blocking tasks like publishing
    if (currentMillisLoop - lastPublishTime >= PUBLISH_INTERVAL) {
      publishOnline();
      lastPublishTime = currentMillisLoop;
    }
    ArduinoOTA.handle();
    handleRtcMosfetSchedule();
  }
}

bool syncTime() {
  configTime(gmt_offset_sec, daylight_offset_sec, ntp_server);
  Serial.println("Waiting for NTP time sync: ");

  const int maxRetries = 3;
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
  WiFi.reconnect();
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
  Serial.println("Connecting to MQTT...");
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("No WiFi");
    return;
  }
  BearSSL::X509List serverTrustedCA(ca_cert);
  espClient.setTrustAnchors(&serverTrustedCA);
  if (client.connect("ESP8266Lucky", mqtt_user, mqtt_pass)) {
    Serial.print("MQTT connected! ");
    client.publish("luckyharvi/status/1101", "online");
    sendOfflineData();
  } else {
    Serial.println("MQTT connection failed.");
    WiFi.mode(WIFI_OFF);
    delay(100);
  }
}

void publishOnline() {
  // Read sensor values from BME280 
  float temperature = bme.readTemperature(); // Read temperature in Celsius
  float humidity = bme.readHumidity(); // Read humidity in %

  long raw = scale.read();
  float dist = rawToCm(raw);

  // Create JSON object
  StaticJsonDocument<200> jsonDoc;
  jsonDoc["pressure"] = raw; // Raw reading
  jsonDoc["distance"] = dist; // Mapped to altitude reading (cm)
  jsonDoc["humidity"] = humidity;
  jsonDoc["temperature"] = temperature;

  char buffer[128];
  serializeJson(jsonDoc, buffer);

  client.publish("luckyharvi/online", buffer);
  Serial.print("Published:");
  Serial.println(buffer);
}

void offlineMode() {
  Serial.println("Failed to connect WiFi, running offline mode.");

  while (WiFi.status() != WL_CONNECTED) {
    DateTime now = rtc.now();  // Get the current time from DS3231
    // Get the Unix timestamp (numeric value)
    unsigned long timestamp = now.unixtime();

    // Read sensor values from BME280
    float temperature = bme.readTemperature(); // Read temperature in Celsius
    float humidity = bme.readHumidity(); // Read humidiy in %
    
    long raw = scale.read();
    String rawStr = String(raw);  
    float dist = rawToCm(raw);
    //float rawUnits = scale.get_units(5);     // take average of 5 readings

    // JSON data
    String jsonData = "{\"timestamp\":" + String(timestamp) +
                 ",\"pressure\":" + rawStr +
                 ",\"distance\":" + String(dist, 2) +
                 ",\"humidity\":" + String(humidity, 2) +
                 ",\"temperature\":" + String(temperature, 2) + "}";

    saveDataToSPIFFS(jsonData);
    Serial.println("Saved offline:" + jsonData);
    delay(5000);
    handleRtcMosfetSchedule();
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

float interp(long x0, float y0, long x1, float y1, long x) {
  return y0 + (float)(y1 - y0) * (x - x0) / (float)(x1 - x0);
}

float rawToCm(long raw) {
  // 1) If below first point, clamp to 0cm
  if (raw <= RAW_PTS[0]) return CM_PTS[0];

  // 2) Between any two known points → linear interpolate
  for (int i = 0; i < N_PTS - 1; i++) {
    if (raw <= RAW_PTS[i+1]) {
      return interp(RAW_PTS[i], CM_PTS[i],
                    RAW_PTS[i+1], CM_PTS[i+1],
                    raw);
    }
  }

  // 3) Beyond last calibration point → extrapolate in steps
  long extra = raw - RAW_PTS[N_PTS-1];
  float   cmExtra = ((float)extra / RAW_STEP) * CM_STEP;
  return CM_PTS[N_PTS-1] + cmExtra;
}

// MOSFET Control Variables (RTC based)
struct MosfetTrigger {
  int hour;
  int minute;
  bool triggeredToday; // Flag to ensure it triggers only once per day per slot
};

// Define a macro to create one entry:
#define ENTRY(h, m) { h, m, false }

// Define a macro to emit all four entries for a given hour:
#define HOUR_ENTRIES(h) \
  ENTRY(h, 0),  ENTRY(h, 10),  ENTRY(h, 30),  ENTRY(h, 40)

// Now build the full array by invoking HOUR_ENTRIES for each hour 0–23:
MosfetTrigger mosfetSchedule[] = {
  HOUR_ENTRIES(0),
  HOUR_ENTRIES(1),
  HOUR_ENTRIES(2),
  HOUR_ENTRIES(3),
  HOUR_ENTRIES(4),
  HOUR_ENTRIES(5),
  HOUR_ENTRIES(6),
  HOUR_ENTRIES(7),
  HOUR_ENTRIES(8),
  HOUR_ENTRIES(9),
  HOUR_ENTRIES(10),
  HOUR_ENTRIES(11),
  HOUR_ENTRIES(12),
  HOUR_ENTRIES(13),
  HOUR_ENTRIES(14),
  HOUR_ENTRIES(15),
  HOUR_ENTRIES(16),
  HOUR_ENTRIES(17),
  HOUR_ENTRIES(18),
  HOUR_ENTRIES(19),
  HOUR_ENTRIES(20),
  HOUR_ENTRIES(21),
  HOUR_ENTRIES(22),
  HOUR_ENTRIES(23)
};
#undef ENTRY
#undef HOUR_ENTRIES

const int numMosfetTriggers = sizeof(mosfetSchedule) / sizeof(mosfetSchedule[0]);
int lastDayProcessedForMosfet = 0; // To track the day for resetting flags (0 is an invalid day)

void handleRtcMosfetSchedule() {
  DateTime now = rtc.now();

  // --- Daily Reset of triggeredToday flags ---
  if (now.day() != lastDayProcessedForMosfet) {
    Serial.print(now.day());
    Serial.print("/");
    Serial.print(now.month());
    Serial.print(" New day. Resetting MOSFET trigger flags for '");

    for (int i = 0; i < numMosfetTriggers; i++) {
      mosfetSchedule[i].triggeredToday = false;
    }
    lastDayProcessedForMosfet = now.day(); // Update the last processed day
  }

  // --- Check Schedule and Trigger MOSFET ---
  for (int i = 0; i < numMosfetTriggers; i++) {
    // Check if this specific time slot hasn't been triggered today AND
    // if the current RTC hour and minute match the scheduled hour and minute.
    auto &slot = mosfetSchedule[i];
    if (!slot.triggeredToday &&
        now.hour()   == slot.hour &&
        now.minute() == slot.minute) {
      
      // Trigger MOSFET (e.g., ON for 1 seconds)
      digitalWrite(MOSFET_PIN, HIGH);
      delay(1000); // Keep it on for 1 seconds (blocking)
      digitalWrite(MOSFET_PIN, LOW);
      mosfetSchedule[i].triggeredToday = true; // Mark as triggered for this slot today

      // Toggle Wi-Fi based on minute
      if (slot.minute == 0 || slot.minute == 30) {
        // Turn Wi-Fi ON
        Serial.printf("%02d:%02d - WiFi ON by schedule\n", slot.hour, slot.minute);
        WiFi.mode(WIFI_STA);
        WiFi.begin(ssid, password);
      } else if (slot.minute == 10 || slot.minute == 40) {
        // Turn Wi-Fi OFF
        Serial.printf("%02d:%02d - WiFi OFF by schedule\n", slot.hour, slot.minute);
        WiFi.mode(WIFI_OFF);
      }

      // Log MOSFET action
      Serial.printf("%02d:%02d:%02d - MOSFET pulse\n",
                    now.hour(), now.minute(), now.second());
    }
  }
}
