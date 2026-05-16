#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <NTPClient.h>
#include <ESP32Servo.h>
#include <Ticker.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <WiFiClientSecure.h>
#include <ESP32PWM.h> // <-- Use this instead!// Sometimes required by ESP32Servo for timer management

#define INA226_ADDRESS 0x40
#define INA226_REG_CONFIG 0x00
#define INA226_REG_SHUNT_VOLTAGE 0x01
#define INA226_REG_BUS_VOLTAGE 0x02
#define INA226_REG_POWER 0x03
#define INA226_REG_CURRENT 0x04
#define INA226_REG_CALIBRATION 0x05

#define CALIBRATION 512

#define JSON_DOC_SIZE 1024

// Configuration values
#define INA226_CONFIG_DEFAULT 0x4527
#define SHUNT_RESISTANCE 0.002  // Your module's built-in shunt
#define MAX_CURRENT 3.2768

// Electronic Load Configuration
#define PWM_PIN 25              // GPIO pin for MOSFET control
#define PWM_FREQ 25000          // 25kHz PWM frequency
#define PWM_RESOLUTION 8        // 8-bit resolution (0-255)

// BLE UUIDs
#define SERVICE_UUID        "91bad492-b950-4226-aa2b-4ede9fa42f59"
#define WIFI_CRED_UUID      "cba1d466-344c-4be3-ab3f-189f80dd7518"
#define IP_ADDR_UUID        "12345678-1234-5678-1234-56789abcdef0" 

Preferences preferences;
WebServer server(80);
WiFiUDP udp;
NTPClient timeClient(udp, "pool.ntp.org", 0, 3600); // Will adjust offset later

Servo myservo;
Ticker sunTrackingTicker;

String currentMode = "Closed";
int currentDegree = 90;
const int minDegree = 0;
const int maxDegree = 180;

bool wifiConnected = false;

// Global variables for measurements
float voltage = 0;
float current = 0;
float power = 0;
float energy = 0;
unsigned long lastTime = 0;
unsigned long startTime = 0;

// MPPT variables
int currentPWM = 0;
bool mpptEnabled = false;
float maxPowerVoltage = 0;
float maxPowerCurrent = 0;
float maxPower = 0;
int optimalPWM = 0;

BLEServer* pServer = nullptr;
BLECharacteristic* wifiCredCharacteristic = nullptr;
BLECharacteristic* ipAddrCharacteristic = nullptr;
void connectToWiFi(const char* ssid, const char* password);

// --- Pipedream & Location Settings ---
const char* PIPEDREAM_URL = "https://eogaas8mnfwdl7z.m.pipedream.net";

// Your Location (Used for both Sun tracking and Data logging)
const float DEVICE_LAT = 30.030474;
const float DEVICE_LNG = -90.241234;

// Upload Timer (e.g., 5 minutes = 300000 ms)
unsigned long lastUploadTime = 0;
const unsigned long UPLOAD_INTERVAL = 300000;

// --- Sun Tracking Memory ---
int sunriseMinutes = 360;  // Defaults to 6:00 AM (6 * 60)
int sunsetMinutes = 1080;  // Defaults to 6:00 PM (18 * 60)
bool hasRealSunData = false;

// Helper function to convert "6:32:14 AM" into total minutes from midnight
int timeStringToMinutes(String timeStr) {
  int h, m, s;
  char ampm[3];
  // Parse the string into hours, minutes, seconds, and AM/PM
  sscanf(timeStr.c_str(), "%d:%d:%d %2s", &h, &m, &s, ampm);
  
  if (strcmp(ampm, "PM") == 0 && h != 12) h += 12; // Convert PM to 24-hour time
  if (strcmp(ampm, "AM") == 0 && h == 12) h = 0;   // Handle midnight
  
  return (h * 60) + m;
}

class WiFiCredsCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    std::string value = std::string(pCharacteristic->getValue().c_str());
    Serial.print("Received BLE data: ");
    Serial.println(value.c_str());

    if (value.length() > 0) {
      const size_t capacity = JSON_OBJECT_SIZE(2) + 60;
      StaticJsonDocument<capacity> doc;
      DeserializationError error = deserializeJson(doc, value.c_str());

      if (error) {
        Serial.print("JSON parsing failed: ");
        Serial.println(error.c_str());
        return;
      }

      String ssid = String((const char*)doc["ssid"]);
      String password = String((const char*)doc["password"]);

      Serial.println("Parsed SSID: " + ssid);
      Serial.println("Parsed Password: " + password);

      preferences.begin("wifi", false);
      preferences.putString("ssid", ssid);
      preferences.putString("password", password);
      preferences.end();

      connectToWiFi(ssid.c_str(), password.c_str());
    }
  }
};

void connectToWiFi(const char* ssid, const char* password) {
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi...");
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 20) {
    delay(1000);
    Serial.print(".");
    retries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    wifiConnected = true;

    // Update IP BLE characteristic and notify
    if (ipAddrCharacteristic) {
      String ipStr = WiFi.localIP().toString();
      Serial.println(ipStr);
      ipAddrCharacteristic->setValue(ipStr.c_str());
      ipAddrCharacteristic->notify();
      Serial.println("Updated IP address characteristic");
    }

    startWebServer();
  } else {
    Serial.println("\nFailed to connect to WiFi.");
    wifiConnected = false;
  }
}


void startBLEServer() {
  uint64_t chipid = ESP.getEfuseMac();
  char name[30];
  snprintf(name, sizeof(name), "ESP32-%04X", (uint16_t)(chipid >> 32));
  BLEDevice::init(name);

  pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);

  wifiCredCharacteristic = pService->createCharacteristic(
    WIFI_CRED_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  wifiCredCharacteristic->setCallbacks(new WiFiCredsCallback());

  ipAddrCharacteristic = pService->createCharacteristic(
    IP_ADDR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  ipAddrCharacteristic->addDescriptor(new BLE2902());

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();

  Serial.println("BLE advertising started");
}

void uploadToPipedream() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Skipping upload: WiFi not connected");
    return;
  }

  Serial.println("Preparing data upload...");

  // ── 1. TIME ─────────────────────────────────────────────────
  timeClient.update();
  unsigned long rawTime = timeClient.getEpochTime();
  time_t now = (time_t)rawTime;
  struct tm* ptm = gmtime(&now);

  char timeString[25];
  if (ptm == NULL || rawTime < 100000) {
    strcpy(timeString, "1970-01-01T00:00:00");
  } else {
    sprintf(timeString, "%04d-%02d-%02dT%02d:%02d:%02d",
            ptm->tm_year + 1900, ptm->tm_mon + 1, ptm->tm_mday,
            ptm->tm_hour, ptm->tm_min, ptm->tm_sec);
  }

  // ── 2. WEATHER  (Open-Meteo, no API key) ────────────────────
  // Mirrors the python fetch_weather() fields exactly
  float wx_temp        = NAN,  wx_apparent   = NAN;
  int   wx_humidity    = -1,   wx_cloud      = -1,  wx_code = -1;
  float wx_wind_speed  = NAN;
  int   wx_wind_dir    = -1;
  bool  weatherOk      = false;

  {
    char wxUrl[256];
    snprintf(wxUrl, sizeof(wxUrl),
      "https://api.open-meteo.com/v1/forecast"
      "?latitude=%.4f&longitude=%.4f"
      "&current=temperature_2m,relative_humidity_2m,apparent_temperature,"
      "weather_code,cloud_cover,wind_speed_10m,wind_direction_10m"
      "&wind_speed_unit=ms&timezone=UTC",
      DEVICE_LAT, DEVICE_LNG);

    String wxBody;
    int wxCode = httpsGet(wxUrl, wxBody);

    if (wxCode == 200) {
      StaticJsonDocument<512> wxDoc;
      if (!deserializeJson(wxDoc, wxBody)) {
        JsonObject cur = wxDoc["current"];
        wx_temp       = cur["temperature_2m"]       | NAN;
        wx_apparent   = cur["apparent_temperature"] | NAN;
        wx_humidity   = cur["relative_humidity_2m"] | -1;
        wx_cloud      = cur["cloud_cover"]          | -1;
        wx_wind_speed = cur["wind_speed_10m"]       | NAN;
        wx_wind_dir   = cur["wind_direction_10m"]   | -1;
        wx_code       = cur["weather_code"]         | -1;
        weatherOk = true;
        Serial.printf("Weather: %.1f°C, %d%% cloud, %d%% humidity\n",
                      wx_temp, wx_cloud, wx_humidity);
      }
    } else {
      Serial.printf("Weather fetch failed (HTTP %d)\n", wxCode);
    }
  }

  // ── 3. SUNRISE / SUNSET  (sunrise-sunset.org) ───────────────
  // Returns UTC times in ISO-8601 format, e.g. "6:32:14 AM"
  // Full response also includes solar_noon, day_length, etc.
  String ss_sunrise    = "unavailable";
  String ss_sunset     = "unavailable";
  String ss_solar_noon = "unavailable";
  String ss_day_length = "unavailable";
  bool   sunOk         = false;

  {
    char ssUrl[256]; // Increased buffer size for the longer URL
    snprintf(ssUrl, sizeof(ssUrl),
      "https://api.sunrise-sunset.org/json"
      "?lat=%.4f&lng=%.4f&formatted=0&tzid=America/Chicago",
      DEVICE_LAT, DEVICE_LNG);

    String ssBody;
    int ssCode = httpsGet(ssUrl, ssBody);

    if (ssCode == 200) {
      StaticJsonDocument<512> ssDoc;
      if (!deserializeJson(ssDoc, ssBody)) {
        const char* status = ssDoc["status"] | "UNKNOWN";
        if (strcmp(status, "OK") == 0) {
          ss_sunrise    = ssDoc["results"]["sunrise"].as<String>();
          ss_sunset     = ssDoc["results"]["sunset"].as<String>();
          ss_solar_noon = ssDoc["results"]["solar_noon"].as<String>();
          ss_day_length = ssDoc["results"]["day_length"].as<String>();
          sunOk = true;
          Serial.printf("Sunrise: %s  Sunset: %s\n",
                        ss_sunrise.c_str(), ss_sunset.c_str());
        }
      }
    } else {
      Serial.printf("Sunrise-sunset fetch failed (HTTP %d)\n", ssCode);
    }
  }

  // ── 4. BUILD JSON PAYLOAD ────────────────────────────────────
  // Structure mirrors the Python MongoDB document exactly:
  //   device_id / timestamp / readings / status /
  //   weather   / solar     / location / metadata
  StaticJsonDocument<JSON_DOC_SIZE> doc;

  doc["device_id"] = "solara_esp32_v1";
  doc["timestamp"] = timeString;

  // readings
  JsonObject readings = doc.createNestedObject("readings");
  readings["voltage_v"]              = voltage;
  readings["current_a"]              = current;
  readings["power_w"]                = power;
  readings["energy_wh"]              = energy;
  readings["pwm_value"]              = currentPWM;
  readings["mppt_status"]            = mpptEnabled ? "ON" : "OFF";
  
  // ADD THESE TWO LINES FOR YOUR SERVO DATA:
  readings["servo_mode"]             = currentMode; 
  readings["servo_degree"]           = currentDegree;

  // status
  const char* status = "active";
  if (voltage < 11.0)  status = "low_battery";
  else if (power > 60) status = "overheating";   // adjust threshold as needed
  doc["status"] = status;

  // weather  (mirrors Python fetch_weather() return dict)
  JsonObject weather = doc.createNestedObject("weather");
  if (weatherOk) {
    weather["temperature_c"]          = wx_temp;
    weather["apparent_temperature_c"] = wx_apparent;
    weather["humidity_pct"]           = wx_humidity;
    weather["cloud_cover_pct"]        = wx_cloud;
    weather["wind_speed_ms"]          = wx_wind_speed;
    weather["wind_direction_deg"]     = wx_wind_dir;
    weather["weather_code"]           = wx_code;
  } else {
    weather["error"] = "fetch_failed";
  }
  weather["source"] = "open-meteo.com";

  // solar  (new: sunrise-sunset.org data)
  JsonObject solar = doc.createNestedObject("solar");
  if (sunOk) {
    solar["sunrise"]    = ss_sunrise;
    solar["sunset"]     = ss_sunset;
    solar["solar_noon"] = ss_solar_noon;
    solar["day_length"] = ss_day_length;
  } else {
    solar["error"] = "fetch_failed";
  }
  solar["source"] = "sunrise-sunset.org";

  // location
  JsonObject location = doc.createNestedObject("location");
  location["latitude"]  = DEVICE_LAT;
  location["longitude"] = DEVICE_LNG;

  // metadata
  JsonObject metadata = doc.createNestedObject("metadata");
  metadata["firmware_version"] = "1.0.2";
  metadata["location"]         = "test_bench_01";

  // ── 5. SERIALIZE & POST ──────────────────────────────────────
  String payload;
  serializeJson(doc, payload);

  Serial.println("Sending payload...");
  Serial.println(payload);   // remove in production

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  http.begin(client, PIPEDREAM_URL);
  http.addHeader("Content-Type", "application/json");

  int httpResponseCode = http.POST(payload);

  if (httpResponseCode > 0) {
    Serial.printf("Data sent! HTTP %d\n", httpResponseCode);
  } else {
    Serial.printf("Upload failed: %s\n",
                  http.errorToString(httpResponseCode).c_str());
  }

  http.end();
}

int httpsGet(const char* url, String& out) {
  WiFiClientSecure client;
  client.setInsecure();          // same pattern as your existing POST

  HTTPClient http;
  http.begin(client, url);
  int code = http.GET();
  if (code > 0) out = http.getString();
  http.end();
  return code;
}

// void requestSunInfo() {
//   if (WiFi.status() != WL_CONNECTED) {
//     Serial.println("WiFi not connected!");
//     return;
//   }

//   WiFiClientSecure client;
//   client.setInsecure();  // Skip certificate validation (for testing only)
  
//   HTTPClient http;
  
//   // Use HTTPS URL with WiFiClientSecure
//   String sunUrl = "https://api.sunrise-sunset.org/json?lat=" + String(DEVICE_LAT, 6) + 
//                 "&lng=" + String(DEVICE_LNG, 6) + "&date=today&tzid=America/Chicago";
//   http.begin(client, sunUrl);
//   http.setTimeout(10000);
  
//   Serial.println("Making secure API request...");
//   int httpResponseCode = http.GET();
  
//   Serial.print("HTTPS Response Code: ");
//   Serial.println(httpResponseCode);
  
//   if (httpResponseCode > 0) {
//     String response = http.getString();
//     Serial.println("Response:");
//     Serial.println(response);
//   } else {
//     Serial.print("HTTPS request failed with error: ");
//     Serial.println(httpResponseCode);
//   }

//   http.end();
// }

void requestSunInfo() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected!");
    return;
  }

  WiFiClientSecure client;
  client.setInsecure();  
  HTTPClient http;
  
  String sunUrl = "https://api.sunrise-sunset.org/json?lat=" + String(DEVICE_LAT, 6) + 
                "&lng=" + String(DEVICE_LNG, 6) + "&date=today&tzid=America/Chicago";
                
  http.begin(client, sunUrl);
  http.setTimeout(10000);
  
  Serial.println("Fetching real sunrise/sunset data...");
  int httpResponseCode = http.GET();
  
  if (httpResponseCode == 200) {
    String response = http.getString();
    
    // Parse the JSON
    StaticJsonDocument<1024> doc;
    DeserializationError error = deserializeJson(doc, response);
    
    if (!error && doc["status"] == "OK") {
      String sunriseStr = doc["results"]["sunrise"].as<String>();
      String sunsetStr = doc["results"]["sunset"].as<String>();
      
      // Convert to minutes and save to global variables
      sunriseMinutes = timeStringToMinutes(sunriseStr);
      sunsetMinutes = timeStringToMinutes(sunsetStr);
      hasRealSunData = true;
      
      Serial.printf("✓ Sun Data Updated! Sunrise: %s, Sunset: %s\n", 
                    sunriseStr.c_str(), sunsetStr.c_str());
    } else {
      Serial.println("Failed to parse sun data JSON");
    }
  } else {
    Serial.printf("HTTPS request failed. Error: %d\n", httpResponseCode);
  }

  http.end();
}

void smoothServoSweep(int startAngle, int midAngle, int endAngle, int delayMs = 90) {
    // Move from start → mid
    if (startAngle < midAngle) {
        for (int pos = startAngle; pos <= midAngle; pos++) {
            myservo.write(pos);
            delay(delayMs);
        }
    } else {
        for (int pos = startAngle; pos >= midAngle; pos--) {
            myservo.write(pos);
            delay(delayMs);
        }
    }

    // Pause briefly at peak
    delay(300);

    // Move from mid → end
    if (midAngle < endAngle) {
        for (int pos = midAngle; pos <= endAngle; pos++) {
            myservo.write(pos);
            delay(delayMs);
        }
    } else {
        for (int pos = midAngle; pos >= endAngle; pos--) {
            myservo.write(pos);
            delay(delayMs);
        }
    }
}

void startWebServer() {
  timeClient.begin();
  timeClient.setTimeOffset(-5 * 3600);
  timeClient.update();

  requestSunInfo();

  // Print the formatted time (e.g., "14:49:00")
  Serial.print("Current Time: ");
  Serial.println(timeClient.getFormattedTime());

  myservo.attach(13);
  myservo.write(90);

  server.on("/", HTTP_GET, []() {
    server.send(200, "text/plain", "ESP32 is connected!");
  });

  server.on("/energy_data", []() {
    // Read fresh data
    readINA226();
    
    // Get shunt voltage for debugging
    int16_t shuntRaw = (int16_t)readRegister(INA226_REG_SHUNT_VOLTAGE);
    float shuntV = shuntRaw * 2.5 / 1000.0;  // in mV
    
    String json = "{";
    json += "\"voltage\":" + String(voltage, 3) + ",";
    json += "\"current\":" + String(current, 3) + ",";
    json += "\"power\":" + String(power, 3) + ",";
    json += "\"energy\":" + String(energy, 4) + ",";
    json += "\"shunt_voltage_mV\":" + String(shuntV, 3) + ",";  // ← NEW
    json += "\"pwm\":" + String(currentPWM) + ",";
    json += "\"pwm_percent\":" + String((currentPWM*100)/255) + ",";
    json += "\"mppt_enabled\":" + String(mpptEnabled ? "true" : "false") + ",";
    json += "\"max_power\":" + String(maxPower, 3) + ",";
    json += "\"optimal_pwm\":" + String(optimalPWM);
    json += "}";
    server.send(200, "application/json", json);
  });

  server.on("/reset_energy", []() {
    energy = 0;
    startTime = millis();
    server.send(200, "text/plain", "Energy counter reset");
  });

  server.on("/set_pwm", []() {
    if (server.hasArg("value")) {
      int pwm = server.arg("value").toInt();
      if (pwm >= 0 && pwm <= 255) {
        mpptEnabled = false;
        currentPWM = pwm;
        ledcWrite(PWM_PIN, currentPWM);
        String response = "PWM set to " + String(pwm) + " (" + String((pwm*100)/255) + "%)";
        server.send(200, "text/plain", response);
      } else {
        server.send(400, "text/plain", "PWM must be 0-255");
      }
    } else {
      server.send(400, "text/plain", "Missing 'value' parameter. Example: /set_pwm?value=128");
    }
  });

  server.on("/mppt_sweep", []() {
    performMPPTSweep();
    String response = "MPPT Sweep Complete!\n\n";
    response += "Optimal PWM: " + String(optimalPWM) + " (" + String((optimalPWM*100)/255) + "%)\n";
    response += "Max Power: " + String(maxPower, 3) + " W\n";
    response += "At Voltage: " + String(maxPowerVoltage, 3) + " V\n";
    response += "At Current: " + String(maxPowerCurrent, 3) + " A\n\n";
    if (maxPowerCurrent > 0.001) {
      float equivalentR = maxPowerVoltage / maxPowerCurrent;
      response += "Equivalent Load Resistance: " + String(equivalentR, 1) + " Ohms\n";
    }
    response += "\nCheck Serial Monitor for detailed sweep data.";
    server.send(200, "text/plain", response);
  });

  server.on("/mppt_enable", []() {
    mpptEnabled = true;
    server.send(200, "text/plain", "Auto MPPT tracking enabled");
  });

  server.on("/mppt_disable", []() {
    mpptEnabled = false;
    server.send(200, "text/plain", "Auto MPPT tracking disabled");
  });

  server.on("/reset_mppt", []() {
    maxPower = 0;
    maxPowerVoltage = 0;
    maxPowerCurrent = 0;
    optimalPWM = 0;
    server.send(200, "text/plain", "MPPT tracking data reset");
  });

  server.on("/servo/closed", []() {
    stopSunTracking();
    currentMode = "Closed";
    currentDegree = 90;
    myservo.write(90);
    Serial.println(currentDegree);
    server.send(200, "text/plain", "Servo is 90 degrees");
  });

  server.on("/servo/open", []() {
    stopSunTracking();
    currentMode = "Open";
    currentDegree = 135;
    myservo.write(135);
    server.send(200, "text/plain", "Servo is 135 degrees");
    
  });

  server.on("/wifi/reset", []() {
    resetWiFiCredentials();
    server.send(200, "text/plain", "WiFi credentials cleared. BLE provisioning active.");
  });

  server.on("/servo/demo", []() {
    stopSunTracking();
    currentMode = "Demo";

    // Smoothly sweep from 90 -> 135 -> 90
    smoothServoSweep(90, 130, 90);

    currentDegree = 0; // End position
    server.send(200, "text/plain", "Servo performed smooth demo sweep");
  });


  server.on("/servo/manual", HTTP_GET, []() {
    if (server.hasArg("degree")) {
      int degree = server.arg("degree").toInt();
      stopSunTracking();
      currentMode = "Manual";
      myservo.write(degree);
      Serial.println(String(degree));
      currentDegree = degree;
      server.send(200, "text/plain", "Servo set to " + String(degree));
    } else {
      server.send(400, "text/plain", "Degree not provided");
    }
  });

  server.on("/apicall", []() {
    uploadToPipedream();
  });

  server.on("/servo/mode", []() {
    server.send(200, "text/plain", currentMode);
  });

  server.on("/servo/degree", []() {
    server.send(200, "text/plain", String(currentDegree));
  });

  server.on("/servo/setManual", []() {
    stopSunTracking();
    currentMode = "Manual";
    server.send(200, "text/plain", "Manual mode set");
  });

  server.on("/servo/setSunTracking", []() {
    currentMode = "Sun Tracking";
    startSunTracking();
    server.send(200, "text/plain", "Sun Tracking mode set");
  });

  server.begin();
  Serial.println("HTTP server started");
}

void startSunTracking() {
  sunTrackingTicker.attach(60, updateSunPosition); 
}

void stopSunTracking() {
  sunTrackingTicker.detach(); 
}

// int calculateSunPosition() {
//   if (timeClient.update()) {
//     unsigned long epochTime = timeClient.getEpochTime();
//     struct tm* timeinfo = localtime((time_t*)&epochTime);
//     int hour = timeinfo->tm_hour;
//     int minute = timeinfo->tm_min;

//     const int totalMinutesInDay = 12 * 60;
//     int currentMinutes = (hour - 6) * 60 + minute;
//     if (currentMinutes < 0) currentMinutes = 0;
//     if (currentMinutes > totalMinutesInDay) currentMinutes = totalMinutesInDay;

//     float degreeRange = 150.0 - 30.0;
//     float position = 30.0 + (degreeRange * currentMinutes) / totalMinutesInDay;
//     return round(position);
//   } else {
//     Serial.println("NTP update failed");
//     return currentDegree;
//   }
// }

int calculateSunPosition() {
  if (timeClient.update()) {
    int localHour = timeClient.getHours(); 
    int localMinute = timeClient.getMinutes();
    int currentTotalMinutes = (localHour * 60) + localMinute;

    // --- NIGHT MODE (Before dawn OR after dusk) ---
    if (currentTotalMinutes < sunriseMinutes || currentTotalMinutes >= sunsetMinutes) {
      return 90; // Safely park at the closed position
    }

    // --- DAYLIGHT MODE (Tracking the sun) ---
    int currentMinutesOfSunlight = currentTotalMinutes - sunriseMinutes;
    int totalMinutesInDay = sunsetMinutes - sunriseMinutes;

    float startAngle = 90.0;
    float endAngle = 130.0;
    float degreeRange = endAngle - startAngle; 
    
    float position = startAngle + (degreeRange * currentMinutesOfSunlight) / totalMinutesInDay;
    
    return round(position);
  } else {
    Serial.println("NTP update failed");
    return currentDegree;
  }
}

void updateSunPosition() {
  if (currentMode == "Sun Tracking") {
    int targetPosition = calculateSunPosition();

    // If the jump is large (e.g., returning to 90 at sunset) -> Move slowly!
    if (abs(targetPosition - currentDegree) > 2) {
      Serial.printf("Smoothly transitioning servo from %d to %d...\n", currentDegree, targetPosition);
      
      if (currentDegree < targetPosition) {
        for (int pos = currentDegree; pos <= targetPosition; pos++) {
          myservo.write(pos);
          delay(80); // 80ms delay per degree = a gentle, slow sweep
        }
      } else {
        for (int pos = currentDegree; pos >= targetPosition; pos--) {
          myservo.write(pos);
          delay(80); // 80ms delay per degree
        }
      }
    } else {
      // Normal daytime tracking: the jump is only 1 degree, so just move it
      myservo.write(targetPosition);
    }

    currentDegree = targetPosition;
    Serial.println("Sun position updated: " + String(currentDegree));
  }
}

void resetWiFiCredentials() {
  Serial.println("Resetting WiFi credentials...");

  // Erase stored SSID and password
  preferences.begin("wifi", false);  // write mode
  preferences.remove("ssid");
  preferences.remove("password");
  preferences.end();

  // Disconnect WiFi if currently connected
  WiFi.disconnect(true);  // erase runtime credentials
  wifiConnected = false;

  Serial.println("WiFi credentials removed.");

  // Start BLE provisioning again
  startBLEServer();
  Serial.println("BLE server started. Ready for new WiFi setup.");
}

bool initINA226() {
  Serial.println("\n=== INA226 Initialization ===");
  
  Wire.beginTransmission(INA226_ADDRESS);
  if (Wire.endTransmission() != 0) {
    return false;
  }

  writeRegister(INA226_REG_CONFIG, INA226_CONFIG_DEFAULT);
  
  float currentLSB = MAX_CURRENT / 32768.0;
  uint16_t cal = (uint16_t)(0.00512 / (currentLSB * SHUNT_RESISTANCE));
  
  writeRegister(INA226_REG_CALIBRATION, cal);
  
  delay(100);
  return true;
}

void readINA226() {
  uint16_t busVoltageRaw = readRegister(INA226_REG_BUS_VOLTAGE);
  voltage = (busVoltageRaw * 1.25) / 1000.0;
  
  int16_t currentRaw = (int16_t)readRegister(INA226_REG_CURRENT);
  float currentLSB = MAX_CURRENT / 32768.0;
  current = currentRaw * currentLSB;
  
  uint16_t powerRaw = readRegister(INA226_REG_POWER);
  float powerLSB = currentLSB * 25;
  power = powerRaw * powerLSB;
  
  // // ★ ADD DEBUG OUTPUT - Print every reading
  // Serial.println("\n--- INA226 Reading ---");
  // Serial.printf("Bus Voltage:   %.3f V\n", voltage);
  // Serial.printf("Shunt Voltage: %.1f µV (raw: %d)\n", shuntVoltage * 1e6, shuntVoltageRaw);
  // Serial.printf("Current:       %.3f A (raw: %d)\n", current, currentRaw);
  // Serial.printf("Power:         %.3f W\n", power);
  // Serial.printf("PWM Load:      %d/255 (%d%%)\n", currentPWM, (currentPWM*100)/255);
  
  // // ★ DIAGNOSTIC
  // if (abs(shuntVoltageRaw) < 5) {
  //   Serial.println("⚠️  WARNING: Shunt voltage near zero!");
  //   Serial.println("   → No current flowing through shunt");
  //   Serial.println("   → Check: PWM enabled? Load connected?");
  // }
  
  // if (voltage < 0.5) {
  //   Serial.println("⚠️  WARNING: Bus voltage very low!");
  //   Serial.println("   → Check solar panel connection");
  //   Serial.println("   → Ensure adequate light on panel");
  // }
  
  // Serial.println("----------------------\n");
}

void testMOSFET() {
  Serial.println("╔═══════════════════════════════════════╗");
  Serial.println("║   MOSFET HARDWARE TEST                ║");
  Serial.println("╚═══════════════════════════════════════╝\n");
  
  // Test 1: OFF
  Serial.println("[1/4] MOSFET OFF (PWM = 0)");
  ledcWrite(PWM_PIN, 0);
  delay(1000);
  readINA226();
  Serial.printf("  Current: %.3f A (%.1f mA)\n", current, current * 1000);
  Serial.println("  Expected: ~0 mA\n");
  delay(2000);
  
  // Test 2: 25%
  Serial.println("[2/4] MOSFET 25% (PWM = 64)");
  ledcWrite(PWM_PIN, 64);
  delay(1000);
  readINA226();
  Serial.printf("  Current: %.3f A (%.1f mA)\n", current, current * 1000);
  Serial.println("  Expected: Low current\n");
  delay(2000);
  
  // Test 3: 50%
  Serial.println("[3/4] MOSFET 50% (PWM = 128)");
  ledcWrite(PWM_PIN, 128);
  delay(1000);
  readINA226();
  Serial.printf("  Current: %.3f A (%.1f mA)\n", current, current * 1000);
  Serial.println("  Expected: Medium current\n");
  delay(2000);
  
  // Test 4: 100%
  Serial.println("[4/4] MOSFET 100% (PWM = 255)");
  ledcWrite(PWM_PIN, 255);
  delay(1000);
  readINA226();
  Serial.printf("  Current: %.3f A (%.1f mA)\n", current, current * 1000);
  Serial.println("  Expected: Maximum current");
  
  if (current > 0.1) {
    Serial.println("\n✓ MOSFET is conducting!");
    Serial.println("  Expected: I = V / R_load");
    Serial.printf("  With 1Ω: I = %.2fV / 1Ω = %.2fA\n", voltage, voltage / 1.0);
  } else {
    Serial.println("\n⚠️  WARNING: Very low current!");
    Serial.println("  Check:");
    Serial.println("  - MOSFET gate connected to GPIO 25?");
    Serial.println("  - MOSFET source connected to GND?");
    Serial.println("  - 1Ω resistor connected from C- to MOSFET drain?");
  }
  
  Serial.println("\n╚═══════════════════════════════════════╝\n");
  
  // Return to safe state
  ledcWrite(PWM_PIN, 50);
  currentPWM = 50;
  delay(1000);
}

void calculateEnergy() {
  // static unsigned long previousMicros = 0;
  // unsigned long currentMicros = micros();
  
  // if (previousMicros == 0) {
  //   previousMicros = currentMicros;
  //   return;
  // }
  
  // float timeHours = (currentMicros - previousMicros) / 3600000000.0;
  
  // if (power > 0) {
  //   energy += power * timeHours;
  // }
  
  // previousMicros = currentMicros;

  static unsigned long previousTime = 0;
  unsigned long currentTime = millis();
  
  if (previousTime > 0) {
    float timeHours = (currentTime - previousTime) / 3600000.0;
    energy += power * timeHours;
  }
  
  previousTime = currentTime;
}

void printMeasurements() {
  unsigned long runtime = (millis() - startTime) / 1000;
  int hours = runtime / 3600;
  int minutes = (runtime % 3600) / 60;
  int seconds = runtime % 60;
  
  Serial.print("\033[2J\033[H");
  
  Serial.println("╔════════════════════════════════════════╗");
  Serial.println("║    ESP32 SOLAR ENERGY MONITOR          ║");
  Serial.println("╚════════════════════════════════════════╝");
  Serial.print("Runtime: ");
  Serial.printf("%02d:%02d:%02d\n", hours, minutes, seconds);
  Serial.println();
  
  Serial.print("🔋 Voltage:  ");
  Serial.print(voltage, 3);
  Serial.println(" V");
  
  Serial.print("⚡ Current:  ");
  Serial.print(current, 3);
  Serial.println(" A");
  
  Serial.print("💡 Power:    ");
  Serial.print(power, 3);
  Serial.println(" W");
  
  Serial.print("📊 Energy:   ");
  Serial.print(energy, 4);
  Serial.println(" Wh");
  
  Serial.println("\n--- Electronic Load Status ---");
  Serial.print("PWM Load: ");
  Serial.print(currentPWM);
  Serial.print("/255 (");
  Serial.print((currentPWM * 100) / 255);
  Serial.println("%)");
  
  Serial.print("Auto MPPT: ");
  Serial.println(mpptEnabled ? "ENABLED ✓" : "DISABLED");
  
  if (maxPower > 0) {
    Serial.println("\n--- Maximum Power Point ---");
    Serial.print("Best Power: ");
    Serial.print(maxPower, 3);
    Serial.println(" W");
    Serial.print("At PWM: ");
    Serial.print(optimalPWM);
    Serial.print(" (");
    Serial.print((optimalPWM * 100) / 255);
    Serial.println("%)");
  }
  
  float estimatedPower = voltage * current * 1000;
  Serial.println("\n--- Assessment ---");
  Serial.print("Instant Power: ");
  Serial.print(estimatedPower, 1);
  Serial.println(" mW");
  
  if (voltage < 0.5) {
    Serial.println("\n⚠️  WARNING: Voltage too low!");
    Serial.println("   Check power source connection");
  } else if (voltage > 4.5 && voltage < 5.5) {
    Serial.println("\n✓ Voltage good (5V source)");
  }
  
  if (current > 0.05) {
    float resistorValue = voltage / current;
    float powerDissipation = current * current * resistorValue;
    
    Serial.println();
    Serial.print("Effective Load: ~");
    Serial.print(resistorValue, 1);
    Serial.println(" Ω");
    
    Serial.print("Load Power: ");
    Serial.print(powerDissipation, 3);
    Serial.println(" W");
    
    if (powerDissipation > 1.0) {
      Serial.println("⚠️  High power - ensure MOSFET has heatsink!");
    }
  }
  
  Serial.println("\n═══════════════════════════════════════");
  Serial.println();
}

void writeRegister(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(INA226_ADDRESS);
  Wire.write(reg);
  Wire.write((value >> 8) & 0xFF);
  Wire.write(value & 0xFF);
  Wire.endTransmission();
}

uint16_t readRegister(uint8_t reg) {
  Wire.beginTransmission(INA226_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false);
  
  Wire.requestFrom(INA226_ADDRESS, 2);
  uint16_t value = 0;
  
  if (Wire.available() == 2) {
    value = Wire.read() << 8;
    value |= Wire.read();
  }
  
  return value;
}

void performMPPTSweep() {
  Serial.println("\n╔═══════════════════════════════════════╗");
  Serial.println("║   MPPT SWEEP - FINDING MAXIMUM POWER  ║");
  Serial.println("╚═══════════════════════════════════════╝\n");
  
  maxPower = 0;
  maxPowerVoltage = 0;
  maxPowerCurrent = 0;
  optimalPWM = 0;
  
  Serial.println("Sweeping PWM from 0% to 100%...\n");
  Serial.println("PWM   Duty%   Voltage    Current    Power");
  Serial.println("----  -----   -------    -------    -----");
  
  for (int pwm = 0; pwm <= 255; pwm += 10) {
    ledcWrite(PWM_PIN, pwm);
    delay(200);
    
    readINA226();
    
    Serial.printf("%3d   %3d%%    %6.3fV    %6.3fA    %6.3fW", 
                  pwm, (pwm * 100) / 255, voltage, current, power);
    
    if (power > maxPower) {
      maxPower = power;
      maxPowerVoltage = voltage;
      maxPowerCurrent = current;
      optimalPWM = pwm;
      Serial.print(" ← ★ MAX");
    }
    Serial.println();
  }
  
  // Set to optimal PWM
  currentPWM = optimalPWM;
  ledcWrite(PWM_PIN, currentPWM);
  
  Serial.println("\n╔═══════════════════════════════════════╗");
  Serial.println("║   MPPT RESULTS                        ║");
  Serial.println("╠═══════════════════════════════════════╣");
  Serial.printf("║ Optimal PWM:  %3d (%3d%%)              ║\n", 
                optimalPWM, (optimalPWM * 100) / 255);
  Serial.printf("║ Max Power:    %.3f W                 ║\n", maxPower);
  Serial.printf("║ At Voltage:   %.3f V                 ║\n", maxPowerVoltage);
  Serial.printf("║ At Current:   %.3f A                 ║\n", maxPowerCurrent);
  Serial.println("╚═══════════════════════════════════════╝\n");
}


void performAutoMPPT() {
  static float lastPower = 0;
  static int perturbDirection = 1;
  static unsigned long lastMPPTTime = 0;
  
  if (millis() - lastMPPTTime < 2000) return;
  lastMPPTTime = millis();
  
  // Perturb & Observe algorithm
  if (power > lastPower) {
    currentPWM += perturbDirection * 5;
  } else {
    perturbDirection = -perturbDirection;
    currentPWM += perturbDirection * 5;
  }
  
  currentPWM = constrain(currentPWM, 0, 255);
  ledcWrite(PWM_PIN, currentPWM);
  lastPower = power;

  Serial.print("⚡ Auto MPPT: PWM=");
  Serial.print(currentPWM);
  Serial.print(" (");
  Serial.print((currentPWM * 100) / 255);
  Serial.print("%), P=");
  Serial.print(power, 3);
  Serial.println("W");
}

void setup() {
  Serial.begin(115200);

  ledcAttach(PWM_PIN, PWM_FREQ, PWM_RESOLUTION);
  ledcWrite(PWM_PIN, 0);
  Serial.println("✓ MOSFET PWM initialized early on Timer 0");

  // 2. NOW SET UP SERVO RULES
  // Because Timer 0 is taken, the servo will safely take Timer 1 later
  myservo.setPeriodHertz(50);

  Wire.begin();

  Serial.println("Scanning I2C bus...");
  Wire.begin();
  byte deviceCount = 0;
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      deviceCount++;
    }
  }
  
  if (deviceCount == 0) {
    Serial.println("ERROR: No I2C devices found!");
    Serial.println("Check INA226 wiring!");
  } else {
    Serial.print("Found ");
    Serial.print(deviceCount);
    Serial.println(" I2C device(s)\n");
  }

  // ---- Load stored WiFi credentials ----
  preferences.begin("wifi", true);   // read-only
  String ssid = preferences.getString("ssid", "");
  String password = preferences.getString("password", "");
  preferences.end();

  Serial.println("Stored SSID: " + ssid);

  // ---- Attempt WiFi Connection ----
  if (ssid.length() > 0 && password.length() > 0) {
    Serial.println("Connecting with saved WiFi credentials...");
    connectToWiFi(ssid.c_str(), password.c_str());
  }

  // ---- Handle Success or Failure ----
  if (wifiConnected) {
    Serial.println("✓ Connected to WiFi!");
    pinMode(2, OUTPUT);
    digitalWrite(2, HIGH);
    // Notice: NO 'return;' here! We just move on to the hardware.
  } else {
    Serial.println("⚠ No WiFi connection. Starting BLE provisioning...");
    startBLEServer();
  }

  if (initINA226()) {
    Serial.println("✓ INA226 initialized successfully\n");
  } else {
    Serial.println("✗ Failed to initialize INA226");
    Serial.println("System halted - fix INA226 connection\n");
    while(1) {
      delay(1000);
    }
  }


  Serial.println("\n===========================================");
  Serial.println("SYSTEM READY - Starting MPPT Test");
  Serial.println("===========================================\n");
  
  // delay(2000);
  
  // // Run hardware test
  // // testMOSFET();
  
  lastTime = millis();
  startTime = millis();


  timeClient.begin();
  timeClient.setTimeOffset(-5 * 3600);
  timeClient.update(); 
  
  // // Optional: Force an update on boot to verify we have time
  if(wifiConnected) {
    timeClient.update(); 
  }
  
  // // pinMode(2, OUTPUT);
  // // digitalWrite(2, HIGH);

  Serial.println("✓ System ready! Starting measurements...\n");
}

void loop() {
  if (wifiConnected) {
    server.handleClient();
  }

  unsigned long currentMillis = millis();

  // 1. Existing Measurement Logic (Runs every 1 second)
  if (currentMillis - lastTime >= 1000) {
    readINA226();
    calculateEnergy();
    if (mpptEnabled) {
      performAutoMPPT();
    }
    // printMeasurements();
    lastTime = currentMillis;
  }

  if (wifiConnected && (currentMillis - lastUploadTime >= UPLOAD_INTERVAL)) {
    uploadToPipedream();
    lastUploadTime = currentMillis;
  }
}