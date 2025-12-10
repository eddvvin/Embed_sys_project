#include <Arduino.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <ArduinoJson.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>
#include <time.h>

// ===========================
// WiFi CREDENTIALS
// ===========================
#define WIFI_SSID "TTUguest"
#define WIFI_PASSWORD "maskedraider"

// ===========================
// FIREBASE CONFIG
// ===========================
#define API_KEY "AIzaSyBayJmG5l9niRQW67GumaUSABTCMyhmato"
#define DATABASE_URL "https://embed-sys-project-default-rtdb.firebaseio.com/"

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// ===========================
// UART SETTINGS
// ===========================
#define RXD2 16
#define TXD2 17

String jsonBuffer = "";

// ===========================
// TIMING
// ===========================
unsigned long lastUpload = 0;
const unsigned long UPLOAD_INTERVAL = 1000; // 1s upload rate

unsigned long lastRelayPoll = 0;
const unsigned long RELAY_POLL_INTERVAL = 300;

// ===========================
// GLOBAL STATE VARIABLES
// Filled after reading STM32 JSON
// ===========================
float currentValue = 0.0;
int relay1_state = 0;
int relay2_state = 0;

//
// ===================================================
//                 SETUP
// ===================================================
//
void setup() {

  Serial.begin(115200);  
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

  // Sync system time for Firebase
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  delay(2000);

  // Connect to Wi-Fi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nWiFi connected!");

  // Firebase Setup
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  config.token_status_callback = tokenStatusCallback;

  Firebase.signUp(&config, &auth, "", "");
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  // Initialize control nodes
  Firebase.RTDB.setBool(&fbdo, "/relayToggle1", false);
  Firebase.RTDB.setBool(&fbdo, "/relayToggle2", false);
}

//
// ===================================================
//                 MAIN LOOP
// ===================================================
//
void loop() {

  // ---------------------------------------
  // Read JSON from STM32 over UART
  // ---------------------------------------
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n') {
      processSTM32JSON(jsonBuffer);
      jsonBuffer = "";
    } else {
      jsonBuffer += c;
    }
  }

  // ---------------------------------------
  // Poll Firebase relay toggle commands
  // ---------------------------------------
  if (millis() - lastRelayPoll >= RELAY_POLL_INTERVAL) {
    checkFirebaseRelayCommands();
    lastRelayPoll = millis();
  }

  // ---------------------------------------
  // Upload sensor data to Firebase
  // ---------------------------------------
  if (millis() - lastUpload >= UPLOAD_INTERVAL) {
    uploadCurrentToFirebase();
    lastUpload = millis();
  }
}

//
// ===================================================
//              PROCESS STM32 JSON LINE
// ===================================================
//
void processSTM32JSON(String jsonStr) {
  StaticJsonDocument<128> doc;

  DeserializationError error = deserializeJson(doc, jsonStr);
  if (error) {
    Serial.println("JSON parse failed!");
    Serial.println(jsonStr);
    return;
  }

  currentValue = doc["current"];
  relay1_state = doc["relay1"];
  relay2_state = doc["relay2"];

  Serial.print("STM32 → ESP32 JSON: ");
  Serial.println(jsonStr);
}

//
// ===================================================
//         CHECK FIREBASE COMMANDS FOR RELAYS
// ===================================================
//
void checkFirebaseRelayCommands() {
  bool value;

  // ---------------- Relay 1 ----------------
  if (Firebase.RTDB.getBool(&fbdo, "/relayToggle1")) {
    value = fbdo.boolData();

    if (value) Serial2.println("L1_ON");
    else       Serial2.println("L1_OFF");

    Serial.print("Firebase → STM32 Relay 1: ");
    Serial.println(value ? "ON" : "OFF");
  }

  // ---------------- Relay 2 ----------------
  if (Firebase.RTDB.getBool(&fbdo, "/relayToggle2")) {
    value = fbdo.boolData();

    if (value) Serial2.println("L2_ON");
    else       Serial2.println("L2_OFF");

    Serial.print("Firebase → STM32 Relay 2: ");
    Serial.println(value ? "ON" : "OFF");
  }
}

//
// ===================================================
//         UPLOAD CURRENT TO FIREBASE
// ===================================================
//
void uploadCurrentToFirebase() {

  // Upload only if the relays are ON
  if (relay1_state == 0 && relay2_state == 0) return;

  time_t nowSec = time(nullptr);
  unsigned long timestamp_ms = nowSec * 1000UL + (millis() % 1000);

  String path;

  if (relay1_state == 1) {
    path = "/RelayData1/" + String(timestamp_ms);
    Firebase.RTDB.setFloat(&fbdo, path, currentValue);
  }

  if (relay2_state == 1) {
    path = "/RelayData2/" + String(timestamp_ms);
    Firebase.RTDB.setFloat(&fbdo, path, currentValue);
  }

  Serial.println("Uploaded Current → Firebase");
}
