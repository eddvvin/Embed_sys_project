#include <WiFi.h>
#include <HTTPClient.h>

const char* ssid     = "capstone_project";
const char* password = "0123M67";

const String databaseURL = "https://embed-sys-project-default-rtdb.firebaseio.com/";
const String auth        = "7p803EsQsDYS4yAmJfYouzVJU38dp3KF2jktr1fG";
HardwareSerial &stm = Serial2;

void uploadToFirebase(const String &jsonPayload);
String readCommands();

void setup() {
  Serial.begin(115200);
  stm.begin(115200, SERIAL_8N1, 16, 17);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.println("Connecting WiFi...");
  }
  Serial.println("WiFi connected");
}

void loop() {

// =============== 1) Upload Data from STM32 ===============
  if (stm.available()) {
    String jsonLine = stm.readStringUntil('\n');
    jsonLine.trim();

    if (jsonLine.length() > 0) {
      uploadToFirebase(jsonLine);
    }
  }

// =============== 2) Read Commands every 1 second ===============
  static unsigned long lastCmd = 0;

  if (millis() - lastCmd >= 1000) {
    lastCmd = millis();
    String cmd = readCommands();

    if (cmd.length() > 0) {
      stm.println(cmd);
      Serial.print("Sent to STM32: ");
      Serial.println(cmd);
    }
  }
}


// ============================================================
//              UPLOAD DATA TO FIREBASE
// ============================================================
void uploadToFirebase(const String &jsonPayload)
{
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi Lost!");
    return;
  }

  HTTPClient http;
  http.useHTTP10(true);   // REQUIRED for Firebase on ESP32

  String url = databaseURL + "energy.json?auth=" + auth;

  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  // Validate JSON begins with '{' and ends with '}'
  if (!(jsonPayload.startsWith("{") && jsonPayload.endsWith("}"))) {
    Serial.println("Invalid JSON, skipping upload:");
    Serial.println(jsonPayload);
    http.end();
    return;
  }

  int code = http.PUT(jsonPayload);

  Serial.print("Firebase Upload HTTP code = ");
  Serial.println(code);

  if (code != 200 && code != 204) {
    Serial.println("Upload failed:");
    Serial.println(http.getString());
  }

  http.end();
}


// ============================================================
//            READ COMMANDS FROM FIREBASE
// ============================================================
String readCommands()
{
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi Lost!");
    return "";
  }

  HTTPClient http;
  http.useHTTP10(true);

  String url = databaseURL + "commands.json?auth=" + auth;
  http.begin(url);

  int code = http.GET();

  Serial.print("Command GET code = ");
  Serial.println(code);

  if (code != 200) {
    Serial.println("Error fetching commands");
    Serial.println(http.getString());
    http.end();
    return "";
  }

  String payload = http.getString();
  http.end();

  payload.trim();

  // Firebase returns "null" → no commands yet
  if (payload == "null" || payload.length() == 0) {
    return "";
  }

  Serial.print("Commands payload: ");
  Serial.println(payload);

  // Look for command fragments
  if (payload.indexOf("L1_ON")  != -1) return "L1_ON";
  if (payload.indexOf("L1_OFF") != -1) return "L1_OFF";
  if (payload.indexOf("L2_ON")  != -1) return "L2_ON";
  if (payload.indexOf("L2_OFF") != -1) return "L2_OFF";

  return "";
}


