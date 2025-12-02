#include <WiFi.h>
#include <HTTPClient.h>

const char* ssid     = "capstone_project";
const char* password = "0123M67";

const String databaseURL = "https://embed-sys-project-default-rtdb.firebaseio.com/";
const String auth        = "AIzaSyBayJmG5l9niRQW67GumaUSABTCMyhmato";

HardwareSerial &stm = Serial2; // UART to STM32 (GPIO16 RX, GPIO17 TX)

void setup() {
  Serial.begin(115200);
  stm.begin(115200, SERIAL_8N1, 16, 17); // RX, TX pins

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting WiFi...");
  }
  Serial.println("WiFi connected");
}

void loop() {
  // 1) Read line from STM32 and upload
  if (stm.available()) {
    String line = stm.readStringUntil('\n');  // {"I":1.23,"V":2.50,"R1":1,"R2":0}
    line.trim();
    if (line.length() > 0) {
      uploadToFirebase(line);
    }
  }

  // 2) Periodically poll commands and send to STM32
  static unsigned long lastCmd = 0;
  if (millis() - lastCmd > 1000) { // every 1s
    lastCmd = millis();
    String cmd = readCommands();
    if (cmd.length() > 0) {
      stm.println(cmd); // e.g., "L1_ON", "L2_OFF"
    }
  }
}

void uploadToFirebase(const String &jsonPayload)
{
  if (WiFi.status() != WL_CONNECTED) return;
  HTTPClient http;
  String url = databaseURL + "energy.json?auth=" + auth;

  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  int httpCode = http.PUT(jsonPayload);

  Serial.print("Upload: ");
  Serial.println(httpCode);

  http.end();
}

String readCommands()
{
  if (WiFi.status() != WL_CONNECTED) return "";

  HTTPClient http;
  String url = databaseURL + "commands.json?auth=" + auth;
  http.begin(url);
  int httpCode = http.GET();
  if (httpCode != 200) {
    http.end();
    return "";
  }
  String payload = http.getString();
  http.end();

  // Expecting something simple like: {"cmd":"L1_ON"}
  if (payload.indexOf("L1_ON")  != -1) return "L1_ON";
  if (payload.indexOf("L1_OFF") != -1) return "L1_OFF";
  if (payload.indexOf("L2_ON")  != -1) return "L2_ON";
  if (payload.indexOf("L2_OFF") != -1) return "L2_OFF";

  return "";
}
