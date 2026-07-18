// ============================================================
// ESP32 - Voltage + Current + Relay + Firebase Control
// Mode: Auto / Manual. Relays controlled via Firebase.
// Corrected FirebaseJson usage.
// ============================================================

#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <HardwareSerial.h>
#include <Firebase_ESP_Client.h>

// ==================== WiFi ====================
const char* ssid     = "Airtel_sahi_0849";
const char* password = "air99772";

// ==================== Firebase ====================
#define FIREBASE_HOST "https://energy-25d04-default-rtdb.firebaseio.com/"
#define FIREBASE_API_KEY "AIzaSyDbCMFt216kuAMoCYZlXzfqUzVeFM085Fw"
#define USER_EMAIL "sb284160@gmail.com"
#define USER_PASSWORD "Password@1"

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// ==================== Pins ====================
const int voltagePins[4] = {35, 34, 39, 36};
const int relayPins[4] = {27, 14, 15, 13};

// ==================== Relay Logic ====================
#define RELAY_ACTIVE_LOW
#ifdef RELAY_ACTIVE_LOW
  #define RELAY_ON  HIGH
  #define RELAY_OFF LOW
#else
  #define RELAY_ON  LOW
  #define RELAY_OFF HIGH
#endif

// ==================== Calibration ====================
const float mVperAmp = 185.0;
float CURRENT_CAL[4] = {1.00, 1.00, 1.00, 1.00};
float voltageCalib = 312.5;

// ==================== Web & Preferences ====================
WebServer server(80);
Preferences preferences;
HardwareSerial STMSerial(2);   // UART2: RX=16, TX=17

// ---------- Control variables (from Firebase) ----------
String currentMode = "MANUAL";              // "MANUAL" or "AUTO"
bool manualRelayState[4] = {1,1,1,1};      // user‑requested states (Manual mode)
bool actualRelayState[4] = {1,1,1,1};      // what the relays are actually doing

// ---------- Sensor data ----------
float voltageRMS[4] = {0,0,0,0};
float currentRMS[4] = {0,0,0,0};
float power[4] = {0,0,0,0};
float energyWh[4] = {0,0,0,0};
unsigned long lastEnergyUpdate = 0;

float smoothV[4] = {0,0,0,0};
const float alpha = 0.85;
float vDCOffset[4] = {0,0,0,0};

float vSumSq[4] = {0,0,0,0};
int vSampleCount[4] = {0,0,0,0};
const int V_SAMPLES = 200;
unsigned long sampleTimer = 0;

String uartData = "";
unsigned long lastCurrentReceive = 0;
unsigned long lastSerialPrint = 0;
unsigned long lastFirebaseSend = 0;
unsigned long lastFirebaseControlCheck = 0;

// ==================== Voltage Calibration ====================
void calibrateVoltageOffsets() {
  Serial.println("Calibrating voltage offsets... (NO AC applied!)");
  const int calSamples = 5000;
  float vSum[4] = {0,0,0,0};
  for (int i = 0; i < calSamples; i++) {
    for (int ch = 0; ch < 4; ch++) {
      vSum[ch] += (analogRead(voltagePins[ch]) / 4095.0) * 3.3;
    }
    delayMicroseconds(1000);
  }
  for (int ch = 0; ch < 4; ch++) {
    vDCOffset[ch] = vSum[ch] / calSamples;
    preferences.putFloat(("vOff"+String(ch)).c_str(), vDCOffset[ch]);
    Serial.printf("CH%d Voffset = %.3f V\n", ch+1, vDCOffset[ch]);
  }
  Serial.println("Voltage calibration done.");
}

// ==================== Setup ====================
void setup() {
  Serial.begin(115200);
  STMSerial.begin(115200, SERIAL_8N1, 16, 17);

  analogReadResolution(12);
  preferences.begin("energy", false);

  for (int i = 0; i < 4; i++) {
    pinMode(relayPins[i], OUTPUT);
    String key = "relay" + String(i);
    manualRelayState[i] = preferences.getBool(key.c_str(), true);
    actualRelayState[i] = manualRelayState[i];
    digitalWrite(relayPins[i], actualRelayState[i] ? RELAY_ON : RELAY_OFF);
  }

  calibrateVoltageOffsets();

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting WiFi");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected! IP: " + WiFi.localIP().toString());
  } else {
    Serial.println("\nWiFi connection failed.");
  }

  // ---- Firebase ----
  config.api_key = FIREBASE_API_KEY;
  config.database_url = FIREBASE_HOST;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  Firebase.reconnectWiFi(true);
  fbdo.setResponseSize(4096);
  Firebase.begin(&config, &auth);

  // ---- REST API ----
  server.on("/data", HTTP_GET, handleData);
  server.on("/relay", HTTP_GET, handleRelay);
  server.on("/", HTTP_GET, []() { server.send(200, "text/plain", "ESP32 OK"); });
  server.begin();
  Serial.println("HTTP server started");

  lastEnergyUpdate = millis();
  lastSerialPrint = millis();
  lastFirebaseSend = millis();
  lastFirebaseControlCheck = millis();
}

// ==================== Main Loop ====================
void loop() {
  // ---- Voltage sampling ----
  if (micros() - sampleTimer >= 1000) {
    sampleTimer = micros();
    for (int ch = 0; ch < 4; ch++) {
      float v = (analogRead(voltagePins[ch]) / 4095.0) * 3.3 - vDCOffset[ch];
      vSumSq[ch] += v * v;
      vSampleCount[ch]++;
      if (vSampleCount[ch] >= V_SAMPLES) {
        float vRms = sqrt(vSumSq[ch] / V_SAMPLES);
        float vReal = vRms * voltageCalib;
        if (vReal < 50.0) vReal = 0.0;
        if (smoothV[ch] == 0) smoothV[ch] = vReal;
        else smoothV[ch] = alpha * vReal + (1 - alpha) * smoothV[ch];
        voltageRMS[ch] = smoothV[ch];
        vSumSq[ch] = 0;
        vSampleCount[ch] = 0;
      }
    }
  }

  // ---- UART Receive (current from STM32) ----
  while (STMSerial.available()) {
    char c = STMSerial.read();
    if (c == '\n') {
      uartData.trim();
      if (uartData.length() > 0) {
        parseCurrent(uartData);
        lastCurrentReceive = millis();
      }
      uartData = "";
    } else if (c != '\r') {
      uartData += c;
    }
  }

  // Zero current if no data for 1.5 sec
  if (millis() - lastCurrentReceive > 1500) {
    for (int ch = 0; ch < 4; ch++) currentRMS[ch] = 0.0;
  }

  // ---- Power & Energy ----
  for (int ch = 0; ch < 4; ch++) power[ch] = voltageRMS[ch] * currentRMS[ch];

  if (millis() - lastEnergyUpdate >= 5000) {
    static unsigned long lastTime = millis();
    unsigned long now = millis();
    float deltaHours = (now - lastTime) / 3600000.0;
    for (int ch = 0; ch < 4; ch++) energyWh[ch] += power[ch] * deltaHours;
    lastTime = now;
    for (int ch = 0; ch < 4; ch++) preferences.putFloat(("energy"+String(ch)).c_str(), energyWh[ch]);
    lastEnergyUpdate = millis();
  }

  // ---- Send sensor data to Firebase ----
  if (millis() - lastFirebaseSend >= 2000) {
    sendToFirebase();
    lastFirebaseSend = millis();
  }

  // ---- Check Firebase control every 500ms ----
  if (millis() - lastFirebaseControlCheck >= 500) {
    readControlFromFirebase();
    lastFirebaseControlCheck = millis();
  }

  // ---- Apply relay logic (Auto / Manual) ----
  applyRelayLogic();

  // ---- Send $ESP to STM32 (for OLED) ----
  static unsigned long lastSend = 0;
  if (millis() - lastSend >= 500) {
    sendToSTM32();
    lastSend = millis();
  }

  // ---- Print serial dashboard ----
  if (millis() - lastSerialPrint >= 2000) {
    printSerialDashboard();
    lastSerialPrint = millis();
  }

  server.handleClient();
}

// ==================== parseCurrent (from STM32) ====================
void parseCurrent(String data) {
  data.trim();
  if (!data.startsWith("$CUR,")) return;
  data.remove(0, 5);
  char buffer[80];
  data.toCharArray(buffer, sizeof(buffer));
  char *token = strtok(buffer, ",");
  int index = 0;
  while (token != NULL && index < 4) {
    float value = atof(token);
    if (value < 0.01) value = 0;
    currentRMS[index] = value * CURRENT_CAL[index];
    token = strtok(NULL, ",");
    index++;
  }
}

// ==================== Firebase: Send sensor data ====================
void sendToFirebase() {
  if (!Firebase.ready()) return;
  String path = "/energy/data";
  FirebaseJson json;
  json.set("timestamp", String(millis()));
  json.set("voltage1", voltageRMS[0]);
  json.set("voltage2", voltageRMS[1]);
  json.set("voltage3", voltageRMS[2]);
  json.set("voltage4", voltageRMS[3]);
  json.set("current1", currentRMS[0]);
  json.set("current2", currentRMS[1]);
  json.set("current3", currentRMS[2]);
  json.set("current4", currentRMS[3]);
  json.set("power1", power[0]);
  json.set("power2", power[1]);
  json.set("power3", power[2]);
  json.set("power4", power[3]);
  json.set("energy1", energyWh[0]);
  json.set("energy2", energyWh[1]);
  json.set("energy3", energyWh[2]);
  json.set("energy4", energyWh[3]);
  json.set("relay1", actualRelayState[0]);
  json.set("relay2", actualRelayState[1]);
  json.set("relay3", actualRelayState[2]);
  json.set("relay4", actualRelayState[3]);
  json.set("mode", currentMode);

  if (Firebase.RTDB.push(&fbdo, path, &json)) {
    // success
  } else {
    Serial.print("Firebase push error: ");
    Serial.println(fbdo.errorReason());
  }
}

// ==================== Firebase: Read control (fixed) ====================
void readControlFromFirebase() {
  if (!Firebase.ready()) return;
  String path = "/control";
  if (Firebase.RTDB.get(&fbdo, path)) {
    // Get the data as FirebaseJson
    FirebaseJson json = fbdo.jsonObject();
    FirebaseJsonData jsonData;

    // Read mode
    json.get(jsonData, "mode");
    if (jsonData.success) {
      String mode = jsonData.stringValue;
      if (mode == "AUTO" || mode == "MANUAL") {
        currentMode = mode;
      }
    }

    // Read manual relay states
    for (int i = 0; i < 4; i++) {
      String key = "relay" + String(i+1);
      json.get(jsonData, key.c_str());
      if (jsonData.success) {
        manualRelayState[i] = jsonData.intValue == 1;
      }
    }
  } else {
    Serial.print("Failed to read control: ");
    Serial.println(fbdo.errorReason());
  }
}

// ==================== Apply Relay Logic ====================
void applyRelayLogic() {
  bool newState[4];
  if (currentMode == "MANUAL") {
    // Manual mode: use the states from Firebase (user controls)
    for (int i = 0; i < 4; i++) {
      newState[i] = manualRelayState[i];
    }
  } else {
    // AUTO mode: automatic protection
    for (int i = 0; i < 4; i++) {
      // If voltage > 230 OR current > 5A -> turn OFF, else ON
      if (voltageRMS[i] > 230.0 || currentRMS[i] > 5.0) {
        newState[i] = 0;   // OFF
      } else {
        newState[i] = 1;   // ON
      }
    }
  }

  // Apply changes (only if different)
  for (int i = 0; i < 4; i++) {
    if (newState[i] != actualRelayState[i]) {
      actualRelayState[i] = newState[i];
      digitalWrite(relayPins[i], actualRelayState[i] ? RELAY_ON : RELAY_OFF);
      // Save to Preferences for persistence
      preferences.putBool(("relay"+String(i)).c_str(), actualRelayState[i]);
    }
  }
}

// ==================== Send to STM32 (OLED) ====================
void sendToSTM32() {
  String ip = WiFi.localIP().toString();
  char buf[200];
  sprintf(buf, "$ESP,%s,%.1f,%.1f,%.1f,%.1f,%d,%d,%d,%d\n",
          ip.c_str(),
          voltageRMS[0], voltageRMS[1], voltageRMS[2], voltageRMS[3],
          actualRelayState[0], actualRelayState[1],
          actualRelayState[2], actualRelayState[3]);
  STMSerial.print(buf);
}

// ==================== Print Serial Dashboard ====================
void printSerialDashboard() {
  Serial.println("--- Energy Monitor ---");
  Serial.print("Mode: ");
  Serial.println(currentMode);
  for (int ch = 0; ch < 4; ch++) {
    Serial.printf("CH%d %.1fV %.2fA %.0fW %s\n",
                  ch+1,
                  voltageRMS[ch],
                  currentRMS[ch],
                  power[ch],
                  actualRelayState[ch] ? "ON" : "OFF");
  }
  Serial.print("IP: ");
  Serial.println(WiFi.localIP().toString());
  Serial.println();
}

// ==================== REST API (for direct control) ====================
void handleData() {
  StaticJsonDocument<1024> doc;
  doc["voltage1"] = voltageRMS[0]; doc["voltage2"] = voltageRMS[1];
  doc["voltage3"] = voltageRMS[2]; doc["voltage4"] = voltageRMS[3];
  doc["current1"] = currentRMS[0]; doc["current2"] = currentRMS[1];
  doc["current3"] = currentRMS[2]; doc["current4"] = currentRMS[3];
  doc["power1"] = power[0]; doc["power2"] = power[1];
  doc["power3"] = power[2]; doc["power4"] = power[3];
  doc["energy1"] = energyWh[0]; doc["energy2"] = energyWh[1];
  doc["energy3"] = energyWh[2]; doc["energy4"] = energyWh[3];
  doc["relay1"] = actualRelayState[0]; doc["relay2"] = actualRelayState[1];
  doc["relay3"] = actualRelayState[2]; doc["relay4"] = actualRelayState[3];
  doc["mode"] = currentMode;
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleRelay() {
  if (server.hasArg("ch") && server.hasArg("state")) {
    int ch = server.arg("ch").toInt();
    int state = server.arg("state").toInt();
    if (ch >= 1 && ch <= 4 && (state == 0 || state == 1)) {
      int idx = ch - 1;
      // If in MANUAL mode, update manual state and apply
      if (currentMode == "MANUAL") {
        manualRelayState[idx] = state;
        // Also push to Firebase so web app stays in sync
        FirebaseJson json;
        json.set("relay"+String(ch), state);
        Firebase.RTDB.updateNode(&fbdo, "/control", &json);
      }
      // else in AUTO mode, ignore direct relay command
      server.send(200, "text/plain", "OK");
      return;
    }
  }
  server.send(400, "text/plain", "Invalid parameters");
}