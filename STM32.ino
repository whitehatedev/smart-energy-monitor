// ============================================================
// STM32 Nucleo - 4-Channel Current Sensor + OLED + UART
// With calibration: per-channel gain via serial commands
// ============================================================

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ==================== UART ====================
HardwareSerial ESPSerial(PA10, PA9);   // RX=PA10, TX=PA9

// ==================== Current Sensor Pins ====================
const int currentPins[4] = {PA3, PA2, PA1, PA0};

// ==================== Calibration ====================
const float mVperAmp = 185.0;          // for 5A ACS712 (change if you have 20A/30A)
const float ADC_REF = 3.3;
const int ADC_RES = 4096;

// Per‑channel gain (adjust via serial)
float CURRENT_CAL[4] = {1.00, 1.00, 1.00, 1.00};

// ==================== Global Variables ====================
float currentRMS[4] = {0,0,0,0};
float smoothI[4] = {0,0,0,0};
const float alpha = 0.85;

float voltage[4] = {0,0,0,0};
bool relay[4] = {1,1,1,1};
float power[4] = {0,0,0,0};
String ipAddress = "Waiting...";

// RMS on‑the‑fly
float iSumSq[4] = {0,0,0,0};
int iSampleCount[4] = {0,0,0,0};
const int I_SAMPLES = 2000;
unsigned long sampleTimer = 0;

float iDCOffset[4] = {0,0,0,0};

String dataStr = "";
unsigned long lastReceive = 0;
unsigned long lastSend = 0;

// ==================== Setup ====================
void setup() {
  Serial.begin(115200);
  ESPSerial.begin(115200);

  analogReadResolution(12);

  Wire.setSDA(PB9);
  Wire.setSCL(PB8);
  Wire.begin();

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) while(1);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println("STM32 Ready");
  display.display();

  calibrateOffsets();

  Serial.println("STM32 ready.");
  Serial.println("Commands: CAL<ch> <gain>  e.g. CAL1 1.05");
  Serial.println("          SHOW              print current values");
  Serial.println("          RESET             reset all gains to 1.00");
}

// ==================== Main Loop ====================
void loop() {
  // 1. Sample current at 1 kHz
  if (micros() - sampleTimer >= 1000) {
    sampleTimer = micros();
    for (int ch = 0; ch < 4; ch++) {
      int raw = analogRead(currentPins[ch]);
      float v = (raw / (float)ADC_RES) * ADC_REF - iDCOffset[ch];
      iSumSq[ch] += v * v;
      iSampleCount[ch]++;
      if (iSampleCount[ch] >= I_SAMPLES) {
        float iRms = sqrt(iSumSq[ch] / I_SAMPLES);
        float iReal = iRms / (mVperAmp / 1000.0);
        if (iReal < 0.01) iReal = 0.0;
        if (smoothI[ch] == 0) smoothI[ch] = iReal;
        else smoothI[ch] = alpha * iReal + (1 - alpha) * smoothI[ch];
        currentRMS[ch] = smoothI[ch] * CURRENT_CAL[ch];
        power[ch] = voltage[ch] * currentRMS[ch];
        iSumSq[ch] = 0;
        iSampleCount[ch] = 0;
      }
    }
  }

  // 2. Send current to ESP32 every 200ms
  if (millis() - lastSend >= 200) {
    char buf[60];
    char s1[10], s2[10], s3[10], s4[10];
    dtostrf(currentRMS[0], 6, 3, s1);
    dtostrf(currentRMS[1], 6, 3, s2);
    dtostrf(currentRMS[2], 6, 3, s3);
    dtostrf(currentRMS[3], 6, 3, s4);
    sprintf(buf, "$CUR,%s,%s,%s,%s\n", s1, s2, s3, s4);
    ESPSerial.print(buf);
    lastSend = millis();
  }

  // 3. Receive from ESP32
  while (ESPSerial.available()) {
    char c = ESPSerial.read();
    if (c == '\n') {
      dataStr.trim();
      if (dataStr.length() > 0) {
        Serial.print("Received -> ");
        Serial.println(dataStr);
        if (dataStr.startsWith("$ESP")) {
          parseVoltageRelay(dataStr);
          lastReceive = millis();
        }
      }
      dataStr = "";
    } else if (c != '\r') {
      dataStr += c;
    }
  }

  // 4. Update OLED
  static unsigned long lastDisplay = 0;
  if (millis() - lastDisplay >= 200) {
    updateDisplay();
    lastDisplay = millis();
  }

  // 5. Handle serial commands
  handleSerialCommands();
}

// ==================== Calibration (zero offset) ====================
void calibrateOffsets() {
  Serial.println("Calibrating offsets... (NO LOAD)");
  const int calSamples = 5000;
  float iSum[4] = {0,0,0,0};
  for (int i = 0; i < calSamples; i++) {
    for (int ch = 0; ch < 4; ch++) {
      int raw = analogRead(currentPins[ch]);
      float v = (raw / (float)ADC_RES) * ADC_REF;
      iSum[ch] += v;
    }
    delayMicroseconds(1000);
  }
  for (int ch = 0; ch < 4; ch++) {
    iDCOffset[ch] = iSum[ch] / calSamples;
    Serial.printf("CH%d offset = %.3f V\n", ch+1, iDCOffset[ch]);
  }
  Serial.println("Zero offset calibration done.");
}

// ==================== Serial Commands ====================
void handleSerialCommands() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("CAL")) {
      // Format: CAL1 1.05
      int ch = cmd.substring(3,4).toInt() - 1;
      if (ch >= 0 && ch < 4) {
        float val = cmd.substring(5).toFloat();
        if (val > 0) {
          CURRENT_CAL[ch] = val;
          Serial.printf("CH%d gain set to %.3f\n", ch+1, CURRENT_CAL[ch]);
        } else {
          Serial.println("Invalid gain (must be > 0)");
        }
      } else {
        Serial.println("Invalid channel (1-4)");
      }
    }
    else if (cmd == "SHOW") {
      debugPrintCurrent();
    }
    else if (cmd == "RESET") {
      for (int i = 0; i < 4; i++) CURRENT_CAL[i] = 1.00;
      Serial.println("All gains reset to 1.00");
    }
    else {
      Serial.println("Commands: CAL<ch> <gain>  e.g. CAL1 1.05");
      Serial.println("          SHOW              print current values");
      Serial.println("          RESET             reset all gains");
    }
  }
}

// ==================== Debug Print ====================
void debugPrintCurrent() {
  Serial.println("=== Current Readings ===");
  for (int ch = 0; ch < 4; ch++) {
    float rawA = currentRMS[ch] / CURRENT_CAL[ch];
    Serial.printf("CH%d: raw=%.3f A, calibrated=%.3f A, gain=%.3f\n",
                  ch+1, rawA, currentRMS[ch], CURRENT_CAL[ch]);
  }
  Serial.println();
}

// ==================== Parse $ESP ====================
void parseVoltageRelay(String data) {
  int first = data.indexOf(',');
  if (first == -1) return;
  int second = data.indexOf(',', first + 1);
  if (second == -1) return;
  ipAddress = data.substring(first + 1, second);
  int start = second + 1;
  float values[8];
  for (int i = 0; i < 8; i++) {
    int comma = data.indexOf(',', start);
    if (comma == -1) {
      if (i == 7) values[i] = data.substring(start).toFloat();
      break;
    }
    values[i] = data.substring(start, comma).toFloat();
    start = comma + 1;
  }
  for (int ch = 0; ch < 4; ch++) {
    voltage[ch] = values[ch];
    relay[ch] = (values[ch + 4] > 0.5);
  }
}

// ==================== OLED Update ====================
void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  bool connected = (millis() - lastReceive < 3000);

  if (connected) {
    if ((millis() / 500) % 2 == 0) display.fillCircle(124,4,3,SSD1306_WHITE);
    display.setCursor(0,0); display.println("Energy Monitor");
    for (int ch=0; ch<4; ch++) {
      int y = 8 + ch*10;
      display.setCursor(0,y);
      display.print("CH"); display.print(ch+1);
      display.print(" "); display.print(voltage[ch],0); display.print("V ");
      display.print(currentRMS[ch],2); display.print("A ");
      display.print(relay[ch] ? "ON " : "OFF");
    }
    display.setCursor(0,54); display.print("IP:"); display.print(ipAddress);
  } else {
    display.setTextSize(2);
    display.setCursor(20,20); display.println("NO");
    display.setCursor(20,40); display.println("DATA");
  }
  display.display();
}