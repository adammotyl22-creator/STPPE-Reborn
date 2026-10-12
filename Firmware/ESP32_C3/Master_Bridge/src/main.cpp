/**
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║   STPE MASTER BRIDGE — Waveshare ESP32-C3 Zero                  ║
 * ║   Wireless gateway between Handheld + Web UI → Teensy 4.1       ║
 * ╠══════════════════════════════════════════════════════════════════╣
 * ║  CONNECTIONS:                                                    ║
 * ║    GPIO 20 (RX) ← Teensy TX (Serial1)                          ║
 * ║    GPIO 21 (TX) → Teensy RX (Serial1)                          ║
 * ║    GPIO 10       → WS2812 RGB LED                                ║
 * ║    GND ↔ GND (share with Teensy)                               ║
 * ╠══════════════════════════════════════════════════════════════════╣
 * ║  SYSTEM ARCHITECTURE:                                           ║
 * ║                                                                 ║
 * ║   [Handheld ESP-C3]──ESP-NOW──▶[THIS]◀──UART──▶[Teensy 4.1]   ║
 * ║                                   │                            ║
 * ║                             WebSocket :81                      ║
 * ║                                   │                            ║
 * ║                         [Browser / index_esp32.html]           ║
 * ╚══════════════════════════════════════════════════════════════════╝
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <WebSocketsServer.h>
#include <Adafruit_NeoPixel.h>
#include <SPIFFS.h>
#include <WebServer.h>

const char* AP_SSID     = "STPE_Robot";
const char* AP_PASSWORD = "stpe1234";
const uint8_t AP_CHANNEL = 1;

#define TEENSY_RX_PIN  20
#define TEENSY_TX_PIN  21
#define TEENSY_BAUD    115200
#define PIN_LED  10
#define WS_PORT  81

#define MPU_SEND_INTERVAL_MS    80
#define JOG_SEND_INTERVAL_MS   100
#define HANDHELD_TIMEOUT_MS   3000
#define MPU_DEADZONE           2.5f
#define MPU_SENSITIVITY        1.5f
#define MPU_Z_STEP             2.5f
#define PITCH_SIGN            +1
#define ROLL_SIGN             +1
#define IK_Z_HOME            215.0f
#define IK_X_MIN            -280.0f
#define IK_X_MAX             280.0f
#define IK_Y_MIN            -280.0f
#define IK_Y_MAX             280.0f
#define IK_Z_MIN              30.0f
#define IK_Z_MAX             380.0f
#define MOV_DURATION_MS        0
#define JOG_STEP              20

#pragma pack(1)
struct HandheldPacket {
  float   pitch;
  float   roll;
  uint8_t btn_up;
  uint8_t btn_down;
  uint8_t btn_left;
  uint8_t btn_right;
  uint8_t btn_scale_up;
  uint8_t btn_scale_down;
  uint8_t btn_deadman;
  uint8_t battery;
  uint8_t rssi;
};
#pragma pack()

WebSocketsServer  wsServer(WS_PORT);
Adafruit_NeoPixel statusLed(1, PIN_LED, NEO_GRB + NEO_KHZ800);
WebServer       httpServer(80);

HandheldPacket    rxBuf;
HandheldPacket    workBuf;
volatile bool     newHandheldPkt   = false;
volatile bool     handheldJustConnected = false;
bool              handheldOnline   = false;
unsigned long     lastHandheldTime = 0;

float   scaleFactor   = 1.0f;
int     currentSpeed  = 100;

float ikX = 0.0f, ikY = 0.0f, ikZ = IK_Z_HOME;
int microX = 0, microY = 0;

unsigned long lastMpuCmdMs  = 0;
unsigned long lastJogCmdMs  = 0;

bool    clutchHeld  = false;
uint8_t prevDeadman = 0;

uint8_t       prevScaleUp = 0, prevScaleDn = 0;
unsigned long scaleUpTime = 0, scaleDnTime = 0;
const uint32_t SCALE_DEBOUNCE_MS = 250;

String teensyBuf = "";
uint8_t wsClients = 0;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
void IRAM_ATTR onEspNowReceive(const esp_now_recv_info_t* info, const uint8_t* data, int len);
#else
void IRAM_ATTR onEspNowReceive(const uint8_t* mac, const uint8_t* data, int len);
#endif

void setLed(uint8_t r, uint8_t g, uint8_t b);
void updateLedStatus();
void readTeensy();
void processHandheld();
void checkHandheldTimeout();
void adjustScale(int dir);
void broadcastHandheldStatus(const HandheldPacket& h);
void wsEvent(uint8_t clientNum, WStype_t type, uint8_t* payload, size_t length);
void parseMovUpdate(const String& cmd);
void sendToTeensy(const String& cmd);
String getContentType(const String& filename);
bool handleFileRead(const String& path);

void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println("\n[STPE] Master Bridge starting...");

  statusLed.begin();
  statusLed.setBrightness(25);
  setLed(0, 0, 60);

  Serial1.begin(TEENSY_BAUD, SERIAL_8N1, TEENSY_RX_PIN, TEENSY_TX_PIN);
  Serial.printf("[STPE] Teensy UART on RX=%d TX=%d @ %d baud\n",
                TEENSY_RX_PIN, TEENSY_TX_PIN, TEENSY_BAUD);

  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL);
  delay(200);
  Serial.print("[STPE] AP started  SSID: "); Serial.println(AP_SSID);
  Serial.print("[STPE] AP IP: ");            Serial.println(WiFi.softAPIP());
  Serial.print("[STPE] Bridge MAC: ");       Serial.println(WiFi.macAddress());
  Serial.printf("[STPE] Copy MAC above → Handheld firmware MASTER_MAC[]\n");

  wsServer.begin();
  wsServer.onEvent(wsEvent);
  Serial.printf("[STPE] WebSocket server on port %d\n", WS_PORT);

  if (!SPIFFS.begin(true)) {
    Serial.println("[ERR] SPIFFS mount failed");
  } else {
    Serial.println("[FS] SPIFFS mounted");
    httpServer.on("/", HTTP_GET, [](){
      if (!handleFileRead("/index_esp32.html")) httpServer.send(404, "text/plain", "Not Found");
    });
    httpServer.onNotFound([](){
      String uri = httpServer.uri();
      if (!handleFileRead(uri)) httpServer.send(404, "text/plain", "Not Found");
    });
    httpServer.begin();
    Serial.println("[HTTP] File server started on port 80");
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERR] ESP-NOW init failed");
    setLed(80, 0, 0);
  } else {
    esp_now_register_recv_cb(onEspNowReceive);
    Serial.println("[STPE] ESP-NOW ready — waiting for handheld...");
  }

  setLed(0, 0, 20);
  Serial.println("[STPE] Boot complete.\n");
}

void loop() {
  wsServer.loop();
  httpServer.handleClient();
  readTeensy();
  processHandheld();
  checkHandheldTimeout();
  updateLedStatus();
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
void IRAM_ATTR onEspNowReceive(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
#else
void IRAM_ATTR onEspNowReceive(const uint8_t* mac, const uint8_t* data, int len) {
#endif
  if (len == sizeof(HandheldPacket)) {
    memcpy(&rxBuf, data, sizeof(HandheldPacket));
    newHandheldPkt = true;
    if (!handheldOnline) handheldJustConnected = true;
  }
}

void processHandheld() {
  if (!newHandheldPkt) return;

  noInterrupts();
  memcpy(&workBuf, &rxBuf, sizeof(HandheldPacket));
  newHandheldPkt       = false;
  lastHandheldTime     = millis();
  bool justConn        = handheldJustConnected;
  handheldJustConnected = false;
  interrupts();

  HandheldPacket h = workBuf;
  unsigned long now = millis();

  if (justConn) {
    handheldOnline = true;
    Serial.println("[ESP-NOW] Handheld connected");
  }

  if (h.btn_scale_up && !prevScaleUp && (now - scaleUpTime > SCALE_DEBOUNCE_MS)) {
    scaleUpTime = now;
    adjustScale(+1);
  }
  if (h.btn_scale_down && !prevScaleDn && (now - scaleDnTime > SCALE_DEBOUNCE_MS)) {
    scaleDnTime = now;
    adjustScale(-1);
  }
  prevScaleUp = h.btn_scale_up;
  prevScaleDn = h.btn_scale_down;

  if (h.btn_deadman != prevDeadman) {
    clutchHeld  = h.btn_deadman;
    prevDeadman = h.btn_deadman;
    broadcastHandheldStatus(h);
    Serial.printf("[CLUTCH] %s\n", clutchHeld ? "HELD — motion enabled" : "RELEASED — motion blocked");
  }

  broadcastHandheldStatus(h);

  if (!h.btn_deadman) return;

  float dZ = 0;
  if (h.btn_up)   dZ = +MPU_Z_STEP * scaleFactor;
  if (h.btn_down) dZ = -MPU_Z_STEP * scaleFactor;
  if (dZ != 0) {
    ikZ = constrain(ikZ + dZ, IK_Z_MIN, IK_Z_MAX);
  }

  if (now - lastMpuCmdMs >= MPU_SEND_INTERVAL_MS) {
    float p = h.pitch;
    float r = h.roll;

    if (fabsf(p) < MPU_DEADZONE) p = 0.0f;
    if (fabsf(r) < MPU_DEADZONE) r = 0.0f;

    if (p != 0.0f || r != 0.0f || dZ != 0.0f) {
      lastMpuCmdMs = now;

      float dX = ROLL_SIGN  * r * MPU_SENSITIVITY * scaleFactor;
      float dY = PITCH_SIGN * p * MPU_SENSITIVITY * scaleFactor;

      ikX = constrain(ikX + dX, IK_X_MIN, IK_X_MAX);
      ikY = constrain(ikY + dY, IK_Y_MIN, IK_Y_MAX);

      float curR = sqrt(ikX*ikX + ikY*ikY);
      if (curR < 95.0f && curR > 0.1f) {
        float ratio = 95.0f / curR;
        ikX *= ratio;
        ikY *= ratio;
      }

      sendToTeensy("STOP");

      String cmd = "MOV X" + String((int)ikX)
                 + " Y"    + String((int)ikY)
                 + " Z"    + String((int)ikZ)
                 + " W0 J490 J590 D" + String(MOV_DURATION_MS);
      sendToTeensy(cmd);
    }
  }
}

void adjustScale(int dir) {
  const float steps[] = { 0.25f, 0.5f, 1.0f, 2.0f, 4.0f };
  const int   numSteps = 5;
  int idx = 2;

  for (int i = 0; i < numSteps; i++) {
    if (fabsf(scaleFactor - steps[i]) < 0.01f) { idx = i; break; }
  }
  idx = constrain(idx + dir, 0, numSteps - 1);
  scaleFactor = steps[idx];

  if (dir > 0) currentSpeed = min(currentSpeed + 20, 100);
  else         currentSpeed = max(currentSpeed - 20, 5);

  sendToTeensy("SPEED " + String(currentSpeed));

  Serial.printf("[SCALE] %.2fx  SPEED %d%%\n", scaleFactor, currentSpeed);

  char buf[128];
  snprintf(buf, sizeof(buf),
    "SCALE_UPDATE {\"scale\":%.2f,\"speed\":%d}",
    scaleFactor, currentSpeed);
  wsServer.broadcastTXT(buf);
}

void broadcastHandheldStatus(const HandheldPacket& h) {
  char buf[300];
  snprintf(buf, sizeof(buf),
    "REMOTE_DATA:{\"pitch\":%.1f,\"roll\":%.1f,"
    "\"clutch\":%d,\"scale\":%.2f,\"speed\":%d,"
    "\"bat\":%d,\"up\":%d,\"dn\":%d,\"lt\":%d,\"rt\":%d,"
    "\"su\":%d,\"sd\":%d,\"online\":1}",
    h.pitch, h.roll,
    h.btn_deadman, scaleFactor, currentSpeed,
    h.battery,
    h.btn_up, h.btn_down, h.btn_left, h.btn_right,
    h.btn_scale_up, h.btn_scale_down
  );
  wsServer.broadcastTXT(buf);
}

void checkHandheldTimeout() {
  if (handheldOnline && (millis() - lastHandheldTime > HANDHELD_TIMEOUT_MS)) {
    handheldOnline = false;
    clutchHeld     = false;
    Serial.println("[WARN] Handheld timeout — motion disabled");
    wsServer.broadcastTXT("REMOTE_DATA:{\"online\":0,\"clutch\":0}");
  }
}

void readTeensy() {
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '\n') {
      teensyBuf.trim();
      if (teensyBuf.length() > 0) {
        wsServer.broadcastTXT(teensyBuf);
        Serial.println("[T←] " + teensyBuf);
        
        if (teensyBuf.startsWith("A") && teensyBuf.indexOf('B') > 0 && teensyBuf.indexOf('G') > 0) {
            int gIdx = teensyBuf.indexOf('G');
            int hIdx = teensyBuf.indexOf('H');
            int iIdx = teensyBuf.indexOf('I');
            if (gIdx != -1 && hIdx != -1 && iIdx != -1) {
                if (!clutchHeld) {
                    ikX = teensyBuf.substring(gIdx+1, hIdx).toFloat();
                    ikY = teensyBuf.substring(hIdx+1, iIdx).toFloat();
                    ikZ = teensyBuf.substring(iIdx+1).toFloat();
                }
            }
        }
      }
      teensyBuf = "";
    } else {
      teensyBuf += c;
      if (teensyBuf.length() > 512) teensyBuf = "";
    }
  }
}

void sendToTeensy(const String& cmd) {
  Serial1.print(cmd);
  Serial1.print('\n');
  Serial.println("[T→] " + cmd);
}

void wsEvent(uint8_t clientNum, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {

    case WStype_CONNECTED:
      wsClients++;
      Serial.printf("[WS] Client #%d connected from %s  (total: %d)\n",
                    clientNum,
                    wsServer.remoteIP(clientNum).toString().c_str(),
                    wsClients);
      {
        char buf[200];
        snprintf(buf, sizeof(buf),
          "REMOTE_DATA:{\"scale\":%.2f,\"speed\":%d,\"clutch\":%d,\"online\":%d}",
          scaleFactor, currentSpeed, (int)clutchHeld, (int)handheldOnline);
        wsServer.sendTXT(clientNum, buf);
      }
      break;

    case WStype_DISCONNECTED:
      if (wsClients > 0) wsClients--;
      Serial.printf("[WS] Client #%d disconnected  (remaining: %d)\n", clientNum, wsClients);
      break;

    case WStype_TEXT: {
      String msg = String((char*)payload).substring(0, length);
      msg.trim();
      if (msg.length() == 0) break;
      Serial.println("[WS→T] " + msg);

      if (msg.startsWith("SPEED ")) {
        currentSpeed = constrain(msg.substring(6).toInt(), 5, 100);
        sendToTeensy(msg);

      } else if (msg == "ESTOP") {
        sendToTeensy("ESTOP");
        ikX = 0; ikY = 0; ikZ = IK_Z_HOME;

      } else if (msg == "REENABLE") {
        sendToTeensy("REENABLE");

      } else if (msg == "RP") {
        sendToTeensy("RP");

      } else {
        sendToTeensy(msg);

        if (msg.startsWith("MOV ")) {
          parseMovUpdate(msg);
        }
      }
      break;
    }

    default: break;
  }
}

void parseMovUpdate(const String& cmd) {
  auto extractVal = [&](char prefix) -> float {
    int idx = cmd.indexOf(prefix);
    if (idx < 0) return NAN;
    float v = cmd.substring(idx + 1).toFloat();
    return v;
  };
  float x = extractVal('X'), y = extractVal('Y'), z = extractVal('Z');
  if (!isnan(x)) ikX = constrain(x, IK_X_MIN, IK_X_MAX);
  if (!isnan(y)) ikY = constrain(y, IK_Y_MIN, IK_Y_MAX);
  if (!isnan(z)) ikZ = constrain(z, IK_Z_MIN, IK_Z_MAX);
}

void setLed(uint8_t r, uint8_t g, uint8_t b) {
  statusLed.setPixelColor(0, statusLed.Color(r, g, b));
  statusLed.show();
}

void updateLedStatus() {
  static unsigned long lastBlink = 0;
  static bool          blinkOn   = false;
  unsigned long now = millis();

  if (now - lastBlink > 500) { lastBlink = now; blinkOn = !blinkOn; }

  if (!handheldOnline && wsClients == 0) {
    setLed(0, 0, blinkOn ? 20 : 5);
  } else if (!handheldOnline) {
    setLed(0, 20, 20);
  } else if (!clutchHeld) {
    setLed(0, 25, 0);
  } else {
    if (now - lastBlink > 120) { lastBlink = now; blinkOn = !blinkOn; }
    setLed(0, blinkOn ? 80 : 30, 0);
  }
}

String getContentType(const String& filename) {
  if (filename.endsWith(".htm") || filename.endsWith(".html")) return "text/html";
  if (filename.endsWith(".css")) return "text/css";
  if (filename.endsWith(".js")) return "application/javascript";
  if (filename.endsWith(".png")) return "image/png";
  if (filename.endsWith(".jpg") || filename.endsWith(".jpeg")) return "image/jpeg";
  if (filename.endsWith(".svg")) return "image/svg+xml";
  if (filename.endsWith(".json")) return "application/json";
  if (filename.endsWith(".ico")) return "image/x-icon";
  return "text/plain";
}

bool handleFileRead(const String& path) {
  String filePath = path;
  if (filePath.endsWith("/")) filePath += "index_esp32.html";
  if (!filePath.startsWith("/")) filePath = "/" + filePath;
  if (!SPIFFS.exists(filePath)) return false;
  File file = SPIFFS.open(filePath, "r");
  if (!file) return false;
  String contentType = getContentType(filePath);
  httpServer.streamFile(file, contentType);
  file.close();
  return true;
}
