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
 * ╠══════════════════════════════════════════════════════════════════╣
 * ║  HANDHELD CONTROLS:                                             ║
 * ║    Clutch (dead-man) MUST be held for any motion                ║
 * ║    Joystick  → Micro Manipulator XY gantry (DX / DY)           ║
 * ║    MPU pitch → IK Z axis (tilt forward/back = arm up/down)     ║
 * ║    MPU roll  → IK X axis (tilt left/right = arm left/right)    ║
 * ║    Scale UP  → 2× scale & +20% speed (up to 4× / 100%)         ║
 * ║    Scale DN  → ½ scale & -20% speed (down to 0.25× / 5%)       ║
 * ╠══════════════════════════════════════════════════════════════════╣
 * ║  LIBRARIES (Arduino Library Manager):                           ║
 * ║    - arduinoWebSockets  by Markus Sattler (WebSocketsServer.h)  ║
 * ║    - Adafruit NeoPixel                                          ║
 * ║    Board: "ESP32C3 Dev Module"                                   ║
 * ║    USB CDC On Boot: "Enabled"                                   ║
 * ╚══════════════════════════════════════════════════════════════════╝
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <WebSocketsServer.h>
#include <Adafruit_NeoPixel.h>

#include <SPIFFS.h>
#include <WebServer.h>

// ─── WIFI CONFIG ─────────────────────────────────────────────────────────────
const char* AP_SSID     = "STPE_Robot";
const char* AP_PASSWORD = "stpe1234";    // min 8 chars or use "" for open
const uint8_t AP_CHANNEL = 1;           // ESP-NOW & WiFi must share this channel

// ─── TEENSY UART ─────────────────────────────────────────────────────────────
#define TEENSY_RX_PIN  20
#define TEENSY_TX_PIN  21
#define TEENSY_BAUD    115200

// ─── LED ─────────────────────────────────────────────────────────────────────
#define PIN_LED  10   // WS2812

// ─── WEBSOCKET PORT ──────────────────────────────────────────────────────────
#define WS_PORT  81  // must match index_esp32.html WS_PORT constant

// ─── MOTION PARAMETERS ───────────────────────────────────────────────────────
#define MPU_SEND_INTERVAL_MS    80   // IK command rate from MPU (ms) — match to MOV duration
#define JOG_SEND_INTERVAL_MS   100   // DX/DY micro-manipulator rate (ms)
#define HANDHELD_TIMEOUT_MS   3000   // ms without packet = lost
#define MPU_DEADZONE           2.5f  // degrees: below this = no movement
#define MPU_SENSITIVITY        1.5f  // mm/degree/interval (X and Y)
#define MPU_Z_STEP             2.5f  // mm/interval (Height Z via buttons)
#define PITCH_SIGN            +1     
#define ROLL_SIGN             +1     
#define IK_Z_HOME            215.0f  
#define IK_X_MIN            -280.0f
#define IK_X_MAX             280.0f
#define IK_Y_MIN            -280.0f
#define IK_Y_MAX             280.0f
#define IK_Z_MIN              30.0f
#define IK_Z_MAX             380.0f
#define MOV_DURATION_MS        0     // 0 = streaming/immediate mode: bypasses queue, lets AccelStepper smooth motion
#define JOG_STEP              20     

// ─── ESP-NOW PACKET (must match Handheld_Controller.ino) ─────────────────────
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

// ─── GLOBALS ─────────────────────────────────────────────────────────────────
WebSocketsServer  wsServer(WS_PORT);
Adafruit_NeoPixel statusLed(1, PIN_LED, NEO_GRB + NEO_KHZ800);
WebServer       httpServer(80);

// Handheld state — packet buffer shared between ISR and main loop
// Use a double-buffer: ISR writes to rxBuf, main loop reads from workBuf
HandheldPacket    rxBuf;                      // written by ESP-NOW ISR only
HandheldPacket    workBuf;                    // read only in loop()
volatile bool     newHandheldPkt   = false;   // set in ISR, cleared in loop
volatile bool     handheldJustConnected = false; // set in ISR, logged in loop
bool              handheldOnline   = false;
unsigned long     lastHandheldTime = 0;

// Scale / speed (affected by handheld scale buttons)
float   scaleFactor   = 1.0f;   // 0.25x, 0.5x, 1.0x, 2.0x, 4.0x
int     currentSpeed  = 100;    // 5–100%

// IK target (tracked incrementally so MPU nudges accumulate)
float ikX = 0.0f, ikY = 0.0f, ikZ = IK_Z_HOME;

// Micro manipulator position (for UI display)
int microX = 0, microY = 0;

// Timing
unsigned long lastMpuCmdMs  = 0;
unsigned long lastJogCmdMs  = 0;

// Dead-man / clutch state
bool    clutchHeld  = false;
uint8_t prevDeadman = 0;

// Scale button debounce
uint8_t       prevScaleUp = 0, prevScaleDn = 0;
unsigned long scaleUpTime = 0, scaleDnTime = 0;
const uint32_t SCALE_DEBOUNCE_MS = 250;

// Teensy receive buffer
String teensyBuf = "";

// WebSocket client count
uint8_t wsClients = 0;

// ─── Forward declarations (needed because callbacks are inside #if blocks) ────
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
void IRAM_ATTR onEspNowReceive(const esp_now_recv_info_t* info, const uint8_t* data, int len);
#else
void IRAM_ATTR onEspNowReceive(const uint8_t* mac, const uint8_t* data, int len);
#endif

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println("\n[STPE] Master Bridge starting...");

  // ─ Status LED ─
  statusLed.begin();
  statusLed.setBrightness(25);
  setLed(0, 0, 60); // blue = booting

  // ─ Teensy UART ─
  Serial1.begin(TEENSY_BAUD, SERIAL_8N1, TEENSY_RX_PIN, TEENSY_TX_PIN);
  Serial.printf("[STPE] Teensy UART on RX=%d TX=%d @ %d baud\n",
                TEENSY_RX_PIN, TEENSY_TX_PIN, TEENSY_BAUD);

  // ─ WiFi AP (must be AP_STA for ESP-NOW to work on same channel) ─
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL);
  delay(200);
  Serial.print("[STPE] AP started  SSID: "); Serial.println(AP_SSID);
  Serial.print("[STPE] AP IP: ");            Serial.println(WiFi.softAPIP());
  Serial.print("[STPE] Bridge MAC: ");       Serial.println(WiFi.macAddress());
  Serial.printf("[STPE] Copy MAC above → Handheld firmware MASTER_MAC[]\n");

  // ─ WebSocket server ─
  wsServer.begin();
  wsServer.onEvent(wsEvent);
  wsServer.enableHeartbeat(5000, 3000, 2);
  Serial.printf("[STPE] WebSocket server on port %d (heartbeat enabled)\n", WS_PORT);

  // ─ SPIFFS + HTTP server (serves the web UI from the ESP) ─
  if (!SPIFFS.begin(true)) {
    Serial.println("[ERR] SPIFFS mount failed");
  } else {
    Serial.println("[FS] SPIFFS mounted");
    // Serve root -> index_esp32.html
    httpServer.on("/", HTTP_GET, [](){
      if (!handleFileRead("/index_esp32.html")) httpServer.send(404, "text/plain", "Not Found");
    });
    // Serve any file present on SPIFFS
    httpServer.onNotFound([](){
      String uri = httpServer.uri();
      if (!handleFileRead(uri)) httpServer.send(404, "text/plain", "Not Found");
    });
    httpServer.begin();
    Serial.println("[HTTP] File server started on port 80");
  }

  // ─ ESP-NOW ─
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERR] ESP-NOW init failed");
    setLed(80, 0, 0);
  } else {
    esp_now_register_recv_cb(onEspNowReceive);
    Serial.println("[STPE] ESP-NOW ready — waiting for handheld...");
  }

  setLed(0, 0, 20); // dim blue = idle, waiting for connections
  Serial.println("[STPE] Boot complete.\n");
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  wsServer.loop();           // WebSocket events
  httpServer.handleClient(); // Serve SPIFFS web UI requests
  readTeensy();              // Forward Teensy→WS
  processHandheld();         // Handle ESP-NOW data
  checkHandheldTimeout();    // Detect lost handheld
  updateLedStatus();         // Status LED
}

// ══════════════════════════════════════════════════════════════════════════════
// ESP-NOW RECEIVE CALLBACK
// !! RUNS IN WIFI TASK CONTEXT — NO Serial, no heap alloc, no blocking calls !!
// SDK v3.x (IDF v5): signature uses esp_now_recv_info_t* instead of mac ptr
// ══════════════════════════════════════════════════════════════════════════════
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
void IRAM_ATTR onEspNowReceive(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
#else
void IRAM_ATTR onEspNowReceive(const uint8_t* mac, const uint8_t* data, int len) {
#endif
  if (len == sizeof(HandheldPacket)) {
    memcpy(&rxBuf, data, sizeof(HandheldPacket)); // only memcpy — ISR safe
    newHandheldPkt = true;
    if (!handheldOnline) handheldJustConnected = true; // logged by loop()
  }
}

// ══════════════════════════════════════════════════════════════════════════════
// PROCESS HANDHELD PACKET  (runs in loop() — safe to use Serial, WS, etc.)
// ══════════════════════════════════════════════════════════════════════════════
void processHandheld() {
  if (!newHandheldPkt) return;

  // Safely copy latest packet from ISR buffer — disable interrupts briefly
  noInterrupts();
  memcpy(&workBuf, &rxBuf, sizeof(HandheldPacket));
  newHandheldPkt       = false;
  lastHandheldTime     = millis();
  bool justConn        = handheldJustConnected;
  handheldJustConnected = false;
  interrupts();

  HandheldPacket h = workBuf;
  unsigned long now = millis();

  // Log first connection here in main loop (Serial is safe here)
  if (justConn) {
    handheldOnline = true;
    Serial.println("[ESP-NOW] Handheld connected");
  }

  // ── Scale buttons (debounced, rising-edge detect) ──────────────────────────
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

  // ── Clutch (dead-man) state change ─────────────────────────────────────────
  if (h.btn_deadman != prevDeadman) {
    clutchHeld  = h.btn_deadman;
    prevDeadman = h.btn_deadman;
    broadcastHandheldStatus(h); // notify UI immediately
    Serial.printf("[CLUTCH] %s\n", clutchHeld ? "HELD — motion enabled" : "RELEASED — motion blocked");
  }

  // ── Always broadcast status to WebSocket UI ────────────────────────────────
  broadcastHandheldStatus(h);

  // ── DEAD-MAN NOT HELD → NO MOTION ─────────────────────────────────────────
  if (!h.btn_deadman) return;

  // ── Joystick buttons → Z axis (Up/Down) ───────────────────────────────────
  float dZ = 0;
  if (h.btn_up)   dZ = +MPU_Z_STEP * scaleFactor;
  if (h.btn_down) dZ = -MPU_Z_STEP * scaleFactor;
  if (dZ != 0) {
    ikZ = constrain(ikZ + dZ, IK_Z_MIN, IK_Z_MAX);
  }

  // ── MPU6050 pitch/roll → Incremental Cartesian (X/Y) ──────────────────────
  if (now - lastMpuCmdMs >= MPU_SEND_INTERVAL_MS) {
    float p = h.pitch;
    float r = h.roll;

    if (fabsf(p) < MPU_DEADZONE) p = 0.0f;
    if (fabsf(r) < MPU_DEADZONE) r = 0.0f;

    // Check if we have MPU input OR if Z was just moved by buttons
    if (p != 0.0f || r != 0.0f || dZ != 0.0f) {
      lastMpuCmdMs = now;

      // Map: tilt FORWARD (positive pitch) → arm moves in Y+ (forward)
      //      tilt RIGHT   (positive roll)  → arm moves in X+ (right)
      float dX = ROLL_SIGN  * r * MPU_SENSITIVITY * scaleFactor;
      float dY = PITCH_SIGN * p * MPU_SENSITIVITY * scaleFactor;

      ikX = constrain(ikX + dX, IK_X_MIN, IK_X_MAX);
      ikY = constrain(ikY + dY, IK_Y_MIN, IK_Y_MAX);

      // Final reach check (95mm min)
      float curR = sqrt(ikX*ikX + ikY*ikY);
      if (curR < 95.0f && curR > 0.1f) {
        // Push out to 95mm if too close to base
        float ratio = 95.0f / curR;
        ikX *= ratio;
        ikY *= ratio;
      }

      // Flush any stale queued waypoints so only the latest target matters
      sendToTeensy("STOP");

      // Format: MOV X<n> Y<n> Z<n> W0 J490 J590 D<ms>
      // D0 = immediate/streaming mode: bypasses waypoint queue,
      //      AccelStepper handles velocity smoothing natively.
      String cmd = "MOV X" + String((int)ikX)
                 + " Y"    + String((int)ikY)
                 + " Z"    + String((int)ikZ)
                 + " W0 J490 J590 D" + String(MOV_DURATION_MS);
      sendToTeensy(cmd);
    }
  }
}

// ── Scale factor step helper ───────────────────────────────────────────────
void adjustScale(int dir) {
  // Scale steps: 0.25 → 0.5 → 1.0 → 2.0 → 4.0
  const float steps[] = { 0.25f, 0.5f, 1.0f, 2.0f, 4.0f };
  const int   numSteps = 5;
  int idx = 2; // default 1.0

  // Find current step
  for (int i = 0; i < numSteps; i++) {
    if (fabsf(scaleFactor - steps[i]) < 0.01f) { idx = i; break; }
  }
  idx = constrain(idx + dir, 0, numSteps - 1);
  scaleFactor = steps[idx];

  // Also adjust speed proportionally
  if (dir > 0) currentSpeed = min(currentSpeed + 20, 100);
  else         currentSpeed = max(currentSpeed - 20, 5);

  sendToTeensy("SPEED " + String(currentSpeed));

  Serial.printf("[SCALE] %.2fx  SPEED %d%%\n", scaleFactor, currentSpeed);

  // Notify UI
  char buf[128];
  snprintf(buf, sizeof(buf),
    "SCALE_UPDATE {\"scale\":%.2f,\"speed\":%d}",
    scaleFactor, currentSpeed);
  wsServer.broadcastTXT(buf);
}

// ── Broadcast handheld telemetry to all WebSocket clients ─────────────────
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

// ── Detect handheld going offline ──────────────────────────────────────────
void checkHandheldTimeout() {
  if (handheldOnline && (millis() - lastHandheldTime > HANDHELD_TIMEOUT_MS)) {
    handheldOnline = false;
    clutchHeld     = false;
    Serial.println("[WARN] Handheld timeout — motion disabled");
    wsServer.broadcastTXT("REMOTE_DATA:{\"online\":0,\"clutch\":0}");
  }
}

// ══════════════════════════════════════════════════════════════════════════════
// TEENSY UART — READ & FORWARD
// ══════════════════════════════════════════════════════════════════════════════
void readTeensy() {
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '\n') {
      teensyBuf.trim();
      if (teensyBuf.length() > 0) {
        wsServer.broadcastTXT(teensyBuf); // Forward to all WebSocket clients
        Serial.println("[T←] " + teensyBuf);
        
        // --- SYNC LOCAL IK MEMORY WITH TRUE PHYSICAL POSITION ---
        // AR4 Telemetry format: A{j1}B{j2}C{j3}...G{X}H{Y}I{Z}
        if (teensyBuf.startsWith("A") && teensyBuf.indexOf('B') > 0 && teensyBuf.indexOf('G') > 0) {
            int gIdx = teensyBuf.indexOf('G');
            int hIdx = teensyBuf.indexOf('H');
            int iIdx = teensyBuf.indexOf('I');
            if (gIdx != -1 && hIdx != -1 && iIdx != -1) {
                // If clutch is NOT held, blindly trust the physical position!
                // (If clutch is held, we are overriding it, so we rely on our constrained values)
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
      if (teensyBuf.length() > 512) teensyBuf = ""; // Overflow guard
    }
  }
}

void sendToTeensy(const String& cmd) {
  Serial1.print(cmd);
  Serial1.print('\n');
  Serial.println("[T→] " + cmd);
}

// ══════════════════════════════════════════════════════════════════════════════
// WEBSOCKET SERVER (browser commands → Teensy)
// ══════════════════════════════════════════════════════════════════════════════
void wsEvent(uint8_t clientNum, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {

    case WStype_CONNECTED:
      wsClients++;
      Serial.printf("[WS] Client #%d connected from %s  (total: %d)\n",
                    clientNum,
                    wsServer.remoteIP(clientNum).toString().c_str(),
                    wsClients);
      // Send current state to new client
      {
        char buf[200];
        snprintf(buf, sizeof(buf),
          "REMOTE_DATA:{\"scale\":%.2f,\"speed\":%d,\"clutch\":%d,\"online\":%d}",
          scaleFactor, currentSpeed, (int)clutchHeld, (int)handheldOnline);
        wsServer.sendTXT(clientNum, buf);
      }
      // Request position telemetry to synchronize local IK memory immediately
      sendToTeensy("RP");
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

      // ── Handle bridge-level commands ──
      if (msg.startsWith("SPEED ")) {
        currentSpeed = constrain(msg.substring(6).toInt(), 5, 100);
        sendToTeensy(msg); // also forward to Teensy

      } else if (msg == "ESTOP") {
        sendToTeensy("ESTOP");
        // Do not hardcode ikZ = IK_Z_HOME to prevent sudden Z-axis jumps on clutch restart;
        // Request fresh physical position from Teensy to resync
        sendToTeensy("RP");

      } else if (msg == "REENABLE") {
        sendToTeensy("REENABLE");

      } else if (msg == "RP") {
        // Request position telemetry
        sendToTeensy("RP");

      } else {
        // All other commands (MOV, D1, D2, A4, HOME, DX, DY, DIAG, etc.)
        sendToTeensy(msg);

        // If browser sends a MOV, sync local IK tracking
        if (msg.startsWith("MOV ")) {
          parseMovUpdate(msg);
        }
      }
      break;
    }

    default: break;
  }
}

// Parse MOV command from browser to keep local IK state in sync
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

// ══════════════════════════════════════════════════════════════════════════════
// STATUS LED
// ══════════════════════════════════════════════════════════════════════════════
void setLed(uint8_t r, uint8_t g, uint8_t b) {
  statusLed.setPixelColor(0, statusLed.Color(r, g, b));
  statusLed.show();
}

void updateLedStatus() {
  static unsigned long lastBlink = 0;
  static bool          blinkOn   = false;
  unsigned long now = millis();

  // Blink period
  if (now - lastBlink > 500) { lastBlink = now; blinkOn = !blinkOn; }

  if (!handheldOnline && wsClients == 0) {
    // No connections at all — slow blue pulse
    setLed(0, 0, blinkOn ? 20 : 5);
  } else if (!handheldOnline) {
    // Only browser connected — steady cyan
    setLed(0, 20, 20);
  } else if (!clutchHeld) {
    // Handheld online but clutch not held — steady green
    setLed(0, 25, 0);
  } else {
    // Clutch held — ACTIVE — bright green fast blink
    if (now - lastBlink > 120) { lastBlink = now; blinkOn = !blinkOn; }
    setLed(0, blinkOn ? 80 : 30, 0);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// SPIFFS / HTTP helpers
// ─────────────────────────────────────────────────────────────────────────────
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
