/**
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║   STPE HANDHELD CONTROLLER — Waveshare ESP32-C3 Zero             ║
 * ║   v2.0 — Complementary Filter (Accel + Gyro fusion)              ║
 * ╠══════════════════════════════════════════════════════════════════╣
 * ║  PINOUT:                                                         ║
 * ║    GPIO 0  →                                                     ║
 * ║    GPIO 1  →                                                     ║
 * ║    GPIO 2  →                                                     ║
 * ║    GPIO 3  →                                                     ║
 * ║    GPIO 4  → Button SCALE UP                                     ║
 * ║    GPIO 5  → Dead-Man / Clutch switch (must hold for motion)     ║
 * ║    GPIO 6  → I2C SDA (MPU6050)                                   ║
 * ║    GPIO 7  → I2C SCL (MPU6050)                                   ║
 * ║    GPIO 8  → Battery ADC (voltage divider 10k+10k)               ║
 * ║    GPIO 9  → Button SCALE DOWN                                   ║
 * ║    GPIO 10 → Status LED                                          ║
 * ╚══════════════════════════════════════════════════════════════════╝
 *
 * KEY CHANGES vs v1.0:
 *   - Reads BOTH accelerometer (0x3B) AND gyroscope (0x43) registers
 *   - Complementary filter: angle = α*(angle + gyro*dt) + (1-α)*accel_angle
 *   - α = 0.95 (trust gyro for fast motion, accel corrects slow drift)
 *   - Output rate-limiter: max degrees/sec the arm is commanded to move
 *   - Dead zone applied AFTER fusion (not before), so it's stable
 *   - SCALE buttons now adjust output rate limit, not filter alpha
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>

// ─── SET THIS TO YOUR MASTER BRIDGE MAC ADDRESS ──────────────────────────────
uint8_t MASTER_MAC[] = { 0x10, 0x00, 0x3B, 0xDD, 0x98, 0x30 };

// ─── PIN DEFINITIONS ─────────────────────────────────────────────────────────
#define PIN_JOY_UP        0
#define PIN_JOY_DOWN      1
#define PIN_JOY_LEFT      2
#define PIN_JOY_RIGHT     3
#define PIN_BTN_SCALE_UP  4
#define PIN_BTN_DEADMAN   5
#define PIN_I2C_SDA       6
#define PIN_I2C_SCL       7
#define PIN_LED           10
#define PIN_BTN_SCALE_DN  9
#define PIN_VBAT          8

// ─── CONFIG ──────────────────────────────────────────────────────────────────
#define SEND_INTERVAL_MS   50        // 20 Hz send rate
#define VBAT_RATIO        2.0f
#define VBAT_MAX          4.2f
#define VBAT_MIN          3.3f

// Complementary filter coefficient.
// 0.95 = strongly trust gyro, accel only corrects slow drift.
// Lower = more accel influence = more jitter but less gyro drift.
// Do NOT change this unless you understand the tradeoff.
#define COMP_ALPHA        0.95f

// Dead zone: ignore tilt below this many degrees (after fusion).
// Prevents arm creep when controller is "still" on a table.
#define DEAD_ZONE_DEG     2.5f

// Output rate limit: max degrees/second the pitch/roll value can change.
// This is what SCALE buttons now control. Lower = smoother but slower.
// Range: 5–60 deg/s is practical for electronics work.
#define RATE_LIMIT_MIN    1.0f
#define RATE_LIMIT_MAX    30.0f
#define RATE_LIMIT_STEP   5.0f
#define RATE_LIMIT_DEFAULT 1.0f    // safe starting point

// MPU6050 gyro sensitivity at ±250°/s full scale: 131 LSB per deg/s
#define GYRO_SENSITIVITY  131.0f

// ─── DATA PACKET ─────────────────────────────────────────────────────────────
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

// ─── OBJECTS ─────────────────────────────────────────────────────────────────
Adafruit_NeoPixel   led(1, PIN_LED, NEO_GRB + NEO_KHZ800);
esp_now_peer_info_t peerInfo;

// ─── STATE ───────────────────────────────────────────────────────────────────
// Fused angles (output of complementary filter)
float fusedPitch = 0.0f;
float fusedRoll  = 0.0f;

// Gyro zero-rate bias (computed on startup)
float gyroBiasX  = 0.0f;
float gyroBiasY  = 0.0f;

// Rate-limited output (what actually gets sent)
float outputPitch = 0.0f;
float outputRoll  = 0.0f;

float rateLimit  = RATE_LIMIT_DEFAULT;  // deg/s, adjusted by SCALE buttons
bool  mpuOk      = false;
uint8_t txFails  = 0;

// Timing for complementary filter
unsigned long lastMpuRead = 0;

void calibrateGyro();

// ─── ESP-NOW Send Callback ────────────────────────────────────────────────────
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
#else
void onSent(const uint8_t *mac, esp_now_send_status_t status) {
#endif
  if (status == ESP_NOW_SEND_SUCCESS) {
    txFails = 0;
  } else {
    txFails = min((int)txFails + 1, 10);
    if (txFails == 1) Serial.println("[WARN] ESP-NOW delivery failed");
  }
}

// ─── SETUP ───────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("[STPE] Handheld Controller v2.0 starting...");

  pinMode(PIN_JOY_UP,       INPUT_PULLUP);
  pinMode(PIN_JOY_DOWN,     INPUT_PULLUP);
  pinMode(PIN_JOY_LEFT,     INPUT_PULLUP);
  pinMode(PIN_JOY_RIGHT,    INPUT_PULLUP);
  pinMode(PIN_BTN_SCALE_UP, INPUT_PULLUP);
  pinMode(PIN_BTN_SCALE_DN, INPUT_PULLUP);
  pinMode(PIN_BTN_DEADMAN,  INPUT_PULLUP);

  led.begin();
  led.setBrightness(30);
  setLed(0, 0, 50);  // Blue = starting

  // ── I2C + MPU6050 ──
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  delay(100);

  // Wake MPU6050
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);  // PWR_MGMT_1
  Wire.write(0x00);  // Wake up (clear sleep bit)
  bool wakeOk = (Wire.endTransmission(true) == 0);

  if (wakeOk) {
    // Set gyro full scale to ±250°/s (register 0x1B, value 0x00)
    Wire.beginTransmission(0x68);
    Wire.write(0x1B);
    Wire.write(0x00);
    Wire.endTransmission(true);

    // Set accel full scale to ±2g (register 0x1C, value 0x00)
    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.write(0x00);
    Wire.endTransmission(true);

    // Set DLPF to ~44Hz bandwidth (register 0x1A, value 0x03)
    // This hardware-filters vibration before we even read the data.
    Wire.beginTransmission(0x68);
    Wire.write(0x1A);
    Wire.write(0x03);
    Wire.endTransmission(true);

    mpuOk = true;
    Serial.println("[STPE] MPU6050 OK — gyro + accel + DLPF configured");
    calibrateGyro();
  } else {
    Serial.println("[WARN] MPU6050 failed to wake up!");
    setLed(50, 30, 0);  // Orange = MPU missing
  }

  // ── WiFi + ESP-NOW ──
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  delay(100);

  Serial.print("[STPE] MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERR] ESP-NOW init failed — halting");
    setLed(80, 0, 0);
    while (1) delay(1000);
  }

  esp_now_register_send_cb(onSent);
  memcpy(peerInfo.peer_addr, MASTER_MAC, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  lastMpuRead = micros();

  if (mpuOk) setLed(0, 60, 0);  // Green = ready
  Serial.printf("[STPE] Ready — rateLimit=%.0f deg/s, COMP_ALPHA=%.2f\n",
                rateLimit, COMP_ALPHA);
}

// ─── MAIN LOOP ───────────────────────────────────────────────────────────────
void loop() {
  static unsigned long lastSend = 0;
  unsigned long now = millis();

  // ── Run complementary filter as fast as possible (every loop) ──
  // This keeps the gyro integration accurate regardless of send rate.
  if (mpuOk) updateFilter();

  if (now - lastSend < SEND_INTERVAL_MS) return;
  lastSend = now;

  // ── Read buttons ──
  uint8_t up     = !digitalRead(PIN_JOY_UP);
  uint8_t down   = !digitalRead(PIN_JOY_DOWN);
  uint8_t left   = !digitalRead(PIN_JOY_LEFT);
  uint8_t right  = !digitalRead(PIN_JOY_RIGHT);
  uint8_t scaleU = !digitalRead(PIN_BTN_SCALE_UP);
  uint8_t scaleD = !digitalRead(PIN_BTN_SCALE_DN);
  uint8_t deadmn = !digitalRead(PIN_BTN_DEADMAN);

  // ── SCALE buttons adjust rate limit (not filter alpha) ──
  // Debounced: only act once per send interval (50ms).
  static uint8_t prevScaleU = 0, prevScaleD = 0;
  if (scaleU && !prevScaleU) {
    rateLimit = constrain(rateLimit + RATE_LIMIT_STEP, RATE_LIMIT_MIN, RATE_LIMIT_MAX);
    Serial.printf("[SCALE] rateLimit → %.0f deg/s\n", rateLimit);
  }
  if (scaleD && !prevScaleD) {
    rateLimit = constrain(rateLimit - RATE_LIMIT_STEP, RATE_LIMIT_MIN, RATE_LIMIT_MAX);
    Serial.printf("[SCALE] rateLimit → %.0f deg/s\n", rateLimit);
  }
  prevScaleU = scaleU;
  prevScaleD = scaleD;

  // ── Apply dead zone to fused angles ──
  float pitchSend = (fabsf(fusedPitch) > DEAD_ZONE_DEG) ? fusedPitch : 0.0f;
  float rollSend  = (fabsf(fusedRoll)  > DEAD_ZONE_DEG) ? fusedRoll  : 0.0f;

  // ── Apply rate limiter to output ──
  // Max change per send interval in degrees:
  float maxDelta = rateLimit * (SEND_INTERVAL_MS / 1000.0f);
  outputPitch = constrain(pitchSend, outputPitch - maxDelta, outputPitch + maxDelta);
  outputRoll  = constrain(rollSend,  outputRoll  - maxDelta, outputRoll  + maxDelta);

  // ── Zero output when clutch not held ──
  HandheldPacket pkt;
  pkt.pitch          = deadmn ? outputPitch : 0.0f;
  pkt.roll           = deadmn ? outputRoll  : 0.0f;
  pkt.btn_up         = up;
  pkt.btn_down       = down;
  pkt.btn_left       = left;
  pkt.btn_right      = right;
  pkt.btn_scale_up   = scaleU;
  pkt.btn_scale_down = scaleD;
  pkt.btn_deadman    = deadmn;
  pkt.battery        = readBattery();
  pkt.rssi           = 0;

  esp_now_send(MASTER_MAC, (uint8_t*)&pkt, sizeof(pkt));
  updateLed(deadmn, txFails, mpuOk, pkt.battery);

  // ── Debug heartbeat every 2s ──
  static unsigned long lastDebug = 0;
  if (now - lastDebug > 2000) {
    lastDebug = now;
    Serial.printf("[TX] fP=%.1f fR=%.1f → oP=%.1f oR=%.1f clutch=%d rate=%.0f bat=%d%%\n",
      fusedPitch, fusedRoll, outputPitch, outputRoll,
      deadmn, rateLimit, pkt.battery);
  }
}

// ─── GYRO CALIBRATION ────────────────────────────────────────────────────────
void calibrateGyro() {
  Serial.println("[STPE] Calibrating Gyro bias (keep device stationary)...");
  setLed(40, 20, 0); // amber during calibration
  const int samples = 500;
  long sumX = 0, sumY = 0;
  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(0x68);
    Wire.write(0x43); // GYRO_XOUT_H
    Wire.endTransmission(false);
    if (Wire.requestFrom(0x68, 4, true) == 4) {
      int16_t rawGx = (Wire.read() << 8) | Wire.read();
      int16_t rawGy = (Wire.read() << 8) | Wire.read();
      sumX += rawGx;
      sumY += rawGy;
    }
    delay(3);
  }
  gyroBiasX = (sumX / (float)samples) / GYRO_SENSITIVITY;
  gyroBiasY = (sumY / (float)samples) / GYRO_SENSITIVITY;
  Serial.printf("[STPE] Gyro calibration complete: biasX=%.3f deg/s, biasY=%.3f deg/s\n", gyroBiasX, gyroBiasY);
}

// ─── COMPLEMENTARY FILTER UPDATE ─────────────────────────────────────────────
// Called every loop() — NOT just at send rate.
// Reads 14 bytes: accel XYZ (6) + temp (2) + gyro XYZ (6)
void updateFilter() {
  unsigned long now = micros();
  float dt = (now - lastMpuRead) / 1000000.0f;  // seconds
  lastMpuRead = now;

  // Guard: skip if dt is unreasonable (first call, or wrap)
  if (dt <= 0.0f || dt > 0.5f) return;

  // Request 14 bytes starting at ACCEL_XOUT_H (0x3B)
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  if (Wire.requestFrom(0x68, 14, true) != 14) return;  // read failed

  // Parse raw values (big-endian signed 16-bit)
  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();  // Skip temperature (2 bytes)
  int16_t rawGx = (Wire.read() << 8) | Wire.read();
  int16_t rawGy = (Wire.read() << 8) | Wire.read();
  int16_t rawGz = (Wire.read() << 8) | Wire.read();

  // Convert gyro to degrees/second with zero-rate bias subtracted
  float gx = (rawGx / GYRO_SENSITIVITY) - gyroBiasX;  // roll rate
  float gy = (rawGy / GYRO_SENSITIVITY) - gyroBiasY;  // pitch rate

  // Accel-derived angles (static tilt, valid only when stationary)
  float ax = (float)rawAx;
  float ay = (float)rawAy;
  float az = (float)rawAz;
  float accelPitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * RAD_TO_DEG;
  float accelRoll  = atan2f( ay, az)                   * RAD_TO_DEG;

  // Complementary filter:
  // - Gyro integration gives fast, accurate short-term angle
  // - Accel correction prevents long-term gyro drift
  // α = 0.95: gyro dominates during motion, accel corrects during rest
  fusedPitch = COMP_ALPHA * (fusedPitch + gy * dt) + (1.0f - COMP_ALPHA) * accelPitch;
  fusedRoll  = COMP_ALPHA * (fusedRoll  + gx * dt) + (1.0f - COMP_ALPHA) * accelRoll;
}

// ─── BATTERY ADC ─────────────────────────────────────────────────────────────
uint8_t readBattery() {
  uint32_t raw = 0;
  for (int i = 0; i < 8; i++) raw += analogRead(PIN_VBAT);
  raw >>= 3;
  float voltage = (raw / 4095.0f) * 3.3f * VBAT_RATIO;
  int pct = (int)(((voltage - VBAT_MIN) / (VBAT_MAX - VBAT_MIN)) * 100.0f);
  return (uint8_t)constrain(pct, 0, 100);
}

// ─── LED STATUS ──────────────────────────────────────────────────────────────
void setLed(uint8_t r, uint8_t g, uint8_t b) {
  led.setPixelColor(0, led.Color(r, g, b));
  led.show();
}

void updateLed(bool clutch, uint8_t fails, bool mpuOkLocal, uint8_t bat) {
  if (!mpuOkLocal)  { setLed(60, 30, 0); return; }  // orange = MPU error
  if (fails > 3)    { setLed(80, 0,  0); return; }  // red    = TX failing
  if (clutch)       { setLed(0,  80, 0); return; }  // green  = ACTIVE
  if (bat < 20)     { setLed(40, 15, 0); return; }  // dim amber = low battery warning
                      setLed(0,  0,  25);           // blue   = idle
}
