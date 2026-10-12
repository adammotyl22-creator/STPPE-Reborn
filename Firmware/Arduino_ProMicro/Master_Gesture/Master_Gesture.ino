/*
 * POSITION CONTROL MASTER GESTURE CONTROLLER
 * Platform: Arduino Pro Micro (ATmega32u4)
 * Sensors:  MPU6050 (I2C) + Digital Joystick
 * Output:   USB Serial Packet: <p1,p2,p3,p4,px,py>
 *
 * CONTROL LOGIC (POSITION / ABSOLUTE):
 * - Sends ABSOLUTE target positions
 * - Slave (Teensy) receives "<45.2,30.5,60.0,15.0,100,50>"
 * - Teensy moves each axis to match target position
 *
 * MAPPING:
 * - Tilt Fwd/Back (Pitch) -> Axis 2 (Shoulder) Position
 * - Tilt Left/Right (Roll) -> Axis 1 (Base) Position
 *
 * CLUTCH BUTTON (Pin 9):
 * - Press to engage/disengage control
 * - When disengaged: arm holds position, you can reposition controller
 * - When engaged: arm mirrors controller position
 *
 * JOYSTICK MODES (Click to Toggle):
 * [Mode 0 - LED OFF]: Axis 3 (Elbow) + Axis 4 (Wrist)
 * [Mode 1 - LED ON]:  Micro X/Y Gantry
 *
 * FEATURES:
 * - Adjustable scaling (movement reduction for precision)
 * - Complementary filter (smooth, drift-free angles)
 * - Auto-calibration on startup
 * - Clutch for repositioning
 */

#include <Arduino.h>
#include <Wire.h>

// ================= PIN DEFINITIONS =================
const int PIN_JOY_UP = 4;
const int PIN_JOY_DOWN = 5;
const int PIN_JOY_LEFT = 6;
const int PIN_JOY_RIGHT = 7;
const int PIN_JOY_CLICK = 8;
const int PIN_CLUTCH = 9;
const int LED_PIN = 17;

// ================= MPU6050 SETTINGS =================
const int MPU_ADDR = 0x68;

// Raw sensor data
int16_t acX, acY, acZ;
int16_t gyX, gyY, gyZ;

// Filtered angles
float pitch = 0, roll = 0;

// Gyro calibration offsets
float gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0;

// Timing for integration
unsigned long lastMPUUpdate = 0;

// ================= CONTROL SETTINGS =================
// SCALING: Higher = more precision, less range
// 1.0  = 1:1 (controller 45° = arm 45°)
// 5.0  = 5:1 (controller 45° = arm 9°)
// 10.0 = 10:1 (controller 90° = arm 9°)
const float POSITION_SCALE_ROLL = 5.0;  // Base axis scaling
const float POSITION_SCALE_PITCH = 5.0; // Shoulder axis scaling

const float TILT_DEADZONE = 3.0; // Ignore tiny tilts (degrees)
const float JOY_STEP_SIZE = 1.0; // Degrees per joystick press
const int JOY_MICRO_STEP = 10;   // Steps for micro manipulator

// Complementary filter
const float GYRO_WEIGHT = 0.98;
const float ACCEL_WEIGHT = 0.02;

// Low-pass filter for smooth output
const float SMOOTHING = 0.6;
float smoothedRoll = 0, smoothedPitch = 0;

// ================= STATE =================
int mode = 0;               // 0=Main Arm, 1=Micro
bool clutchEngaged = false; // Start with clutch disengaged

// Target positions for each axis
float targetPos1 = 0; // Base (roll)
float targetPos2 = 0; // Shoulder (pitch)
float targetPos3 = 0; // Elbow (joystick)
float targetPos4 = 0; // Wrist (joystick)
long targetPosX = 0;  // Micro X
long targetPosY = 0;  // Micro Y

// Zero reference (where controller was when clutch first engaged)
float rollZero = 0;
float pitchZero = 0;

unsigned long lastUpdate = 0;
const int UPDATE_RATE = 50; // ms (20Hz)

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  initMPU6050();

  Serial.println("=== POSITION CONTROL MASTER ===");
  Serial.println("Calibrating... Keep still!");

  calibrateGyro();

  Serial.println("Calibration complete!");
  Serial.println("Press clutch button to engage control");

  // Init Pins
  pinMode(PIN_JOY_UP, INPUT_PULLUP);
  pinMode(PIN_JOY_DOWN, INPUT_PULLUP);
  pinMode(PIN_JOY_LEFT, INPUT_PULLUP);
  pinMode(PIN_JOY_RIGHT, INPUT_PULLUP);
  pinMode(PIN_JOY_CLICK, INPUT_PULLUP);
  pinMode(PIN_CLUTCH, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(LED_PIN, LOW);
  lastMPUUpdate = millis();
}

// ================= LOOP =================
void loop() {
  if (millis() - lastUpdate < UPDATE_RATE)
    return;
  lastUpdate = millis();

  // --- HANDLE CLUTCH BUTTON ---
  handleClutch();

  // --- READ SENSORS ---
  readMPUFused();

  // --- UPDATE TARGET POSITIONS ---
  if (clutchEngaged) {
    // Calculate position relative to zero reference
    float relativeRoll = roll - rollZero;
    float relativePitch = pitch - pitchZero;

    // Apply deadzone
    relativeRoll = applyDeadzone(relativeRoll, TILT_DEADZONE);
    relativePitch = applyDeadzone(relativePitch, TILT_DEADZONE);

    // Smooth the output
    smoothedRoll = smoothedRoll * SMOOTHING + relativeRoll * (1.0 - SMOOTHING);
    smoothedPitch =
        smoothedPitch * SMOOTHING + relativePitch * (1.0 - SMOOTHING);

    // Scale and set target positions
    targetPos1 = smoothedRoll / POSITION_SCALE_ROLL;
    targetPos2 = smoothedPitch / POSITION_SCALE_PITCH;
  }

  // --- READ JOYSTICK ---
  readJoystick();

  // --- SEND POSITION PACKET ---
  sendPositionPacket();
}

// ================= CLUTCH HANDLING =================
void handleClutch() {
  static bool lastClutchState = false;
  bool clutchPressed = !digitalRead(PIN_CLUTCH);

  // Detect button press (rising edge)
  if (clutchPressed && !lastClutchState) {
    clutchEngaged = !clutchEngaged;

    if (clutchEngaged) {
      // Just engaged - set current angles as zero reference
      rollZero = roll;
      pitchZero = pitch;
      smoothedRoll = 0;
      smoothedPitch = 0;
      Serial.println("CLUTCH ENGAGED - Control active");
    } else {
      Serial.println("CLUTCH RELEASED - Arm holds position");
    }

    // Visual feedback
    for (int i = 0; i < 3; i++) {
      digitalWrite(LED_PIN, HIGH);
      delay(50);
      digitalWrite(LED_PIN, LOW);
      delay(50);
    }
    digitalWrite(LED_PIN, mode ? HIGH : LOW);
  }
  lastClutchState = clutchPressed;
}

// ================= MPU6050 INIT =================
void initMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  delay(100);
}

// ================= GYRO CALIBRATION =================
void calibrateGyro() {
  const int samples = 1000;
  long sumX = 0, sumY = 0, sumZ = 0;

  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);

    gyX = Wire.read() << 8 | Wire.read();
    gyY = Wire.read() << 8 | Wire.read();
    gyZ = Wire.read() << 8 | Wire.read();

    sumX += gyX;
    sumY += gyY;
    sumZ += gyZ;
    delay(3);
  }

  gyroXOffset = (sumX / samples) / 131.0;
  gyroYOffset = (sumY / samples) / 131.0;
  gyroZOffset = (sumZ / samples) / 131.0;
}

// ================= READ MPU WITH FUSION =================
void readMPUFused() {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastMPUUpdate) / 1000.0;
  lastMPUUpdate = currentTime;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  acX = Wire.read() << 8 | Wire.read();
  acY = Wire.read() << 8 | Wire.read();
  acZ = Wire.read() << 8 | Wire.read();
  Wire.read();
  Wire.read(); // Skip temp
  gyX = Wire.read() << 8 | Wire.read();
  gyY = Wire.read() << 8 | Wire.read();
  gyZ = Wire.read() << 8 | Wire.read();

  float gyroXRate = (gyX / 131.0) - gyroXOffset;
  float gyroYRate = (gyY / 131.0) - gyroYOffset;

  float accelRoll = atan2(acY, acZ) * RAD_TO_DEG;
  float accelPitch =
      atan2(-acX, sqrt((long)acY * acY + (long)acZ * acZ)) * RAD_TO_DEG;

  roll = GYRO_WEIGHT * (roll + gyroXRate * dt) + ACCEL_WEIGHT * accelRoll;
  pitch = GYRO_WEIGHT * (pitch + gyroYRate * dt) + ACCEL_WEIGHT * accelPitch;
}

// ================= SMOOTH DEADZONE =================
float applyDeadzone(float value, float deadzone) {
  if (abs(value) <= deadzone) {
    return 0;
  }
  float sign = (value > 0) ? 1.0 : -1.0;
  return sign * (abs(value) - deadzone);
}

// ================= READ JOYSTICK =================
void readJoystick() {
  bool up = !digitalRead(PIN_JOY_UP);
  bool down = !digitalRead(PIN_JOY_DOWN);
  bool left = !digitalRead(PIN_JOY_LEFT);
  bool right = !digitalRead(PIN_JOY_RIGHT);
  bool click = !digitalRead(PIN_JOY_CLICK);

  // Mode switching
  static bool lastClick = false;
  static unsigned long lastClickTime = 0;

  if (click && !lastClick && (millis() - lastClickTime > 200)) {
    mode = !mode;
    digitalWrite(LED_PIN, mode ? HIGH : LOW);
    lastClickTime = millis();
  }
  lastClick = click;

  // Process movement
  if (mode == 0) { // === MODE 0: ELBOW & WRIST ===
    if (up)
      targetPos3 += JOY_STEP_SIZE;
    if (down)
      targetPos3 -= JOY_STEP_SIZE;
    if (left)
      targetPos4 -= JOY_STEP_SIZE;
    if (right)
      targetPos4 += JOY_STEP_SIZE;
  } else { // === MODE 1: MICRO MANIPULATOR ===
    if (up)
      targetPosY += JOY_MICRO_STEP;
    if (down)
      targetPosY -= JOY_MICRO_STEP;
    if (left)
      targetPosX -= JOY_MICRO_STEP;
    if (right)
      targetPosX += JOY_MICRO_STEP;
  }
}

// ================= SEND POSITION PACKET =================
void sendPositionPacket() {
  // Packet format: <p1,p2,p3,p4,px,py>
  // All values are ABSOLUTE target positions
  Serial.print("<");
  Serial.print(targetPos1, 2);
  Serial.print(",");
  Serial.print(targetPos2, 2);
  Serial.print(",");
  Serial.print(targetPos3, 2);
  Serial.print(",");
  Serial.print(targetPos4, 2);
  Serial.print(",");
  Serial.print(targetPosX);
  Serial.print(",");
  Serial.print(targetPosY);
  Serial.println(">");
}