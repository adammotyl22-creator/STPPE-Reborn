/*
 * IK Control - Inverse Kinematics + Full Hardware + USB Host
 * - Robot: stppe_zlozenie_nowe (6-DOF + Gripper)
 * - 6x Main Arm Steppers J1-J6 (TMC2209)
 * - 1x Gripper Servo (Pin 21)
 * - 3x AS5600 Encoders
 * - USB Host (Master Gesture Controller)
 * - Non-Blocking Homing State Machine
 * - Analytical IK Solver
 * - Cartesian Linear Path (MOVL)
 * - Teach & Playback Mode
 * - JSON Telemetry Stream
 * - Tool / Work Frame Offsets
 * - Watchdog Timer
 *
 * ROBOT DIMENSIONS (URDF stppe_zlozenie_nowe):
 * - Base Height (BASE_H): 143.896 mm
 * - Link 1 (Shoulder):    185.845 mm
 * - Link 2 (Forearm):     225.478 mm
 * - Link 3 (Wrist J5-J6):  36.105 mm
 * - Tool Total Length:     31.530 mm
 */

#include <AccelStepper.h>
#include <Arduino.h>
#include <EEPROM.h>
#include <IntervalTimer.h>
#include <Servo.h>
#include <TMCStepper.h>
#include <USBHost_t36.h>
#include <Watchdog_t4.h>
#include <Wire.h>
#include <math.h>

struct SplinePoint { float j1, j2, j3, j4, j5, j6; };
struct Waypoint;
enum class WristMode : uint8_t {
  LEVEL     = 0,   // tool poziomo (pick & place z boku)
  DOWN_0    = 1,   // tool pionowo w dół, J5=90°  (pick z góry, prosto)
  DOWN_90   = 2,   // tool pionowo w dół, J5=180° (pick z góry, obrócony)
  PITCH_0   = 3,   // pitch=0°,   J5 z parametru
  PITCH_90  = 4,   // pitch=90°,  J5 z parametru
  PITCH_180 = 5,   // pitch=180°, J5 z parametru
  MANUAL    = 6    // J4 i J5 podane ręcznie
};
struct CubicCoeffs { float c0, c1, c2, c3; };
struct MotionTrajectory {
  CubicCoeffs coeffJ1;
  CubicCoeffs coeffJ2;
  CubicCoeffs coeffJ3;
  CubicCoeffs coeffJ4;
  CubicCoeffs coeffJ5;
  CubicCoeffs coeffJ6;
  float durationUs;
  float duration_s;
};
// REFACTOR: const& zamiast kopii przez wartość — eliminuje kopiowanie ~40B na stos przy każdym wywołaniu
SplinePoint calcTangent(const SplinePoint& prev, const SplinePoint& next);
bool solveCubicHermite(float p0, float p1, float v0, float v1, float duration_s, CubicCoeffs &coeff);
void evaluateCubic(const CubicCoeffs &coeff, float t, float &pos, float &vel, float &acc);
void buildActiveCubicCoeffs(const SplinePoint &p0, const SplinePoint &p1, const SplinePoint &v0, const SplinePoint &v1, float duration_s);
void blendActiveSpline(const Waypoint &wpTarget);
void applyHoldCurrent();
void applyRunCurrent();
void applySpeedOverride();

const char *FIRMWARE_VERSION = "v3.2.0";
const char *BUILD_DATE = __DATE__;
const char *BUILD_TIME = __TIME__;

// ─── Watchdog ────────────────────────────────────────────────────────────────
WDT_T4<WDT1> watchdog;
IntervalTimer motionTimer;
IntervalTimer stepTimer;

volatile bool isHomed = false;
volatile bool motionLoopActive = false;
volatile float motionTargetPos1 = 0.0f;
volatile float motionTargetPos2 = 0.0f;
volatile float motionTargetPos3 = 0.0f;
volatile float motionTargetPos4 = 0.0f;
volatile float motionTargetPos5 = 0.0f;
volatile float motionTargetPos6 = 0.0f;
volatile float motionTargetSpeed1 = 0.0f;
volatile float motionTargetSpeed2 = 0.0f;
volatile float motionTargetSpeed3 = 0.0f;
volatile float motionTargetSpeed4 = 0.0f;
volatile float motionTargetSpeed5 = 0.0f;
volatile float motionTargetSpeed6 = 0.0f;
#define motionTargetServo4 motionTargetPos4
#define motionTargetServo5 motionTargetPos5
#define motionTargetServo6 motionTargetPos6
volatile float motionLoopDtUs = 0.0f;
volatile float cachedEncAngle1 = 0.0f;
volatile float cachedEncAngle2 = 0.0f;
volatile float cachedEncAngle3 = 0.0f;
volatile bool cachedEncoderValid = false;

void motionLoopISR();
void stepLoopISR();
void startSegmentFromQueue();
void refreshEncoderCache();

// ================= USB HOST SETUP =================
USBHost myusb;
USBHub hub1(myusb);
USBSerial userial(myusb); // Serial from handheld controller

// ================= PIN DEFINITIONS =================
// Axis 1 (Main Base)
const int PIN_AXIS1_STEP = 3;
const int PIN_AXIS1_DIR = 2;
const int PIN_AXIS1_EN = 6;

// Axis 2 (Shoulder)
const int PIN_AXIS2_STEP = 11;
const int PIN_AXIS2_DIR = 10;
const int PIN_AXIS2_EN = 27;

// Axis 3 (Elbow)
const int PIN_AXIS3_STEP = 29;
const int PIN_AXIS3_DIR = 28;
const int PIN_AXIS3_EN = 32;

// Axis 4, 5, 6 Stepper Drivers (TMC2209)
const int PIN_AXIS4_STEP = 34;
const int PIN_AXIS4_DIR = 33;
const int PIN_AXIS5_STEP = 31;
const int PIN_AXIS5_DIR = 30;
const int PIN_AXIS6_STEP = 23;
const int PIN_AXIS6_DIR = 22;
const int PIN_WRIST_EN = 12;

// Compatibility aliases for legacy pin symbols
const int PIN_MICRO_X_STEP = 34;
const int PIN_MICRO_X_DIR = 33;
const int PIN_MICRO_Y_STEP = 31;
const int PIN_MICRO_Y_DIR = 30;
const int PIN_MICRO_EN = 12;
const int PIN_MICRO_X_LIMIT = 7;
const int PIN_MICRO_Y_LIMIT = 8;

// Gripper servo (Axis 6)
const int PIN_GRIPPER_SERVO = 21;

// ── Limit switch pins (wire your physical switches here) ──────────────────
const int PIN_J1_LIMIT = 5;
const int PIN_J2_LIMIT = 4;
const int PIN_J3_LIMIT = 36;

// ── EEPROM memory map ─────────────────────────────────────────────────────
#define EEPROM_MAGIC_ADDR       0    // 4 bytes
#define EEPROM_GEAR1_ADDR       4    // 4 bytes float
#define EEPROM_GEAR2_ADDR       8    // 4 bytes float
#define EEPROM_JNT_EN_ADDR     12    // 3 bytes  (one per joint)
#define EEPROM_SPEED_OVR_ADDR  15    // 4 bytes float
#define EEPROM_TOOL_OFF_ADDR   19    // 12 bytes (3x float)
#define EEPROM_WORK_OFF_ADDR   31    // 12 bytes (3x float)
#define EEPROM_MAXSPD_ADDR     43    // 12 bytes (3x float) — MAX_SPEED per axis
#define EEPROM_MAXACC_ADDR     55    // 12 bytes (3x float) — MAX_ACCEL per axis
#define EEPROM_SAFETY_ADDR     67    // 4 bytes float — spline safety margin
#define EEPROM_TEACH_COUNT_ADDR 71   // 1 byte uint8_t
#define EEPROM_TEACH_DATA_ADDR  72   // MAX_TEACH_POINTS * sizeof(TeachPoint) (max 1536 bytes)
#define EEPROM_BACKLASH_ADDR  2048   // 28 bytes (magic + 6x float backlash)
#define EEPROM_MAGIC_NUMBER    0x494B3103  // "IK" + version 03 (teach points persistence)
#define BACKLASH_MAGIC         0x424C5348  // "BLSH"

// ── Backlash Compensation Configuration ───────────────────────────────────
struct BacklashConfig {
  uint32_t magic;
  float backlashDeg[6];
};
BacklashConfig backlashConfig = {
  BACKLASH_MAGIC,
  { 0.25f, 0.40f, 0.40f, 0.20f, 0.20f, 0.20f }
};
int8_t lastDirection[6] = { 0, 0, 0, 0, 0, 0 };
float backlashOffsetSteps[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

// ── EEPROM Write Coalescing (3-second debounced dirty flag) ───────────────
bool eepromDirty = false;
unsigned long eepromDirtyTime = 0;
const unsigned long EEPROM_DEBOUNCE_MS = 3000;

inline void scheduleEepromSave() {
  eepromDirty = true;
  eepromDirtyTime = millis();
}

// ── Speed override (global multiplier 0.0–1.0) ───────────────────────────
// Marked volatile as it is read by motionLoopISR() and written in loop()
volatile float speedOverride = 1.0f;   // 1.0 = 100%, 0.5 = 50%

// (SD card support removed)

// AS5600 Encoders — Hardware I2C
// ENC1 (Base)     → Wire2: SDA=25, SCL=24
// ENC2 (Shoulder) → Wire:  SDA=18, SCL=19
// ENC3 (Elbow)    → Wire1: SDA=44, SCL=45

float enc1Offset = 0;
float enc2Offset = 0;
float enc3Offset = 0;

// Cached encoder values to prevent redundant I2C reads
uint16_t lastEnc2 = 0;
uint16_t lastEnc3 = 0;

// Shared flags with ISR context: marked volatile
volatile bool useEncoders = true; // Default: Enabled
volatile bool estopActive = false; // Track emergency stop state

// UART / TMC Configuration
#define SERIAL_PORT Serial4
#define R_SENSE 0.11f

// ================= ROBOT GEOMETRY (URDF: stppe_zlozenie_nowe) =================
#define BASE_H          143.896f   // Floor to shoulder J2 axis (mm)
#define L1              185.845f   // Shoulder J2 to elbow J3 (mm)
#define L2              225.478f   // Elbow J3 to wrist J5 (mm)
#define L2_PAR          225.478f   // Compatibility alias
#define L2_PERP           0.0f     // Compatibility alias
#define L3               36.105f   // Wrist J5 to tool flange J6 (mm)
#define TOOL_TOTAL_LEN   31.530f   // Effective radial offset from J5 to TCP (mm)
#define TH1_0           (25.0097f * DEG_TO_RAD)
#define TH2_0           (140.4962f * DEG_TO_RAD)
#define REL_0           (TH2_0 - TH1_0)
#define GAMMA_0         (PI - REL_0)
#define TOOL_REST_ANGLE (4.896f * DEG_TO_RAD)
#define EXCL_R2         6400.0f    // r < 80mm exclusion zone (80² = 6400)
#define FLOOR_CLR         20.0f    // min clearance above floor (mm)

// ================= SAFETY LIMITS (URDF: stppe_zlozenie_nowe) =================
const float LIMIT_J1_MIN = -90.0f, LIMIT_J1_MAX =  90.0f;
const float LIMIT_J2_MIN = -65.0f, LIMIT_J2_MAX = 115.0f;
const float LIMIT_J3_MIN = -35.0f, LIMIT_J3_MAX = 170.0f;
const float LIMIT_J4_MIN = -90.0f, LIMIT_J4_MAX =  90.0f;
const float LIMIT_J5_MIN = -90.0f, LIMIT_J5_MAX =  90.0f;
const float LIMIT_J6_MIN = -180.0f, LIMIT_J6_MAX = 180.0f;

// Commanded Positions (Closed Loop Targets)
volatile float commandedAngle1 = 0;
volatile float commandedAngle2 = 0;
volatile float commandedAngle3 = 0;
volatile float commandedAngle4 = 0;
volatile float commandedAngle5 = 0;
volatile float commandedAngle6 = 0;

// ================= MOTION SETTINGS =================
const int STEPS_PER_REV = 200;
const int MICROSTEPS = 16;
const float STEPS_PER_MOTOR_REV = STEPS_PER_REV * MICROSTEPS; // 3200

// Main Arm Gearing
float gearboxRatioAxis1 = 8.33;
float stepsPerDegAxis1 = (STEPS_PER_MOTOR_REV * gearboxRatioAxis1) / 360.0; // ~74.05

float gearboxRatio = 20.0;
float stepsPerDegGeared = (STEPS_PER_MOTOR_REV * gearboxRatio) / 360.0; // ~177.78

// Wrist Gearing & Microsteps
float gearboxRatioAxis4 = 1.0f;
float gearboxRatioAxis5 = 1.0f;
float gearboxRatioAxis6 = 1.0f;
float stepsPerDegAxis4 = (STEPS_PER_MOTOR_REV * gearboxRatioAxis4) / 360.0f; // ~8.889
float stepsPerDegAxis5 = (STEPS_PER_MOTOR_REV * gearboxRatioAxis5) / 360.0f;
float stepsPerDegAxis6 = (STEPS_PER_MOTOR_REV * gearboxRatioAxis6) / 360.0f;

// ─── CALIBRATION: Adjust these to find your hardware's sweet spot ────────
// Maximum motor speed in steps/second per axis.
// Start LOW and increase until steps are lost, then back off 20%.
// Halved from previous values (40k/80k/60k) which caused step loss.
float MAX_SPEED_J1 = 10000.0f;   // Base   — was 40000
float MAX_SPEED_J2 = 25000.0f;   // Shoulder — was 80000 (main culprit)
float MAX_SPEED_J3 = 10000.0f;   // Elbow — was 60000
float MAX_SPEED_J4 = 10000.0f;   // Forearm Roll
float MAX_SPEED_J5 = 10000.0f;   // Wrist Pitch
float MAX_SPEED_J6 = 10000.0f;   // Tool Flange Roll

// Maximum acceleration in steps/s² per axis (prevents sudden speed jumps)
float MAX_ACCEL_J1 = 15000.0f;
float MAX_ACCEL_J2 = 25000.0f;
float MAX_ACCEL_J3 = 20000.0f;
float MAX_ACCEL_J4 = 20000.0f;
float MAX_ACCEL_J5 = 20000.0f;
float MAX_ACCEL_J6 = 20000.0f;

// Jerk limit in steps/s³. High bandwidth for clean cubic spline tracking.
float MAX_JERK_J1 = MAX_ACCEL_J1 * 20.0f;
float MAX_JERK_J2 = MAX_ACCEL_J2 * 20.0f;
float MAX_JERK_J3 = MAX_ACCEL_J3 * 20.0f;
float MAX_JERK_J4 = MAX_ACCEL_J4 * 20.0f;
float MAX_JERK_J5 = MAX_ACCEL_J5 * 20.0f;
float MAX_JERK_J6 = MAX_ACCEL_J6 * 20.0f;

// Cubic Hermite spline safety multiplier (peak velocity = 1.5 * average velocity)
float SPLINE_SAFETY_MARGIN = 1.5f;

// Rate-limiter state (used in velocity-mode update)
float lastSpdJ1 = 0.0f, lastSpdJ2 = 0.0f, lastSpdJ3 = 0.0f;
float lastSpdJ4 = 0.0f, lastSpdJ5 = 0.0f, lastSpdJ6 = 0.0f;
float lastAccJ1 = 0.0f, lastAccJ2 = 0.0f, lastAccJ3 = 0.0f;
float lastAccJ4 = 0.0f, lastAccJ5 = 0.0f, lastAccJ6 = 0.0f;

static constexpr float CAL_ENC_NO_DATA = -1.0f;

// Calibration test state
bool calTestActive = false;
volatile bool calTestDone = false;
float calTestStartEnc1 = CAL_ENC_NO_DATA;
float calTestStartEnc2 = CAL_ENC_NO_DATA;
float calTestStartEnc3 = CAL_ENC_NO_DATA;

// J3 (elbow) is mechanically inverted versus IK positive angle convention.
const float J3_STEP_SIGN = -1.0f;
inline long j3AngleToSteps(float deg) { return (long)roundf(deg * stepsPerDegGeared * J3_STEP_SIGN); }
inline float j3StepsToAngle(long steps) { return (float)steps / (stepsPerDegGeared * J3_STEP_SIGN); }

inline long j4AngleToSteps(float deg) { return (long)roundf(deg * stepsPerDegAxis4); }
inline float j4StepsToAngle(long steps) { return (float)steps / stepsPerDegAxis4; }
inline long j5AngleToSteps(float deg) { return (long)roundf(deg * stepsPerDegAxis5); }
inline float j5StepsToAngle(long steps) { return (float)steps / stepsPerDegAxis5; }
inline long j6AngleToSteps(float deg) { return (long)roundf(deg * stepsPerDegAxis6); }
inline float j6StepsToAngle(long steps) { return (float)steps / stepsPerDegAxis6; }

float getStepsPerDeg(uint8_t axisIdx) {
  switch (axisIdx) {
    case 0: return stepsPerDegAxis1;
    case 1: return stepsPerDegGeared;
    case 2: return stepsPerDegGeared * fabsf(J3_STEP_SIGN);
    case 3: return stepsPerDegAxis4;
    case 4: return stepsPerDegAxis5;
    case 5: return stepsPerDegAxis6;
    default: return 1.0f;
  }
}

float applyBacklashComp(uint8_t axisIdx, float deltaDeg) {
  if (axisIdx >= 6) return 0.0f;
  if (fabsf(deltaDeg) > 0.005f) {
    int8_t dir = (deltaDeg > 0.0f) ? 1 : -1;
    if (dir != lastDirection[axisIdx]) {
      float blSteps = backlashConfig.backlashDeg[axisIdx] * getStepsPerDeg(axisIdx);
      if (dir > 0) {
        backlashOffsetSteps[axisIdx] = 0.0f;
      } else {
        backlashOffsetSteps[axisIdx] = (axisIdx == 2 && J3_STEP_SIGN < 0) ? blSteps : -blSteps;
      }
      lastDirection[axisIdx] = dir;
    }
  }
  return backlashOffsetSteps[axisIdx];
}

// ================= OBJECTS =================
TMC2209Stepper driver1(&SERIAL_PORT, R_SENSE, 0b00); // Address 0
TMC2209Stepper driver2(&SERIAL_PORT, R_SENSE, 0b10); // Address 2
TMC2209Stepper driver3(&SERIAL_PORT, R_SENSE, 0b01); // Address 1

AccelStepper stepper1(AccelStepper::DRIVER, PIN_AXIS1_STEP, PIN_AXIS1_DIR);
AccelStepper stepper2(AccelStepper::DRIVER, PIN_AXIS2_STEP, PIN_AXIS2_DIR);
AccelStepper stepper3(AccelStepper::DRIVER, PIN_AXIS3_STEP, PIN_AXIS3_DIR);
AccelStepper stepper4(AccelStepper::DRIVER, PIN_AXIS4_STEP, PIN_AXIS4_DIR);
AccelStepper stepper5(AccelStepper::DRIVER, PIN_AXIS5_STEP, PIN_AXIS5_DIR);
AccelStepper stepper6(AccelStepper::DRIVER, PIN_AXIS6_STEP, PIN_AXIS6_DIR);

#define stepperX stepper4
#define stepperY stepper5

Servo gripper;
float gripperAngle = 90.0f;

// Buffers (Fixed 128 bytes)
char inputBuffer[128];
uint8_t inputBufferPos = 0;
char usbBuffer[128];
uint8_t usbBufferPos = 0;
char espBuffer[128];
uint8_t espBufferPos = 0;

// PID Controller Structure
struct PIDController {
  float kP, kI, kD, kFF, kG;
  float integral = 0;
  float lastError = 0;
  float maxIntegral = 5.0f;  // Anti-windup limit: prevents excessive integral accumulation
  float deadband = 0.35f;    // Calibrated deadband (0.35°) prevents sensor noise jitter and motor hunting

  // Initialize with specific gains
  void init(float p, float i, float d, float ff, float g = 0.0f) {
    kP = p; kI = i; kD = d; kFF = ff; kG = g;
    integral = 0;
    lastError = 0;
  }

  float calculate(float error, float dt, float targetVelocity) {
    // Deadband: within 0.35°, zero adjustment to avoid hunting around target
    if (fabsf(error) < deadband) {
      integral = 0;
      return 0.0f;
    }

    integral += error * dt;
    integral = constrain(integral, -maxIntegral, maxIntegral);

    float derivative = (dt > 0.0f) ? (error - lastError) / dt : 0.0f;
    lastError = error;

    float output = (kP * error) + (kI * integral) + (kD * derivative);
    return constrain(output, -1.2f, 1.2f); // Clamp max adjustment to ±1.2° to prevent twitching
  }
};

PIDController pid1, pid2, pid3;

// Closed Loop Adjustments (written by ISR in updateClosedLoop, read in loop)
volatile float pidAdj1 = 0.0f;
volatile float pidAdj2 = 0.0f;
volatile float pidAdj3 = 0.0f;

// Timing Constants
const unsigned long MAX_INPUT_TIME_US = 15000;
const unsigned long MAX_LOOP_TIME_US = 10000;
const uint8_t MAX_ENCODER_FAILURES = 10;

// State Flags
bool manualCL = false; // Set by CL_ON: suppress auto-disable on encoder errors
bool encEnabled[3] = {false, true, true};
bool jointEnabled[6] = {true, true, true, true, true, true};
bool useTelemetry = false;    // 10Hz position stream (AR4 format)
bool useJsonTelemetry = false; // 10Hz JSON stream for dashboard

// ── Collision detection ──────────────────────────────────────────────────
// Calibrated to 1.8°: provides ample margin above AS5600 12-bit encoder resolution (0.088°/LSB)
// and 20:1 planetary/cycloidal gearbox backlash (0.3°–0.8° on direction reversal),
// while reliably catching genuine stalls or physical obstructions without false trips.
#define COLLISION_THRESH_DEG  1.8f
int  J1collisionTrue = 0;
int  J2collisionTrue = 0;
int  J3collisionTrue = 0;

// ── Hold current state ───────────────────────────────────────────────────
bool holdCurrentActive = false;

// ── Waypoint queue ───────────────────────────────────────────────────────
struct Waypoint {
  float j1, j2, j3, j4, j5, j6;
  unsigned long duration_ms;
};

const uint8_t WAYPOINT_QUEUE_SIZE = 32;  // supports a 16+ waypoint look-ahead buffer
const uint8_t LOOKAHEAD_WAYPOINTS = 16;
Waypoint motionQueue[WAYPOINT_QUEUE_SIZE];
volatile uint8_t qHead = 0;
volatile uint8_t qTail = 0;
volatile bool isMovingQueue = false;

// ── Spline Interpolation State ───────────────────────────────────────────
volatile float splineT = 0.0f;
volatile unsigned long lastSplineTime = 0; // stored in microseconds!
volatile float splineDuration = 0.0f;      // stored in microseconds!
SplinePoint sp1, sp2, sv1, sv2, sa1, sa2;
SplinePoint spPrev;
volatile bool wasContinuous = false;
CubicCoeffs cubicCoeffJ1, cubicCoeffJ2, cubicCoeffJ3, cubicCoeffJ4, cubicCoeffJ5, cubicCoeffJ6;
MotionTrajectory activeTrajectory;
volatile float currentVelJ1 = 0.0f, currentVelJ2 = 0.0f, currentVelJ3 = 0.0f, currentVelJ4 = 0.0f, currentVelJ5 = 0.0f, currentVelJ6 = 0.0f;
volatile float currentAccJ1 = 0.0f, currentAccJ2 = 0.0f, currentAccJ3 = 0.0f, currentAccJ4 = 0.0f, currentAccJ5 = 0.0f, currentAccJ6 = 0.0f;
volatile float activeSegmentDuration_s = 0.001f;

// ── Slider / Joystick Streaming Target ────────────────────────────────
// When slider commands arrive rapidly, we skip the spline queue and
// directly set a live target for AccelStepper to chase.
bool   streamMode       = false;
float  streamTargetJ1   = 0.0f;
float  streamTargetJ2   = 0.0f;
float  streamTargetJ3   = 0.0f;
float  streamTargetJ4   = 0.0f;
float  streamTargetJ5   = 0.0f;
float  streamTargetJ6   = 0.0f;
float  streamVelJ1      = 0.0f;
float  streamVelJ2      = 0.0f;
float  streamVelJ3      = 0.0f;
float  streamVelJ4      = 0.0f;
float  streamVelJ5      = 0.0f;
float  streamVelJ6      = 0.0f;
unsigned long streamLastUpdateMs = 0;
unsigned long lastStreamMicros = 0;
const unsigned long STREAM_TIMEOUT_MS = 450; // ms without update → back to hold (tolerant to 80ms wireless packets)

void stopStreamMode(bool snapToTarget = true) {
  noInterrupts();
  if (snapToTarget) {
    commandedAngle1 = streamTargetJ1;
    commandedAngle2 = streamTargetJ2;
    commandedAngle3 = streamTargetJ3;
    commandedAngle4 = streamTargetJ4;
    commandedAngle5 = streamTargetJ5;
    commandedAngle6 = streamTargetJ6;
  }

  streamMode = false;
  isMovingQueue = false;
  wasContinuous = false;
  motionLoopActive = false;
  streamVelJ1 = 0.0f;
  streamVelJ2 = 0.0f;
  streamVelJ3 = 0.0f;
  streamVelJ4 = 0.0f;
  streamVelJ5 = 0.0f;
  streamVelJ6 = 0.0f;
  lastStreamMicros = 0;
  // Clear rate limiter and state trackers so subsequent moves start cleanly from rest
  lastSpdJ1 = 0.0f;
  lastSpdJ2 = 0.0f;
  lastSpdJ3 = 0.0f;
  lastSpdJ4 = 0.0f;
  lastSpdJ5 = 0.0f;
  lastSpdJ6 = 0.0f;
  lastAccJ1 = 0.0f;
  lastAccJ2 = 0.0f;
  lastAccJ3 = 0.0f;
  lastAccJ4 = 0.0f;
  lastAccJ5 = 0.0f;
  lastAccJ6 = 0.0f;
  currentVelJ1 = 0.0f;
  currentVelJ2 = 0.0f;
  currentVelJ3 = 0.0f;
  currentVelJ4 = 0.0f;
  currentVelJ5 = 0.0f;
  currentVelJ6 = 0.0f;
  currentAccJ1 = 0.0f;
  currentAccJ2 = 0.0f;
  currentAccJ3 = 0.0f;
  currentAccJ4 = 0.0f;
  currentAccJ5 = 0.0f;
  currentAccJ6 = 0.0f;
  motionTargetSpeed1 = 0.0f;
  motionTargetSpeed2 = 0.0f;
  motionTargetSpeed3 = 0.0f;
  motionTargetSpeed4 = 0.0f;
  motionTargetSpeed5 = 0.0f;
  motionTargetSpeed6 = 0.0f;

  stepper1.setSpeed(0.0f);
  stepper2.setSpeed(0.0f);
  stepper3.setSpeed(0.0f);
  stepper4.setSpeed(0.0f);
  stepper5.setSpeed(0.0f);
  stepper6.setSpeed(0.0f);

  stepper1.moveTo((long)roundf(commandedAngle1 * stepsPerDegAxis1));
  stepper2.moveTo((long)roundf(commandedAngle2 * stepsPerDegGeared));
  stepper3.moveTo(j3AngleToSteps(commandedAngle3));
  stepper4.moveTo(j4AngleToSteps(commandedAngle4));
  stepper5.moveTo(j5AngleToSteps(commandedAngle5));
  stepper6.moveTo(j6AngleToSteps(commandedAngle6));
  interrupts();
}

void syncCommandedToStepperPosition() {
  noInterrupts();
  commandedAngle1 = (float)stepper1.currentPosition() / stepsPerDegAxis1;
  commandedAngle2 = (float)stepper2.currentPosition() / stepsPerDegGeared;
  commandedAngle3 = j3StepsToAngle(stepper3.currentPosition());
  commandedAngle4 = j4StepsToAngle(stepper4.currentPosition());
  commandedAngle5 = j5StepsToAngle(stepper5.currentPosition());
  commandedAngle6 = j6StepsToAngle(stepper6.currentPosition());

  streamTargetJ1 = commandedAngle1;
  streamTargetJ2 = commandedAngle2;
  streamTargetJ3 = commandedAngle3;
  streamTargetJ4 = commandedAngle4;
  streamTargetJ5 = commandedAngle5;
  streamTargetJ6 = commandedAngle6;

  stepper1.moveTo(stepper1.currentPosition());
  stepper2.moveTo(stepper2.currentPosition());
  stepper3.moveTo(stepper3.currentPosition());
  stepper4.moveTo(stepper4.currentPosition());
  stepper5.moveTo(stepper5.currentPosition());
  stepper6.moveTo(stepper6.currentPosition());
  interrupts();
}

static float applyJerkLimitedSpeed(float desiredSpeed,
                                   float &currentSpeed,
                                   float &currentAccel,
                                   float dt_s,
                                   float maxSpeed,
                                   float maxAccel,
                                   float maxJerk) {
  if (dt_s <= 0.0f) dt_s = 0.0001f;

  float clampedTarget = constrain(desiredSpeed, -maxSpeed, maxSpeed);
  float maxSpeedDelta = maxAccel * dt_s;
  clampedTarget = constrain(clampedTarget,
                            currentSpeed - maxSpeedDelta,
                            currentSpeed + maxSpeedDelta);

  float desiredAccel = (clampedTarget - currentSpeed) / dt_s;
  float maxAccelDelta = maxJerk * dt_s;
  float limitedAccel = constrain(desiredAccel,
                                 currentAccel - maxAccelDelta,
                                 currentAccel + maxAccelDelta);

  float smoothedSpeed = currentSpeed + limitedAccel * dt_s;
  smoothedSpeed = constrain(smoothedSpeed, -maxSpeed, maxSpeed);

  currentSpeed = smoothedSpeed;
  currentAccel = limitedAccel;
  return smoothedSpeed;
}

SplinePoint calcTangent(const SplinePoint& prev, const SplinePoint& next) {
   SplinePoint v;
   v.j1 = 0.5f * (next.j1 - prev.j1);
   v.j2 = 0.5f * (next.j2 - prev.j2);
   v.j3 = 0.5f * (next.j3 - prev.j3);
   v.j4 = 0.5f * (next.j4 - prev.j4);
   v.j5 = 0.5f * (next.j5 - prev.j5);
   v.j6 = 0.5f * (next.j6 - prev.j6);
   return v;
}

bool solveCubicHermite(float p0, float p1, float v0, float v1, float duration_s, CubicCoeffs &coeff) {
  if (duration_s < 0.001f) duration_s = 0.001f;
  float scaledV0 = v0 * duration_s;
  float scaledV1 = v1 * duration_s;
  float dp = p1 - p0;

  coeff.c0 = p0;
  coeff.c1 = scaledV0;
  coeff.c2 = 3.0f * dp - 2.0f * scaledV0 - scaledV1;
  coeff.c3 = -2.0f * dp + scaledV0 + scaledV1;
  return true;
}

void evaluateCubic(const CubicCoeffs &coeff, float t, float &pos, float &vel, float &acc) {
  float t2 = t * t;
  float t3 = t2 * t;

  pos = coeff.c0 + coeff.c1 * t + coeff.c2 * t2 + coeff.c3 * t3;
  vel = coeff.c1 + 2.0f * coeff.c2 * t + 3.0f * coeff.c3 * t2;
  acc = 2.0f * coeff.c2 + 6.0f * coeff.c3 * t;
}

void buildActiveCubicCoeffs(const SplinePoint &p0, const SplinePoint &p1, const SplinePoint &v0, const SplinePoint &v1, float duration_s) {
  solveCubicHermite(p0.j1, p1.j1, v0.j1, v1.j1, duration_s, cubicCoeffJ1);
  solveCubicHermite(p0.j2, p1.j2, v0.j2, v1.j2, duration_s, cubicCoeffJ2);
  solveCubicHermite(p0.j3, p1.j3, v0.j3, v1.j3, duration_s, cubicCoeffJ3);
  solveCubicHermite(p0.j4, p1.j4, v0.j4, v1.j4, duration_s, cubicCoeffJ4);
  solveCubicHermite(p0.j5, p1.j5, v0.j5, v1.j5, duration_s, cubicCoeffJ5);
  solveCubicHermite(p0.j6, p1.j6, v0.j6, v1.j6, duration_s, cubicCoeffJ6);
}

inline void publishActiveTrajectory(const MotionTrajectory &trajectory) {
  noInterrupts();
  activeTrajectory = trajectory;
  splineDuration = trajectory.durationUs;
  activeSegmentDuration_s = trajectory.duration_s;
  interrupts();
}

static float estimateLookAheadScale(const SplinePoint &currentPos,
                                    const Waypoint &currentTarget,
                                    const Waypoint &nextTarget) {
  float vectors[2][6] = {
    { currentTarget.j1 - currentPos.j1, currentTarget.j2 - currentPos.j2, currentTarget.j3 - currentPos.j3,
      currentTarget.j4 - currentPos.j4, currentTarget.j5 - currentPos.j5, currentTarget.j6 - currentPos.j6 },
    { nextTarget.j1 - currentTarget.j1, nextTarget.j2 - currentTarget.j2, nextTarget.j3 - currentTarget.j3,
      nextTarget.j4 - currentTarget.j4, nextTarget.j5 - currentTarget.j5, nextTarget.j6 - currentTarget.j6 }
  };

  float normA = 0.0f;
  float normB = 0.0f;
  float dot = 0.0f;
  for (int i = 0; i < 6; ++i) {
    float a = vectors[0][i];
    float b = vectors[1][i];
    normA += a * a;
    normB += b * b;
    dot += a * b;
  }

  normA = sqrtf(normA);
  normB = sqrtf(normB);
  if (normA < 0.001f || normB < 0.001f) {
    return 1.0f;
  }

  float cosine = constrain(dot / (normA * normB), -1.0f, 1.0f);
  if (cosine <= 0.0f) {
    return 0.55f; // Sharp turn (>= 90 deg)
  }
  float turnAngle = acosf(cosine);
  float turnSeverity = constrain(turnAngle / (PI / 2.0f), 0.0f, 1.0f);
  return max(0.55f, 1.0f - (0.45f * turnSeverity));
}

static float planLookAheadDurationUs(const SplinePoint &currentPos,
                                     const Waypoint &currentTarget,
                                     float requestedDurationMs) {
  float delta1 = fabsf(currentTarget.j1 - currentPos.j1) * stepsPerDegAxis1;
  float delta2 = fabsf(currentTarget.j2 - currentPos.j2) * stepsPerDegGeared;
  float delta3 = fabsf(currentTarget.j3 - currentPos.j3) * stepsPerDegGeared;
  float delta4 = fabsf(currentTarget.j4 - currentPos.j4) * stepsPerDegAxis4;
  float delta5 = fabsf(currentTarget.j5 - currentPos.j5) * stepsPerDegAxis5;
  float delta6 = fabsf(currentTarget.j6 - currentPos.j6) * stepsPerDegAxis6;

  float minTime1 = (delta1 / MAX_SPEED_J1) * SPLINE_SAFETY_MARGIN * 1000000.0f;
  float minTime2 = (delta2 / MAX_SPEED_J2) * SPLINE_SAFETY_MARGIN * 1000000.0f;
  float minTime3 = (delta3 / MAX_SPEED_J3) * SPLINE_SAFETY_MARGIN * 1000000.0f;
  float minTime4 = (delta4 / MAX_SPEED_J4) * SPLINE_SAFETY_MARGIN * 1000000.0f;
  float minTime5 = (delta5 / MAX_SPEED_J5) * SPLINE_SAFETY_MARGIN * 1000000.0f;
  float minTime6 = (delta6 / MAX_SPEED_J6) * SPLINE_SAFETY_MARGIN * 1000000.0f;

  // Acceleration duration constraint: T_accel = sqrt(6.0 * delta / MAX_ACCEL) * 1e6 us
  float tAcc1 = (MAX_ACCEL_J1 > 0.0f) ? sqrtf((6.0f * delta1) / MAX_ACCEL_J1) * 1000000.0f : 0.0f;
  float tAcc2 = (MAX_ACCEL_J2 > 0.0f) ? sqrtf((6.0f * delta2) / MAX_ACCEL_J2) * 1000000.0f : 0.0f;
  float tAcc3 = (MAX_ACCEL_J3 > 0.0f) ? sqrtf((6.0f * delta3) / MAX_ACCEL_J3) * 1000000.0f : 0.0f;
  float tAcc4 = (MAX_ACCEL_J4 > 0.0f) ? sqrtf((6.0f * delta4) / MAX_ACCEL_J4) * 1000000.0f : 0.0f;
  float tAcc5 = (MAX_ACCEL_J5 > 0.0f) ? sqrtf((6.0f * delta5) / MAX_ACCEL_J5) * 1000000.0f : 0.0f;
  float tAcc6 = (MAX_ACCEL_J6 > 0.0f) ? sqrtf((6.0f * delta6) / MAX_ACCEL_J6) * 1000000.0f : 0.0f;

  float minRequiredTime = max(minTime1, max(minTime2, max(minTime3, max(minTime4, max(minTime5, minTime6)))));
  float minAccTime = max(tAcc1, max(tAcc2, max(tAcc3, max(tAcc4, max(tAcc5, tAcc6)))));
  minRequiredTime = max(minRequiredTime, minAccTime);

  float baseDurationUs = max(requestedDurationMs * 1000.0f, minRequiredTime);
  float plannedScale = 1.0f;

  uint8_t pending = (qHead - qTail + WAYPOINT_QUEUE_SIZE) % WAYPOINT_QUEUE_SIZE;
  uint8_t lookAheadSteps = pending > 1 ? pending - 1 : 0;
  if (lookAheadSteps > LOOKAHEAD_WAYPOINTS) lookAheadSteps = LOOKAHEAD_WAYPOINTS;

  for (uint8_t offset = 1; offset <= lookAheadSteps; ++offset) {
    uint8_t futureIdx = (qTail + offset) % WAYPOINT_QUEUE_SIZE;
    if (futureIdx == qHead) break;
    float transitionScale = estimateLookAheadScale(currentPos, currentTarget, motionQueue[futureIdx]);
    if (transitionScale < plannedScale) {
      plannedScale = transitionScale;
    }
  }

  if (plannedScale < 0.999f) {
    baseDurationUs *= (1.0f / plannedScale);
  }

  return baseDurationUs;
}

void blendActiveSpline(const Waypoint &wpTarget) {
  if (!isMovingQueue) return;

  // Snapshot active trajectory state with atomic copy
  noInterrupts();
  MotionTrajectory curTraj = activeTrajectory;
  float curSplineDuration = splineDuration;
  float curSplineT = splineT;
  float curSpeedOverride = speedOverride;
  interrupts();

  float currentOverride = max(0.05f, curSpeedOverride);
  float realDuration_us = curSplineDuration / currentOverride;
  if (realDuration_us < 1000.0f) realDuration_us = 1000.0f;

  float t = constrain(curSplineT, 0.0f, 1.0f);
  float pos1, vel1, acc1;
  float pos2, vel2, acc2;
  float pos3, vel3, acc3;
  float pos4, vel4, acc4;
  float pos5, vel5, acc5;
  float pos6, vel6, acc6;

  evaluateCubic(curTraj.coeffJ1, t, pos1, vel1, acc1);
  evaluateCubic(curTraj.coeffJ2, t, pos2, vel2, acc2);
  evaluateCubic(curTraj.coeffJ3, t, pos3, vel3, acc3);
  evaluateCubic(curTraj.coeffJ4, t, pos4, vel4, acc4);
  evaluateCubic(curTraj.coeffJ5, t, pos5, vel5, acc5);
  evaluateCubic(curTraj.coeffJ6, t, pos6, vel6, acc6);

  float realDuration_s = realDuration_us / 1000000.0f;
  if (realDuration_s < 0.001f) realDuration_s = 0.001f;

  SplinePoint local_sp1 = { pos1, pos2, pos3, pos4, pos5, pos6 };
  SplinePoint local_sp2 = { wpTarget.j1, wpTarget.j2, wpTarget.j3, wpTarget.j4, wpTarget.j5, wpTarget.j6 };
  SplinePoint local_sv1 = { vel1 / realDuration_s, vel2 / realDuration_s, vel3 / realDuration_s, vel4 / realDuration_s, vel5 / realDuration_s, vel6 / realDuration_s };
  SplinePoint local_sa1 = { acc1 / (realDuration_s * realDuration_s), acc2 / (realDuration_s * realDuration_s), acc3 / (realDuration_s * realDuration_s), acc4 / (realDuration_s * realDuration_s), acc5 / (realDuration_s * realDuration_s), acc6 / (realDuration_s * realDuration_s) };

  // Destination velocity must be zero for blend move stopping at target
  SplinePoint local_sv2 = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
  SplinePoint local_sa2 = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

  float remainingUs = max(1000.0f, (1.0f - t) * realDuration_us);
  float newDuration_s = remainingUs / 1000000.0f;
  if (newDuration_s < 0.001f) newDuration_s = 0.001f;

  MotionTrajectory newTrajectory;
  solveCubicHermite(local_sp1.j1, local_sp2.j1, local_sv1.j1, local_sv2.j1, newDuration_s, newTrajectory.coeffJ1);
  solveCubicHermite(local_sp1.j2, local_sp2.j2, local_sv1.j2, local_sv2.j2, newDuration_s, newTrajectory.coeffJ2);
  solveCubicHermite(local_sp1.j3, local_sp2.j3, local_sv1.j3, local_sv2.j3, newDuration_s, newTrajectory.coeffJ3);
  solveCubicHermite(local_sp1.j4, local_sp2.j4, local_sv1.j4, local_sv2.j4, newDuration_s, newTrajectory.coeffJ4);
  solveCubicHermite(local_sp1.j5, local_sp2.j5, local_sv1.j5, local_sv2.j5, newDuration_s, newTrajectory.coeffJ5);
  solveCubicHermite(local_sp1.j6, local_sp2.j6, local_sv1.j6, local_sv2.j6, newDuration_s, newTrajectory.coeffJ6);

  newTrajectory.durationUs = remainingUs;
  newTrajectory.duration_s = newDuration_s;

  noInterrupts();
  activeTrajectory = newTrajectory;
  splineDuration = remainingUs * currentOverride; // Store base duration to prevent double-scaling in ISR
  activeSegmentDuration_s = newDuration_s;
  sp1 = local_sp1;
  sp2 = local_sp2;
  sv1 = local_sv1;
  sv2 = local_sv2;
  sa1 = local_sa1;
  sa2 = local_sa2;
  currentVelJ1 = local_sv1.j1;
  currentVelJ2 = local_sv1.j2;
  currentVelJ3 = local_sv1.j3;
  currentVelJ4 = local_sv1.j4;
  currentVelJ5 = local_sv1.j5;
  currentVelJ6 = local_sv1.j6;
  currentAccJ1 = local_sa1.j1;
  currentAccJ2 = local_sa1.j2;
  currentAccJ3 = local_sa1.j3;
  currentAccJ4 = local_sa1.j4;
  currentAccJ5 = local_sa1.j5;
  currentAccJ6 = local_sa1.j6;
  spPrev = local_sp1;
  splineT = 0.0f;
  lastSplineTime = micros();

  lastSpdJ1 = constrain(local_sv1.j1 * stepsPerDegAxis1, -MAX_SPEED_J1, MAX_SPEED_J1);
  lastSpdJ2 = constrain(local_sv1.j2 * stepsPerDegGeared, -MAX_SPEED_J2, MAX_SPEED_J2);
  lastSpdJ3 = constrain(local_sv1.j3 * stepsPerDegGeared * J3_STEP_SIGN, -MAX_SPEED_J3, MAX_SPEED_J3);
  lastSpdJ4 = constrain(local_sv1.j4 * stepsPerDegAxis4, -MAX_SPEED_J4, MAX_SPEED_J4);
  lastSpdJ5 = constrain(local_sv1.j5 * stepsPerDegAxis5, -MAX_SPEED_J5, MAX_SPEED_J5);
  lastSpdJ6 = constrain(local_sv1.j6 * stepsPerDegAxis6, -MAX_SPEED_J6, MAX_SPEED_J6);

  motionQueue[qTail] = wpTarget;
  applyRunCurrent();
  interrupts();
}

// ── Tool & Work Frame Offsets ────────────────────────────────────────────
struct OffsetFrame { float x = 0, y = 0, z = 0; };
OffsetFrame toolOffset;  // tool-tip offset from J5 centre (mm)
OffsetFrame workOffset;  // work-object origin shift (mm)

// ── Teach Mode ───────────────────────────────────────────────────────────
struct TeachPoint { float j1, j2, j3, j4, j5, j6, grip; };
const uint8_t MAX_TEACH_POINTS = 64;
TeachPoint teachPoints[MAX_TEACH_POINTS];
uint8_t teachCount   = 0;
uint8_t teachPlayIdx = 0;
bool isTeachMode     = false;  // arm in teach-recording mode
bool isTeachPlaying  = false;

// ── EEPROM Static Layout Verification ─────────────────────────────────────
static_assert(EEPROM_TOOL_OFF_ADDR + sizeof(OffsetFrame) <= EEPROM_WORK_OFF_ADDR, "EEPROM: Tool and Work offset regions overlap!");
static_assert(EEPROM_WORK_OFF_ADDR + sizeof(OffsetFrame) <= EEPROM_MAXSPD_ADDR, "EEPROM: Work offset and Max Speed regions overlap!");
static_assert(EEPROM_TEACH_DATA_ADDR + (MAX_TEACH_POINTS * sizeof(TeachPoint)) <= 2048, "EEPROM: Teach data exceeds allocated space!");

// ── Non-Blocking Homing State Machine ───────────────────────────────────
enum HomingState {
  HOMING_IDLE,
  HOMING_J1_FAST, HOMING_J1_BACKOFF, HOMING_J1_SLOW,
  HOMING_J2_FAST, HOMING_J2_BACKOFF, HOMING_J2_SLOW,
  HOMING_J3_FAST, HOMING_J3_BACKOFF, HOMING_J3_SLOW,
  HOMING_COMPLETE, HOMING_FAULT
};
HomingState homingState  = HOMING_IDLE;
bool homingDoJ1 = false, homingDoJ2 = false, homingDoJ3 = false;
uint8_t homingRetries    = 0;
unsigned long homingStepCount = 0;
const uint8_t MAX_HOME_RETRIES_NB = 3;

// Function Prototypes
void calculateIK(float x, float y, float z, unsigned long duration = 1000,
                 WristMode wristMode = WristMode::LEVEL,
                 float j4_manual_deg = 0.0f, float j5_deg = 0.0f, bool isMovL = false, float j6_deg = 0.0f);
void calculateIK6DOF(float x, float y, float z, float roll_deg, float pitch_deg, float yaw_deg,
                     unsigned long duration = 1000, bool isMovL = false);
void moveToAngles(float theta1, float theta2, float theta3,
                  unsigned long duration_ms, bool skipWorkspaceCheck = false,
                  float t4 = -999.0f, float t5 = -999.0f, bool isMovL = false, float t6 = -999.0f);
void movL(float x2, float y2, float z2, unsigned long duration = 2000,
          WristMode wristMode = WristMode::LEVEL, float j4 = 0.0f, float j5 = 0.0f, float j6 = 0.0f);
void movL6DOF(float x2, float y2, float z2, float r2, float p2, float yw2, unsigned long duration = 2000);
void parseCommand(const char *command, float value);
void parsePacket(const char *data);
void processInput(Stream &stream, char *buffer, uint8_t &pos, size_t bufSize);
void printDriver(const char *name, TMC2209Stepper &driver);
uint16_t readAS5600(TwoWire &wire);
uint16_t readEncoderRetry(TwoWire &wire, uint8_t maxAttempts = 3);
void updateClosedLoop();
float getEncoderAngle(uint16_t raw, float offset);
void queueCalibrationWaypoint(float t1, float t2, float t3, unsigned long duration_ms);
void checkCalTestResults();
void printBacklash();
void parseBacklashCommand(const char *args);
void loadBacklashFromEEPROM();
void saveBacklashToEEPROM();

// ── EEPROM helpers ────────────────────────────────────────────────────────
bool eeprom_valid();
void eeprom_save_all();
void eeprom_load_all();

// ── Forward Kinematics ───────────────────────────────────────────────────
void forwardKinematics(float j1_deg, float j2_deg, float j3_deg,
                       float &x, float &y, float &z, bool raw = false,
                       float j4_deg = 0.0f, float j5_deg = 0.0f, float j6_deg = 0.0f);
void forwardKinematics6DOF(float j1_deg, float j2_deg, float j3_deg, float j4_deg, float j5_deg, float j6_deg,
                          float &x, float &y, float &z, float &roll_deg, float &pitch_deg, float &yaw_deg, bool raw = false);

// ── Workspace check ──────────────────────────────────────────────────────
bool checkWorkspace(float j1, float j2, float j3, float j4 = 0.0f, float j5 = 0.0f, float j6 = 0.0f);

// ── Homing ───────────────────────────────────────────────────────────────
void startHoming(bool doJ1, bool doJ2, bool doJ3);
void updateHoming();           // called every loop() — non-blocking

// ── AR4-format responses ─────────────────────────────────────────────────
void sendRobotPos();
void sendACK();
void sendError(const char *code);
void sendJSONTelemetry();      // JSON telemetry stream

// ── Encoder collision detection ──────────────────────────────────────────
void checkEncoders();

// ── iHold current management ─────────────────────────────────────────────
void applyHoldCurrent();
void applyRunCurrent();
void applySpeedOverride();

// (SD card teach/playback functions removed)

// ── Gripper ──────────────────────────────────────────────────────────────
void setGripper(float angle);

// ── Teach Mode ───────────────────────────────────────────────────────────
void updateTeachPlayback();

// ================= GRIPPER =================
void setGripper(float angle) {
  gripperAngle = constrain(angle, 0.0f, 180.0f);
  gripper.write((int)gripperAngle);
}

// ================= JSON TELEMETRY =================
void sendJSONTelemetry() {
  float cx, cy, cz;
  forwardKinematics(commandedAngle1, commandedAngle2, commandedAngle3, cx, cy, cz, false, commandedAngle4, commandedAngle5, commandedAngle6);
  char buf[350];
  const char *estop_str = estopActive ? "true" : "false";
  const char *homing_str = (homingState != HOMING_IDLE) ? "true" : "false";
  const char *teach_str = isTeachMode ? "true" : "false";
  int spd = (int)(speedOverride * 100);
  float j4_val = constrain(commandedAngle4, LIMIT_J4_MIN, LIMIT_J4_MAX);
  float j5_val = constrain(commandedAngle5, LIMIT_J5_MIN, LIMIT_J5_MAX);
  float j6_val = constrain(commandedAngle6, LIMIT_J6_MIN, LIMIT_J6_MAX);
  snprintf(buf, sizeof(buf),
    "{\"j1\":%.2f,\"j2\":%.2f,\"j3\":%.2f,"
    "\"j4\":%.1f,\"j5\":%.1f,\"j6\":%.1f,\"grip\":%d,"
    "\"x\":%.1f,\"y\":%.1f,\"z\":%.1f,"
    "\"spd\":%d,\"estop\":%s,\"homing\":%s,\"teach\":%s,\"tpts\":%d}",
    commandedAngle1, commandedAngle2, commandedAngle3,
    j4_val, j5_val, j6_val, (int)gripperAngle,
    cx, cy, cz,
    spd, estop_str, homing_str, teach_str, teachCount);
  Serial.println(buf);
}

// ================= LINEAR CARTESIAN MOVE (MOVL) =================
void movL(float x2, float y2, float z2, unsigned long duration,
          WristMode wristMode, float j4, float j5, float j6) {
  // Compute current TCP via FK
  float x1, y1, z1;
  forwardKinematics(commandedAngle1, commandedAngle2, commandedAngle3, x1, y1, z1, false, commandedAngle4, commandedAngle5, commandedAngle6);

  float dx = x2 - x1, dy = y2 - y1, dz = z2 - z1;
  float dist = sqrtf(dx*dx + dy*dy + dz*dz);
  if (dist < 1.0f) { // already there (< 1 mm), just do IK directly
    calculateIK(x2, y2, z2, duration, wristMode, j4, j5, false, j6);
    return;
  }

  // Determine number of segments (aim for ~5 mm per step, capped at 30)
  int segs = (int)(dist / 5.0f);
  segs = constrain(segs, 2, 30);
  unsigned long segDur = duration / (unsigned long)segs;
  if (segDur < 50UL) {
    segs = max(1, (int)(duration / 50UL));
    segDur = (segs > 0) ? duration / (unsigned long)segs : duration;
    segDur = max(segDur, 50UL);
  }

  Serial.print("MOVL: dist="); Serial.print(dist, 1);
  Serial.print("mm segs="); Serial.println(segs);

  float sj4 = commandedAngle4;
  float sj5 = commandedAngle5;
  float sj6 = commandedAngle6;

  for (int i = 1; i <= segs; i++) {
    float t = (float)i / segs;
    float xi = x1 + dx * t;
    float yi = y1 + dy * t;
    float zi = z1 + dz * t;
    float ji4 = sj4 + (j4 - sj4) * t;
    float ji5 = sj5 + (j5 - sj5) * t;
    float ji6 = sj6 + (j6 - sj6) * t;
    // calculateIK will enqueue into the waypoint queue
    calculateIK(xi, yi, zi, segDur, wristMode, ji4, ji5, true, ji6);
  }
}

// ================= LINEAR CARTESIAN 6-DOF MOVE (MOVL6DOF) =================
void movL6DOF(float x2, float y2, float z2, float r2, float p2, float yw2, unsigned long duration) {
  float x1, y1, z1, r1, p1, yw1;
  forwardKinematics6DOF(commandedAngle1, commandedAngle2, commandedAngle3, commandedAngle4, commandedAngle5, commandedAngle6,
                        x1, y1, z1, r1, p1, yw1, false);

  float dx = x2 - x1;
  float dy = y2 - y1;
  float dz = z2 - z1;
  float dist = sqrtf(dx * dx + dy * dy + dz * dz);

  float dr = fabsf(r2 - r1);
  float dp = fabsf(p2 - p1);
  float dyw = fabsf(yw2 - yw1);
  float maxRot = fmaxf(dr, fmaxf(dp, dyw));

  int segs = (int)ceilf(dist / 5.0f);
  int rotSegs = (int)ceilf(maxRot / 5.0f);
  if (rotSegs > segs) segs = rotSegs;
  segs = constrain(segs, 1, 30); // Capped to fit within WAYPOINT_QUEUE_SIZE

  unsigned long segDur = (unsigned long)(duration / segs);
  if (segDur < 50UL) {
    segDur = 50UL;
  }

  Serial.print("MOVL6DOF: dist="); Serial.print(dist, 1);
  Serial.print("mm rot="); Serial.print(maxRot, 1);
  Serial.print("deg segs="); Serial.println(segs);

  for (int i = 1; i <= segs; i++) {
    float t = (float)i / segs;
    float xi = x1 + dx * t;
    float yi = y1 + dy * t;
    float zi = z1 + dz * t;
    float ri = r1 + (r2 - r1) * t;
    float pi = p1 + (p2 - p1) * t;
    float ywi = yw1 + (yw2 - yw1) * t;
    calculateIK6DOF(xi, yi, zi, ri, pi, ywi, segDur, true);
  }
}

// ================= TEACH MODE PLAYBACK =================
void updateTeachPlayback() {
  if (!isTeachPlaying) return;
  if (isMovingQueue || qHead != qTail) return; // wait for current move to finish

  if (teachPlayIdx >= teachCount) {
    isTeachPlaying = false;
    teachPlayIdx = 0;
    Serial.println("TEACH: Playback complete");
    sendACK();
    return;
  }

  TeachPoint &tp = teachPoints[teachPlayIdx++];
  float d1 = fabsf(tp.j1 - commandedAngle1);
  float d2 = fabsf(tp.j2 - commandedAngle2);
  float d3 = fabsf(tp.j3 - commandedAngle3);
  float d4 = fabsf(tp.j4 - commandedAngle4);
  float d5 = fabsf(tp.j5 - commandedAngle5);
  float d6 = fabsf(tp.j6 - commandedAngle6);
  float maxD = max(max(max(d1, d2), max(d3, d4)), max(d5, d6));
  unsigned long dur = constrain((unsigned long)(maxD * 25.0f), 500UL, 3000UL);
  moveToAngles(tp.j1, tp.j2, tp.j3, dur, false, tp.j4, tp.j5, false, tp.j6);
  setGripper(tp.grip);
}

// ================= NON-BLOCKING HOMING =================
void startHoming(bool doJ1, bool doJ2, bool doJ3) {
  if (estopActive) {
    Serial.println("ERR: Cannot home while ESTOP active");
    return;
  }
  noInterrupts();
  homingDoJ1 = doJ1; homingDoJ2 = doJ2; homingDoJ3 = doJ3;
  homingRetries = 0;
  homingStepCount = 0;
  qHead = 0; qTail = 0; isMovingQueue = false;
  wasContinuous = false;
  motionLoopActive = false;
  currentVelJ1 = 0.0f; currentVelJ2 = 0.0f; currentVelJ3 = 0.0f;
  currentVelJ4 = 0.0f; currentVelJ5 = 0.0f; currentVelJ6 = 0.0f;
  currentAccJ1 = 0.0f; currentAccJ2 = 0.0f; currentAccJ3 = 0.0f;
  currentAccJ4 = 0.0f; currentAccJ5 = 0.0f; currentAccJ6 = 0.0f;
  lastSpdJ1 = 0.0f; lastSpdJ2 = 0.0f; lastSpdJ3 = 0.0f;
  lastSpdJ4 = 0.0f; lastSpdJ5 = 0.0f; lastSpdJ6 = 0.0f;
  lastAccJ1 = 0.0f; lastAccJ2 = 0.0f; lastAccJ3 = 0.0f;
  lastAccJ4 = 0.0f; lastAccJ5 = 0.0f; lastAccJ6 = 0.0f;
  for (int i = 0; i < 6; ++i) {
    backlashOffsetSteps[i] = 0.0f;
    lastDirection[i] = 0;
  }
  interrupts();

  // Apply homing speeds
  stepper1.setMaxSpeed(20000.0f);
  stepper2.setMaxSpeed(35000.0f);
  stepper3.setMaxSpeed(25000.0f);

  Serial.println("HOMING: Starting (non-blocking)...");

  if (doJ1) {
    homingState = HOMING_J1_FAST;
    stepper1.setSpeed(-stepsPerDegAxis1 * 20.0f);  // 20 deg/s fast
  } else if (doJ2) {
    homingState = HOMING_J2_FAST;
    stepper2.setSpeed(-stepsPerDegGeared * 20.0f);
  } else if (doJ3) {
    homingState = HOMING_J3_FAST;
    stepper3.setSpeed(-stepsPerDegGeared * 20.0f);
  }
}

// FIX B-3: Non-blocking paced homing with runSpeed() step counting
void updateHoming() {
  if (homingState == HOMING_IDLE || homingState == HOMING_COMPLETE || homingState == HOMING_FAULT)
    return;

  const long MAX_FAST_STEPS  = (long)(200.0f * stepsPerDegGeared);
  const long BACKOFF_STEPS   = (long)(5.0f   * stepsPerDegGeared);
  const float SLOW_SPD_DEG   = 2.0f;

  switch (homingState) {

    // ── J1 FAST ─────────────────────────────────────────────────────────
    case HOMING_J1_FAST:
      if (stepper1.runSpeed()) homingStepCount++;
      if (digitalRead(PIN_J1_LIMIT) == HIGH) {
        stepper1.setSpeed(stepsPerDegAxis1 * SLOW_SPD_DEG); // reverse
        homingStepCount = 0;
        homingState = HOMING_J1_BACKOFF;
        Serial.print("HOMING J1 backoff... ");
      } else if (homingStepCount > (unsigned long)(180.0f * stepsPerDegAxis1)) {
        if (++homingRetries >= MAX_HOME_RETRIES_NB) {
          homingState = HOMING_FAULT;
          Serial.println("ERR:HOME_FAULT_J1");
        } else {
          stepper1.setSpeed(stepsPerDegAxis1 * SLOW_SPD_DEG);
          homingStepCount = 0;
          homingState = HOMING_J1_BACKOFF;
        }
      }
      break;

    case HOMING_J1_BACKOFF:
      if (stepper1.runSpeed()) homingStepCount++;
      if (homingStepCount >= (unsigned long)(5.0f * stepsPerDegAxis1)) {
        stepper1.setSpeed(-stepsPerDegAxis1 * SLOW_SPD_DEG);
        homingStepCount = 0;
        homingState = HOMING_J1_SLOW;
        Serial.print("slow... ");
      }
      break;

    case HOMING_J1_SLOW:
      if (stepper1.runSpeed()) homingStepCount++;
      if (digitalRead(PIN_J1_LIMIT) == HIGH) {
        stepper1.setCurrentPosition(0);
        noInterrupts();
        commandedAngle1 = 0.0f;
        interrupts();
        if (encEnabled[0]) {
          uint16_t raw = readAS5600(Wire2);
          if (raw != 0xFFFF) enc1Offset = (raw / 4096.0f) * 360.0f - commandedAngle1;
        }
        Serial.println("OK");
        homingRetries = 0; homingStepCount = 0;
        if (homingDoJ2) {
          stepper2.setSpeed(-stepsPerDegGeared * 20.0f);
          homingState = HOMING_J2_FAST;
          Serial.print("HOMING J2 fast... ");
        } else if (homingDoJ3) {
          stepper3.setSpeed(-stepsPerDegGeared * 20.0f);
          homingState = HOMING_J3_FAST;
          Serial.print("HOMING J3 fast... ");
        } else {
          homingState = HOMING_COMPLETE;
        }
      } else if (homingStepCount > (unsigned long)(10.0f * stepsPerDegAxis1)) {
        homingState = HOMING_FAULT;
        Serial.println("ERR:HOME_FAULT_J1_SLOW");
      }
      break;

    // ── J2 FAST ─────────────────────────────────────────────────────────
    case HOMING_J2_FAST:
      if (stepper2.runSpeed()) homingStepCount++;
      if (digitalRead(PIN_J2_LIMIT) == HIGH) {
        stepper2.setSpeed(stepsPerDegGeared * SLOW_SPD_DEG);
        homingStepCount = 0;
        homingState = HOMING_J2_BACKOFF;
        Serial.print("HOMING J2 backoff... ");
      } else if (homingStepCount > (unsigned long)MAX_FAST_STEPS) {
        if (++homingRetries >= MAX_HOME_RETRIES_NB) {
          homingState = HOMING_FAULT;
          Serial.println("ERR:HOME_FAULT_J2");
        } else {
          stepper2.setSpeed(stepsPerDegGeared * SLOW_SPD_DEG);
          homingStepCount = 0;
          homingState = HOMING_J2_BACKOFF;
        }
      }
      break;

    case HOMING_J2_BACKOFF:
      if (stepper2.runSpeed()) homingStepCount++;
      if (homingStepCount >= (unsigned long)BACKOFF_STEPS) {
        stepper2.setSpeed(-stepsPerDegGeared * SLOW_SPD_DEG);
        homingStepCount = 0;
        homingState = HOMING_J2_SLOW;
        Serial.print("slow... ");
      }
      break;

    case HOMING_J2_SLOW:
      if (stepper2.runSpeed()) homingStepCount++;
      if (digitalRead(PIN_J2_LIMIT) == HIGH) {
        stepper2.setCurrentPosition((long)(LIMIT_J2_MIN * stepsPerDegGeared));
        noInterrupts();
        commandedAngle2 = LIMIT_J2_MIN;
        interrupts();
        if (encEnabled[1]) {
          uint16_t raw = readAS5600(Wire);
          if (raw != 0xFFFF) enc2Offset = (raw / 4096.0f) * 360.0f - commandedAngle2;
        }
        Serial.println("OK");
        homingRetries = 0; homingStepCount = 0;
        if (homingDoJ3) {
          stepper3.setSpeed(-stepsPerDegGeared * 20.0f);
          homingState = HOMING_J3_FAST;
          Serial.print("HOMING J3 fast... ");
        } else {
          homingState = HOMING_COMPLETE;
        }
      } else if (homingStepCount > (unsigned long)(10.0f * stepsPerDegGeared)) {
        homingState = HOMING_FAULT;
        Serial.println("ERR:HOME_FAULT_J2_SLOW");
      }
      break;

    // ── J3 FAST ─────────────────────────────────────────────────────────
    case HOMING_J3_FAST:
      if (stepper3.runSpeed()) homingStepCount++;
      if (digitalRead(PIN_J3_LIMIT) == HIGH) {
        stepper3.setSpeed(stepsPerDegGeared * SLOW_SPD_DEG);
        homingStepCount = 0;
        homingState = HOMING_J3_BACKOFF;
        Serial.print("HOMING J3 backoff... ");
      } else if (homingStepCount > (unsigned long)MAX_FAST_STEPS) {
        if (++homingRetries >= MAX_HOME_RETRIES_NB) {
          homingState = HOMING_FAULT;
          Serial.println("ERR:HOME_FAULT_J3");
        } else {
          stepper3.setSpeed(stepsPerDegGeared * SLOW_SPD_DEG);
          homingStepCount = 0;
          homingState = HOMING_J3_BACKOFF;
        }
      }
      break;

    case HOMING_J3_BACKOFF:
      if (stepper3.runSpeed()) homingStepCount++;
      if (homingStepCount >= (unsigned long)BACKOFF_STEPS) {
        stepper3.setSpeed(-stepsPerDegGeared * SLOW_SPD_DEG);
        homingStepCount = 0;
        homingState = HOMING_J3_SLOW;
        Serial.print("slow... ");
      }
      break;

    case HOMING_J3_SLOW:
      if (stepper3.runSpeed()) homingStepCount++;
      if (digitalRead(PIN_J3_LIMIT) == HIGH) {
        stepper3.setCurrentPosition(j3AngleToSteps(LIMIT_J3_MIN));
        noInterrupts();
        commandedAngle3 = LIMIT_J3_MIN;
        interrupts();
        if (encEnabled[2]) {
          uint16_t raw = readAS5600(Wire1);
          if (raw != 0xFFFF) enc3Offset = (raw / 4096.0f) * 360.0f - commandedAngle3;
        }
        Serial.println("OK");
        homingState = HOMING_COMPLETE;
      } else if (homingStepCount > (unsigned long)(10.0f * stepsPerDegGeared)) {
        homingState = HOMING_FAULT;
        Serial.println("ERR:HOME_FAULT_J3_SLOW");
      }
      break;

    case HOMING_COMPLETE:
      applySpeedOverride();
      for (int i = 0; i < 6; ++i) {
        backlashOffsetSteps[i] = 0.0f;
        lastDirection[i] = 0;
      }
      isHomed = true;
      moveToAngles(0.0f, 0.0f, 0.0f, 3000, true, 0.0f, 0.0f, false, 0.0f);
      sendACK();
      Serial.println("HOMING: Complete (Arm at Home 0,0,0,0,0,0)");
      homingState = HOMING_IDLE;
      break;

    case HOMING_FAULT:
      homingStepCount = 0;
      homingRetries = 0;
      Serial.println("HOMING: FAULT - some axes failed");
      homingState = HOMING_IDLE;
      break;

    default:
      break;
  }
}

// ================= EEPROM HELPERS =================
bool eeprom_valid() {
  uint32_t magic;
  EEPROM.get(EEPROM_MAGIC_ADDR, magic);
  return (magic == EEPROM_MAGIC_NUMBER);
}

void eeprom_save_all() {
  uint32_t magic = EEPROM_MAGIC_NUMBER;
  EEPROM.put(EEPROM_MAGIC_ADDR, magic);
  EEPROM.put(EEPROM_GEAR1_ADDR, gearboxRatioAxis1);
  EEPROM.put(EEPROM_GEAR2_ADDR, gearboxRatio);
  EEPROM.put(EEPROM_JNT_EN_ADDR,     (uint8_t)jointEnabled[0]);
  EEPROM.put(EEPROM_JNT_EN_ADDR + 1, (uint8_t)jointEnabled[1]);
  EEPROM.put(EEPROM_JNT_EN_ADDR + 2, (uint8_t)jointEnabled[2]);
  float spdOvrToSave = speedOverride;
  EEPROM.put(EEPROM_SPEED_OVR_ADDR, spdOvrToSave);
  EEPROM.put(EEPROM_TOOL_OFF_ADDR,  toolOffset);
  EEPROM.put(EEPROM_WORK_OFF_ADDR,  workOffset);
  // Calibration data
  EEPROM.put(EEPROM_MAXSPD_ADDR,      MAX_SPEED_J1);
  EEPROM.put(EEPROM_MAXSPD_ADDR + 4,  MAX_SPEED_J2);
  EEPROM.put(EEPROM_MAXSPD_ADDR + 8,  MAX_SPEED_J3);
  EEPROM.put(EEPROM_MAXACC_ADDR,      MAX_ACCEL_J1);
  EEPROM.put(EEPROM_MAXACC_ADDR + 4,  MAX_ACCEL_J2);
  EEPROM.put(EEPROM_MAXACC_ADDR + 8,  MAX_ACCEL_J3);
  EEPROM.put(EEPROM_SAFETY_ADDR,      SPLINE_SAFETY_MARGIN);
  // Teach data
  EEPROM.put(EEPROM_TEACH_COUNT_ADDR, teachCount);
  for (uint8_t i = 0; i < teachCount && i < MAX_TEACH_POINTS; ++i) {
    EEPROM.put(EEPROM_TEACH_DATA_ADDR + i * sizeof(TeachPoint), teachPoints[i]);
  }
  saveBacklashToEEPROM();
}

void eeprom_load_all() {
  if (!eeprom_valid()) return;
  EEPROM.get(EEPROM_GEAR1_ADDR, gearboxRatioAxis1);
  EEPROM.get(EEPROM_GEAR2_ADDR, gearboxRatio);
  if (isnan(gearboxRatioAxis1) || gearboxRatioAxis1 <= 0.1f || gearboxRatioAxis1 > 1000.0f) gearboxRatioAxis1 = 8.33f;
  if (isnan(gearboxRatio) || gearboxRatio <= 0.1f || gearboxRatio > 1000.0f) gearboxRatio = 20.0f;
  stepsPerDegAxis1   = (STEPS_PER_MOTOR_REV * gearboxRatioAxis1) / 360.0;
  stepsPerDegGeared  = (STEPS_PER_MOTOR_REV * gearboxRatio)      / 360.0;
  uint8_t j0, j1, j2;
  EEPROM.get(EEPROM_JNT_EN_ADDR,     j0);
  EEPROM.get(EEPROM_JNT_EN_ADDR + 1, j1);
  EEPROM.get(EEPROM_JNT_EN_ADDR + 2, j2);
  jointEnabled[0] = (bool)j0;
  jointEnabled[1] = (bool)j1;
  jointEnabled[2] = (bool)j2;
  float loadedSpdOvr = 1.0f;
  EEPROM.get(EEPROM_SPEED_OVR_ADDR, loadedSpdOvr);
  speedOverride = constrain(loadedSpdOvr, 0.05f, 1.0f);
  EEPROM.get(EEPROM_TOOL_OFF_ADDR,  toolOffset);
  EEPROM.get(EEPROM_WORK_OFF_ADDR,  workOffset);

  // Validate offsets to prevent NaN issues
  if (isnan(toolOffset.x) || isnan(toolOffset.y) || isnan(toolOffset.z))
    toolOffset = {0, 0, 0};
  if (isnan(workOffset.x) || isnan(workOffset.y) || isnan(workOffset.z))
    workOffset = {0, 0, 0};

  // Load calibration data
  float tmp;
  EEPROM.get(EEPROM_MAXSPD_ADDR, tmp);
  if (!isnan(tmp) && tmp > 0) MAX_SPEED_J1 = tmp;
  EEPROM.get(EEPROM_MAXSPD_ADDR + 4, tmp);
  if (!isnan(tmp) && tmp > 0) MAX_SPEED_J2 = tmp;
  EEPROM.get(EEPROM_MAXSPD_ADDR + 8, tmp);
  if (!isnan(tmp) && tmp > 0) MAX_SPEED_J3 = tmp;
  EEPROM.get(EEPROM_MAXACC_ADDR, tmp);
  if (!isnan(tmp) && tmp > 0) MAX_ACCEL_J1 = tmp;
  EEPROM.get(EEPROM_MAXACC_ADDR + 4, tmp);
  if (!isnan(tmp) && tmp > 0) MAX_ACCEL_J2 = tmp;
  EEPROM.get(EEPROM_MAXACC_ADDR + 8, tmp);
  if (!isnan(tmp) && tmp > 0) MAX_ACCEL_J3 = tmp;
  MAX_JERK_J1 = MAX_ACCEL_J1 * 10.0f;
  MAX_JERK_J2 = MAX_ACCEL_J2 * 10.0f;
  MAX_JERK_J3 = MAX_ACCEL_J3 * 10.0f;
  EEPROM.get(EEPROM_SAFETY_ADDR, tmp);
  if (!isnan(tmp) && tmp >= 1.0f) {
    if (tmp > 2.0f) tmp = 1.5f; // Automatically migrate legacy 2.5 safety margin to 1.5
    SPLINE_SAFETY_MARGIN = tmp;
  }

  // Load Teach data
  uint8_t loadedTeachCount = 0;
  EEPROM.get(EEPROM_TEACH_COUNT_ADDR, loadedTeachCount);
  if (loadedTeachCount <= MAX_TEACH_POINTS) {
    teachCount = loadedTeachCount;
    for (uint8_t i = 0; i < teachCount; ++i) {
      EEPROM.get(EEPROM_TEACH_DATA_ADDR + i * sizeof(TeachPoint), teachPoints[i]);
    }
  }
  loadBacklashFromEEPROM();
}

// ================= FORWARD KINEMATICS =================
void forwardKinematics(float j1_deg, float j2_deg, float j3_deg,
                       float &x, float &y, float &z, bool raw,
                       float j4_deg, float j5_deg, float j6_deg) {
    float j1r = j1_deg * DEG_TO_RAD;
    float j2r = j2_deg * DEG_TO_RAD;
    float j3r = j3_deg * DEG_TO_RAD;
    float j5r = j5_deg * DEG_TO_RAD;

    float th1 = TH1_0 - j2r;
    float th2 = TH2_0 - (j2r + j3r);

    float j3_reach = L1 * sinf(th1);
    float j3_z     = BASE_H + L1 * cosf(th1);

    float j5_reach = j3_reach + L2 * sinf(th2);
    float j5_z     = j3_z + L2 * cosf(th2);

    float ang_tool = TOOL_REST_ANGLE + (j2r + j3r + j5r);
    float tcp_reach = j5_reach + TOOL_TOTAL_LEN * cosf(ang_tool);
    float tcp_z     = j5_z + TOOL_TOTAL_LEN * sinf(ang_tool);

    x = tcp_reach * cosf(j1r);
    y = tcp_reach * sinf(j1r);
    z = tcp_z;

    if (!raw) {
        // Apply inverse offsets to report position in Work Frame
        x += workOffset.x;  y += workOffset.y;  z += workOffset.z;
        x -= toolOffset.x;  y -= toolOffset.y;  z -= toolOffset.z;
    }
}

// ================= BACKLASH EEPROM & MANAGEMENT =================
void saveBacklashToEEPROM() {
  uint32_t magic = BACKLASH_MAGIC;
  EEPROM.put(EEPROM_BACKLASH_ADDR, magic);
  for (int i = 0; i < 6; ++i) {
    EEPROM.put(EEPROM_BACKLASH_ADDR + 4 + i * sizeof(float), backlashConfig.backlashDeg[i]);
  }
}

void loadBacklashFromEEPROM() {
  uint32_t magic = 0;
  EEPROM.get(EEPROM_BACKLASH_ADDR, magic);
  if (magic == BACKLASH_MAGIC) {
    for (int i = 0; i < 6; ++i) {
      float v = 0.0f;
      EEPROM.get(EEPROM_BACKLASH_ADDR + 4 + i * sizeof(float), v);
      if (!isnan(v) && !isinf(v) && v >= 0.0f && v <= 5.0f) {
        backlashConfig.backlashDeg[i] = v;
      }
    }
  }
}

void printBacklash() {
  Serial.println("=== BACKLASH COMPENSATION ===");
  for (int i = 0; i < 6; ++i) {
    Serial.printf("  J%d: %.3f deg (offset %.1f steps)\n", i + 1, backlashConfig.backlashDeg[i], backlashOffsetSteps[i]);
  }
  Serial.println("=============================");
}

void parseBacklashCommand(const char *args) {
  int axis = 0;
  float val = 0.0f;
  if (sscanf(args, "J%d %f", &axis, &val) == 2 || sscanf(args, "%d %f", &axis, &val) == 2) {
    if (axis >= 1 && axis <= 6) {
      if (val >= 0.0f && val <= 5.0f) {
        backlashConfig.backlashDeg[axis - 1] = val;
        scheduleEepromSave();
        Serial.printf("Backlash J%d set to %.3f deg\n", axis, val);
      } else {
        Serial.println("ERR: Backlash value must be between 0.0 and 5.0 deg");
      }
    } else {
      Serial.println("ERR: Invalid axis (1-6)");
    }
  } else {
    Serial.println("ERR: Usage: BACKLASH [J<1-6> <deg>]");
  }
}

// ================= FORWARD KINEMATICS 6-DOF =================
void forwardKinematics6DOF(float j1_deg, float j2_deg, float j3_deg, float j4_deg, float j5_deg, float j6_deg,
                           float &x, float &y, float &z, float &roll_deg, float &pitch_deg, float &yaw_deg, bool raw) {
    float j1r = j1_deg * DEG_TO_RAD;
    float j2r = j2_deg * DEG_TO_RAD;
    float j3r = j3_deg * DEG_TO_RAD;
    float j4r = j4_deg * DEG_TO_RAD;
    float j5r = j5_deg * DEG_TO_RAD;
    float j6r = j6_deg * DEG_TO_RAD;

    float th1 = TH1_0 - j2r;
    float th2 = TH2_0 - (j2r + j3r);

    float j3_reach = L1 * sinf(th1);
    float j3_z     = BASE_H + L1 * cosf(th1);

    float j5_reach = j3_reach + L2 * sinf(th2);
    float j5_z     = j3_z + L2 * cosf(th2);

    float p5x = j5_reach * cosf(j1r);
    float p5y = j5_reach * sinf(j1r);
    float p5z = j5_z;

    // Pitch of tool at j4=0, j5=0 is TOOL_REST_ANGLE + (j2r + j3r) to harmonize with 3-DOF FK
    float pitch_base = TOOL_REST_ANGLE + (j2r + j3r);
    float c1 = cosf(j1r), s1 = sinf(j1r);
    float cp = cosf(pitch_base), sp = sinf(pitch_base);

    float R03[3][3] = {
      { c1 * cp, -s1, -c1 * sp },
      { s1 * cp,  c1, -s1 * sp },
      { sp,       0.0f, cp }
    };

    float c4 = cosf(j4r), s4 = sinf(j4r);
    float c5 = cosf(j5r), s5 = sinf(j5r);
    float c6 = cosf(j6r), s6 = sinf(j6r);

    float Rx4[3][3] = {
      { 1.0f,  0.0f,   0.0f },
      { 0.0f,    c4,    -s4 },
      { 0.0f,    s4,     c4 }
    };
    float Ry5[3][3] = {
      {   c5,  0.0f,    -s5 },
      { 0.0f,  1.0f,   0.0f },
      {   s5,  0.0f,     c5 }
    };
    float Rx6[3][3] = {
      { 1.0f,  0.0f,   0.0f },
      { 0.0f,    c6,    -s6 },
      { 0.0f,    s6,     c6 }
    };

    float R35[3][3];
    for(int i = 0; i < 3; ++i) {
      for(int j = 0; j < 3; ++j) {
        R35[i][j] = Rx4[i][0]*Ry5[0][j] + Rx4[i][1]*Ry5[1][j] + Rx4[i][2]*Ry5[2][j];
      }
    }
    float R36[3][3];
    for(int i = 0; i < 3; ++i) {
      for(int j = 0; j < 3; ++j) {
        R36[i][j] = R35[i][0]*Rx6[0][j] + R35[i][1]*Rx6[1][j] + R35[i][2]*Rx6[2][j];
      }
    }
    float R06[3][3];
    for(int i = 0; i < 3; ++i) {
      for(int j = 0; j < 3; ++j) {
        R06[i][j] = R03[i][0]*R36[0][j] + R03[i][1]*R36[1][j] + R03[i][2]*R36[2][j];
      }
    }

    float ux = R06[0][0];
    float uy = R06[1][0];
    float uz = R06[2][0];

    x = p5x + TOOL_TOTAL_LEN * ux;
    y = p5y + TOOL_TOTAL_LEN * uy;
    z = p5z + TOOL_TOTAL_LEN * uz;

    if (!raw) {
        x += workOffset.x;  y += workOffset.y;  z += workOffset.z;
        x -= toolOffset.x;  y -= toolOffset.y;  z -= toolOffset.z;
    }

    pitch_deg = atan2f(-R06[2][0], sqrtf(R06[0][0]*R06[0][0] + R06[1][0]*R06[1][0])) * RAD_TO_DEG;
    yaw_deg   = atan2f(R06[1][0], R06[0][0]) * RAD_TO_DEG;
    roll_deg  = atan2f(R06[2][1], R06[2][2]) * RAD_TO_DEG;
}

// ================= WORKSPACE CHECK =================
bool checkWorkspace(float j1, float j2, float j3, float j4, float j5, float j6) {
    if (j1 < LIMIT_J1_MIN || j1 > LIMIT_J1_MAX) return false;
    if (j2 < LIMIT_J2_MIN || j2 > LIMIT_J2_MAX) return false;
    if (j3 < LIMIT_J3_MIN || j3 > LIMIT_J3_MAX) return false;
    if (j4 < LIMIT_J4_MIN || j4 > LIMIT_J4_MAX) return false;
    if (j5 < LIMIT_J5_MIN || j5 > LIMIT_J5_MAX) return false;
    if (j6 < LIMIT_J6_MIN || j6 > LIMIT_J6_MAX) return false;

    float j2r = j2 * DEG_TO_RAD;
    float th1 = TH1_0 - j2r;
    float j3_z = BASE_H + L1 * cosf(th1);

    float tcp_x, tcp_y, tcp_z;
    forwardKinematics(j1, j2, j3, tcp_x, tcp_y, tcp_z, true, j4, j5, j6);

    if ((tcp_x * tcp_x + tcp_y * tcp_y) < EXCL_R2 && tcp_z < BASE_H) return false;

    if (j3_z < FLOOR_CLR) return false;
    if (tcp_z < FLOOR_CLR) return false;

    return true;
}

// ================= AR4-FORMAT RESPONSES =================
void sendRobotPos() {
  // Compute current FK position
  float cx, cy, cz;
  forwardKinematics(commandedAngle1, commandedAngle2, commandedAngle3, cx, cy, cz, false, commandedAngle4, commandedAngle5, commandedAngle6);
  float j4_val = constrain(commandedAngle4, LIMIT_J4_MIN, LIMIT_J4_MAX);
  float j5_val = constrain(commandedAngle5, LIMIT_J5_MIN, LIMIT_J5_MAX);
  float j6_val = constrain(commandedAngle6, LIMIT_J6_MIN, LIMIT_J6_MAX);

  Serial.print("A"); Serial.print(commandedAngle1, 3);
  Serial.print("B"); Serial.print(commandedAngle2, 3);
  Serial.print("C"); Serial.print(commandedAngle3, 3);
  Serial.print("D"); Serial.print(j4_val, 1);
  Serial.print("E"); Serial.print(j5_val, 1);
  Serial.print("F"); Serial.print(j6_val, 1);
  Serial.print("G"); Serial.print(cx, 2);
  Serial.print("H"); Serial.print(cy, 2);
  Serial.print("I"); Serial.println(cz, 2);
}

void sendACK() {
  Serial.println("OK");
}

void sendError(const char *code) {
  Serial.println(code);
}

// ================= ENCODER COLLISION DETECTION =================
void checkEncoders() {
  J1collisionTrue = 0;
  J2collisionTrue = 0;
  J3collisionTrue = 0;

  static uint8_t j1Violations = 0;
  static uint8_t j2Violations = 0;
  static uint8_t j3Violations = 0;

  auto getAngleError = [](float target, float actual) -> float {
    float diff = target - actual;
    diff = fmodf(diff + 180.0f, 360.0f);
    if (diff < 0.0f) diff += 360.0f;
    return fabsf(diff - 180.0f);
  };

  // J1 — Base (Wire2)
  if (encEnabled[0]) {
    uint16_t raw = readEncoderRetry(Wire2);
    if (raw != 0xFFFF) {
      float actual = getEncoderAngle(raw, enc1Offset);
      float error  = getAngleError(commandedAngle1, actual);
      if (error > COLLISION_THRESH_DEG) {
        if (++j1Violations >= 2) {
          J1collisionTrue = 1;
          Serial.print("COLLISION J1: err=");
          Serial.print(error, 1);
          Serial.println("deg");
        }
      } else {
        j1Violations = 0;
      }
    }
  }

  // J2 — Shoulder (Wire)
  if (encEnabled[1]) {
    uint16_t raw = readEncoderRetry(Wire);
    if (raw != 0xFFFF) {
      float actual = getEncoderAngle(raw, enc2Offset);
      float error  = getAngleError(commandedAngle2, actual);
      if (error > COLLISION_THRESH_DEG) {
        if (++j2Violations >= 2) {
          J2collisionTrue = 1;
          Serial.print("COLLISION J2: err=");
          Serial.print(error, 1);
          Serial.println("deg");
        }
      } else {
        j2Violations = 0;
      }
    }
  }

  // J3 — Elbow (Wire1)
  if (encEnabled[2]) {
    uint16_t raw = readEncoderRetry(Wire1);
    if (raw != 0xFFFF) {
      float actual = getEncoderAngle(raw, enc3Offset);
      float error  = getAngleError(commandedAngle3, actual);
      if (error > COLLISION_THRESH_DEG) {
        if (++j3Violations >= 2) {
          J3collisionTrue = 1;
          Serial.print("COLLISION J3: err=");
          Serial.print(error, 1);
          Serial.println("deg");
        }
      } else {
        j3Violations = 0;
      }
    }
  }

  // Build collision flag string (AR4 format: "EC000")
  int total = J1collisionTrue + J2collisionTrue + J3collisionTrue;
  if (total > 0) {
    // BUG-D FIX: stack-allocated buffer — replaces Arduino String heap alloc that fragments RAM
    char flag[16];
    snprintf(flag, sizeof(flag), "EC%d%d%d",
             J1collisionTrue, J2collisionTrue, J3collisionTrue);
    Serial.println(flag);
    if (!estopActive) {
      Serial.println("ERR: COLLISION DETECTED - Triggering ESTOP");
      parseCommand("ESTOP", 0);
    }
  }
}

// ================= IHOLD CURRENT MANAGEMENT =================
void applyHoldCurrent() {
  if (holdCurrentActive) return;
  holdCurrentActive = true;
}

void applyRunCurrent() {
  if (!holdCurrentActive) return;
  holdCurrentActive = false;
}

void applySpeedOverride() {
  // Main axes (J1-J6) AccelStepper limits set to physical limits for safe hold mode
  stepper1.setMaxSpeed(MAX_SPEED_J1);
  stepper2.setMaxSpeed(MAX_SPEED_J2);
  stepper3.setMaxSpeed(MAX_SPEED_J3);
  stepper4.setMaxSpeed(MAX_SPEED_J4);
  stepper5.setMaxSpeed(MAX_SPEED_J5);
  stepper6.setMaxSpeed(MAX_SPEED_J6);
  stepper1.setAcceleration(MAX_ACCEL_J1);
  stepper2.setAcceleration(MAX_ACCEL_J2);
  stepper3.setAcceleration(MAX_ACCEL_J3);
  stepper4.setAcceleration(MAX_ACCEL_J4);
  stepper5.setAcceleration(MAX_ACCEL_J5);
  stepper6.setAcceleration(MAX_ACCEL_J6);
  stepperX.setMaxSpeed(500.0f * speedOverride);
  stepperY.setMaxSpeed(500.0f * speedOverride);
}

// (Legacy blocking homeAxes removed in favor of startHoming/updateHoming state machine)

// ================= HOMING GANTRY =================
// Non-blocking time-paced gantry homing with watchdog and USB service
void homeGantry(bool doX, bool doY) {
  if (estopActive) {
    Serial.println("ERR: Cannot home while ESTOP is active");
    return;
  }

  Serial.println("HOMING GANTRY: Starting...");
  digitalWrite(PIN_MICRO_EN, LOW); // Enable drivers
  myusb.Task(); // Keep alive

  noInterrupts();
  wasContinuous = false;
  currentVelJ1 = 0.0f; currentVelJ2 = 0.0f; currentVelJ3 = 0.0f;
  currentAccJ1 = 0.0f; currentAccJ2 = 0.0f; currentAccJ3 = 0.0f;
  interrupts();

  // Helper lambda — drive one stepper until limit pin goes HIGH, with timed pacing and watchdog feeding
  auto driveGantryLim = [&](AccelStepper &s, int limitPin, float stepsPerSec, int maxSt) -> bool {
    s.setSpeed(stepsPerSec);
    float stepIntervalUs = (fabsf(stepsPerSec) > 0.0f) ? (1000000.0f / fabsf(stepsPerSec)) : 2000.0f;
    unsigned long timeoutUs = (unsigned long)((float)maxSt * stepIntervalUs) + 500000UL;
    unsigned long startUs = micros();
    long startPos = s.currentPosition();

    while ((micros() - startUs) < timeoutUs && labs(s.currentPosition() - startPos) < maxSt) {
      if (digitalRead(limitPin) == HIGH) return true; // switch triggered
      s.runSpeed();
      watchdog.feed();
      myusb.Task();
      yield();
    }
    return (digitalRead(limitPin) == HIGH);
  };

  auto driveGantrySteps = [&](AccelStepper &s, float stepsPerSec, long stepsToMove) {
    s.setSpeed(stepsPerSec);
    float stepIntervalUs = (fabsf(stepsPerSec) > 0.0f) ? (1000000.0f / fabsf(stepsPerSec)) : 2000.0f;
    unsigned long timeoutUs = (unsigned long)((float)stepsToMove * stepIntervalUs) + 200000UL;
    unsigned long startUs = micros();
    long startPos = s.currentPosition();
    while ((micros() - startUs) < timeoutUs && labs(s.currentPosition() - startPos) < stepsToMove) {
      s.runSpeed();
      watchdog.feed();
      myusb.Task();
      yield();
    }
  };

  const float FAST_SPD = -500.0f; // negative towards switch
  const float SLOW_SPD = -100.0f;
  const long  BOUNCE   = 200;     // relative steps away
  const int   MAX_X_STEPS = 10000;
  const int   MAX_Y_STEPS = 10000;

  if (doX) {
    Serial.print("X... ");
    stepperX.setMaxSpeed(1000);
    // Move to limit
    if (driveGantryLim(stepperX, PIN_MICRO_X_LIMIT, FAST_SPD, MAX_X_STEPS)) {
       // Back off
       driveGantrySteps(stepperX, -FAST_SPD, BOUNCE);
       // Slow hit
       driveGantryLim(stepperX, PIN_MICRO_X_LIMIT, SLOW_SPD, BOUNCE*2);
       stepperX.setCurrentPosition(0);
       Serial.println("OK");
    } else {
       Serial.println("ERR: TIMEOUT");
    }
  }

  if (doY) {
    Serial.print("Y... ");
    stepperY.setMaxSpeed(1000);
    // Move to limit
    if (driveGantryLim(stepperY, PIN_MICRO_Y_LIMIT, FAST_SPD, MAX_Y_STEPS)) {
       // Back off
       driveGantrySteps(stepperY, -FAST_SPD, BOUNCE);
       // Slow hit
       driveGantryLim(stepperY, PIN_MICRO_Y_LIMIT, SLOW_SPD, BOUNCE*2);
       stepperY.setCurrentPosition(0);
       Serial.println("OK");
    } else {
       Serial.println("ERR: TIMEOUT");
    }
  }

  // Steppers will auto-disable via the timeout in loop() !
}

// ================= SETUP =================
void startSegmentFromQueue() {
  if (qHead == qTail) {
    isMovingQueue = false;
    return;
  }
  isMovingQueue = true;
  Waypoint &wpTarget = motionQueue[qTail];

  float baseDuration = (float)wpTarget.duration_ms;
  if (baseDuration < 1.0f) baseDuration = 1.0f;

  SplinePoint currentPos = { commandedAngle1, commandedAngle2, commandedAngle3, commandedAngle4, commandedAngle5, commandedAngle6 };
  splineDuration = planLookAheadDurationUs(currentPos, wpTarget, baseDuration);

  sp1.j1 = commandedAngle1;
  sp1.j2 = commandedAngle2;
  sp1.j3 = commandedAngle3;
  sp1.j4 = commandedAngle4;
  sp1.j5 = commandedAngle5;
  sp1.j6 = commandedAngle6;

  sp2.j1 = wpTarget.j1;
  sp2.j2 = wpTarget.j2;
  sp2.j3 = wpTarget.j3;
  sp2.j4 = wpTarget.j4;
  sp2.j5 = wpTarget.j5;
  sp2.j6 = wpTarget.j6;

  // Apply software backlash compensation on segment direction reversal
  applyBacklashComp(0, sp2.j1 - sp1.j1);
  applyBacklashComp(1, sp2.j2 - sp1.j2);
  applyBacklashComp(2, sp2.j3 - sp1.j3);
  applyBacklashComp(3, sp2.j4 - sp1.j4);
  applyBacklashComp(4, sp2.j5 - sp1.j5);
  applyBacklashComp(5, sp2.j6 - sp1.j6);

  uint8_t nextIdx = (qTail + 1) % WAYPOINT_QUEUE_SIZE;
  bool hasNextWp = (nextIdx != qHead);
  Waypoint &nextWp = motionQueue[nextIdx];

  // Compute 6D corner turning angle & centripetal corner velocity scale
  float cornerScale = 1.0f;
  if (hasNextWp) {
    float d1_vec[6] = {
      sp2.j1 - sp1.j1, sp2.j2 - sp1.j2, sp2.j3 - sp1.j3,
      sp2.j4 - sp1.j4, sp2.j5 - sp1.j5, sp2.j6 - sp1.j6
    };
    float d2_vec[6] = {
      nextWp.j1 - sp2.j1, nextWp.j2 - sp2.j2, nextWp.j3 - sp2.j3,
      nextWp.j4 - sp2.j4, nextWp.j5 - sp2.j5, nextWp.j6 - sp2.j6
    };
    float dotProd = 0.0f, norm1 = 0.0f, norm2 = 0.0f;
    for (int k = 0; k < 6; ++k) {
      dotProd += d1_vec[k] * d2_vec[k];
      norm1 += d1_vec[k] * d1_vec[k];
      norm2 += d2_vec[k] * d2_vec[k];
    }
    norm1 = sqrtf(norm1);
    norm2 = sqrtf(norm2);
    if (norm1 > 0.001f && norm2 > 0.001f) {
      float cosTh = constrain(dotProd / (norm1 * norm2), -1.0f, 1.0f);
      if (cosTh <= 0.0f) {
        cornerScale = 0.0f; // Sharp turn (>= 90 deg) or reversal: stop at apex
      } else {
        float th = acosf(cosTh);
        cornerScale = cosf(th * 0.5f); // Smooth corner blending based on turning angle
      }
    }
  }

  // Fritsch-Carlson Monotone Spline Boundary Tangent & Zero-Overshoot Clamping with Corner Scale
  auto computeMonotoneTangents = [cornerScale](float p0, float p1, float p2_next, bool hasNextWp,
                                               float dt1_s, float dt2_s, float inVel,
                                               float &v0, float &v1, float &a0, float &a1) {
    float dp1 = p1 - p0;
    a0 = 0.0f;
    a1 = 0.0f;

    if (fabsf(dp1) < 0.0001f) {
      v0 = 0.0f;
      v1 = 0.0f;
      return;
    }

    if (inVel * dp1 <= 0.0f) {
      v0 = 0.0f;
    } else {
      float maxV0 = 1.5f * fabsf(dp1) / max(0.001f, dt1_s);
      v0 = constrain(inVel, -maxV0, maxV0);
    }

    if (!hasNextWp) {
      v1 = 0.0f; // Terminal waypoint stops at target
    } else {
      float dp2 = p2_next - p1;
      if (dp1 * dp2 <= 0.0f || cornerScale <= 0.001f) {
        v1 = 0.0f;
      } else {
        float s1 = dp1 / max(0.001f, dt1_s);
        float s2 = dp2 / max(0.001f, dt2_s);
        float sgn = (s1 > 0.0f) ? 1.0f : -1.0f;
        float vHarmonic = min(0.5f * (fabsf(s1) + fabsf(s2)), 1.25f * min(fabsf(s1), fabsf(s2)));
        v1 = sgn * vHarmonic * cornerScale;
      }
    }
  };

  float dtNext_s = hasNextWp ? max(0.001f, (float)nextWp.duration_ms / 1000.0f) : 0.001f;
  float dtCurr_s = max(0.001f, (float)wpTarget.duration_ms / 1000.0f);

  float inV1 = wasContinuous ? currentVelJ1 : 0.0f;
  float inV2 = wasContinuous ? currentVelJ2 : 0.0f;
  float inV3 = wasContinuous ? currentVelJ3 : 0.0f;
  float inV4 = wasContinuous ? currentVelJ4 : 0.0f;
  float inV5 = wasContinuous ? currentVelJ5 : 0.0f;
  float inV6 = wasContinuous ? currentVelJ6 : 0.0f;

  computeMonotoneTangents(sp1.j1, sp2.j1, nextWp.j1, hasNextWp, dtCurr_s, dtNext_s, inV1, sv1.j1, sv2.j1, sa1.j1, sa2.j1);
  computeMonotoneTangents(sp1.j2, sp2.j2, nextWp.j2, hasNextWp, dtCurr_s, dtNext_s, inV2, sv1.j2, sv2.j2, sa1.j2, sa2.j2);
  computeMonotoneTangents(sp1.j3, sp2.j3, nextWp.j3, hasNextWp, dtCurr_s, dtNext_s, inV3, sv1.j3, sv2.j3, sa1.j3, sa2.j3);
  computeMonotoneTangents(sp1.j4, sp2.j4, nextWp.j4, hasNextWp, dtCurr_s, dtNext_s, inV4, sv1.j4, sv2.j4, sa1.j4, sa2.j4);
  computeMonotoneTangents(sp1.j5, sp2.j5, nextWp.j5, hasNextWp, dtCurr_s, dtNext_s, inV5, sv1.j5, sv2.j5, sa1.j5, sa2.j5);
  computeMonotoneTangents(sp1.j6, sp2.j6, nextWp.j6, hasNextWp, dtCurr_s, dtNext_s, inV6, sv1.j6, sv2.j6, sa1.j6, sa2.j6);

  wasContinuous = true;

  activeSegmentDuration_s = max(0.001f, splineDuration / 1000000.0f);
  buildActiveCubicCoeffs(sp1, sp2, sv1, sv2, activeSegmentDuration_s);
  MotionTrajectory newTrajectory;
  newTrajectory.coeffJ1 = cubicCoeffJ1;
  newTrajectory.coeffJ2 = cubicCoeffJ2;
  newTrajectory.coeffJ3 = cubicCoeffJ3;
  newTrajectory.coeffJ4 = cubicCoeffJ4;
  newTrajectory.coeffJ5 = cubicCoeffJ5;
  newTrajectory.coeffJ6 = cubicCoeffJ6;
  newTrajectory.durationUs = splineDuration;
  newTrajectory.duration_s = activeSegmentDuration_s;
  activeTrajectory = newTrajectory;

  spPrev = sp1;
  splineT = 0.0f;
  lastSplineTime = micros();
  applyRunCurrent();
}

void stepLoopISR() {
  if (homingState != HOMING_IDLE) {
    return;
  }

  if (motionLoopActive && isMovingQueue) {
    stepper1.runSpeed();
    stepper2.runSpeed();
    stepper3.runSpeed();
    stepper4.runSpeed();
    stepper5.runSpeed();
    stepper6.runSpeed();
  } else if (streamMode) {
    stepper1.runSpeed();
    stepper2.runSpeed();
    stepper3.runSpeed();
    stepper4.runSpeed();
    stepper5.runSpeed();
    stepper6.runSpeed();
  } else {
    stepper1.run();
    stepper2.run();
    stepper3.run();
    stepper4.run();
    stepper5.run();
    stepper6.run();
  }
}

void motionLoopISR() {
  static unsigned long lastMotionUs = 0;
  unsigned long nowUs = micros();
  motionLoopDtUs = (nowUs > lastMotionUs) ? (float)(nowUs - lastMotionUs) : 0.0f;
  lastMotionUs = nowUs;

  if (!motionLoopActive) return;

  // Immediate ESTOP Quiescence: halt velocity commands within 1 tick (1 ms)
  if (estopActive) {
    motionTargetSpeed1 = 0.0f;
    motionTargetSpeed2 = 0.0f;
    motionTargetSpeed3 = 0.0f;
    motionTargetSpeed4 = 0.0f;
    motionTargetSpeed5 = 0.0f;
    motionTargetSpeed6 = 0.0f;
    stepper1.setSpeed(0.0f);
    stepper2.setSpeed(0.0f);
    stepper3.setSpeed(0.0f);
    stepper4.setSpeed(0.0f);
    stepper5.setSpeed(0.0f);
    stepper6.setSpeed(0.0f);
    isMovingQueue = false;
    return;
  }

  if (qHead != qTail && !isMovingQueue) {
    startSegmentFromQueue();
  }

  // Single unified spline evaluation pass
  if (isMovingQueue) {
    float currentOverride = max(0.05f, speedOverride);
    float realDuration_us = splineDuration / currentOverride;
    if (realDuration_us < 1000.0f) realDuration_us = 1000.0f;
    float realDuration_s = realDuration_us / 1000000.0f;
    if (realDuration_s < 0.001f) realDuration_s = 0.001f;

    splineT += motionLoopDtUs / realDuration_us;
    if (splineT >= 1.0f) splineT = 1.0f;

    float t = splineT;
    float pos1, velNorm1, accNorm1;
    float pos2, velNorm2, accNorm2;
    float pos3, velNorm3, accNorm3;
    float pos4, velNorm4, accNorm4;
    float pos5, velNorm5, accNorm5;
    float pos6, velNorm6, accNorm6;

    evaluateCubic(activeTrajectory.coeffJ1, t, pos1, velNorm1, accNorm1);
    evaluateCubic(activeTrajectory.coeffJ2, t, pos2, velNorm2, accNorm2);
    evaluateCubic(activeTrajectory.coeffJ3, t, pos3, velNorm3, accNorm3);
    evaluateCubic(activeTrajectory.coeffJ4, t, pos4, velNorm4, accNorm4);
    evaluateCubic(activeTrajectory.coeffJ5, t, pos5, velNorm5, accNorm5);
    evaluateCubic(activeTrajectory.coeffJ6, t, pos6, velNorm6, accNorm6);

    commandedAngle1 = pos1;
    commandedAngle2 = pos2;
    commandedAngle3 = pos3;
    commandedAngle4 = pos4;
    commandedAngle5 = pos5;
    commandedAngle6 = pos6;
    motionTargetPos1 = pos1;
    motionTargetPos2 = pos2;
    motionTargetPos3 = pos3;
    motionTargetPos4 = pos4;
    motionTargetPos5 = pos5;
    motionTargetPos6 = pos6;

    lastAccJ1 = (accNorm1 / (realDuration_s * realDuration_s)) * stepsPerDegAxis1;
    lastAccJ2 = (accNorm2 / (realDuration_s * realDuration_s)) * stepsPerDegGeared;
    lastAccJ3 = (accNorm3 / (realDuration_s * realDuration_s)) * stepsPerDegGeared * J3_STEP_SIGN;
    lastAccJ4 = (accNorm4 / (realDuration_s * realDuration_s)) * stepsPerDegAxis4;
    lastAccJ5 = (accNorm5 / (realDuration_s * realDuration_s)) * stepsPerDegAxis5;
    lastAccJ6 = (accNorm6 / (realDuration_s * realDuration_s)) * stepsPerDegAxis6;

    float vel1_deg = velNorm1 / realDuration_s;
    float vel2_deg = velNorm2 / realDuration_s;
    float vel3_deg = velNorm3 / realDuration_s;
    float vel4_deg = velNorm4 / realDuration_s;
    float vel5_deg = velNorm5 / realDuration_s;
    float vel6_deg = velNorm6 / realDuration_s;

    float spd1 = constrain(vel1_deg * stepsPerDegAxis1, -MAX_SPEED_J1 * currentOverride, MAX_SPEED_J1 * currentOverride);
    float spd2 = constrain(vel2_deg * stepsPerDegGeared, -MAX_SPEED_J2 * currentOverride, MAX_SPEED_J2 * currentOverride);
    float spd3 = constrain(vel3_deg * stepsPerDegGeared * J3_STEP_SIGN, -MAX_SPEED_J3 * currentOverride, MAX_SPEED_J3 * currentOverride);
    float spd4 = constrain(vel4_deg * stepsPerDegAxis4, -MAX_SPEED_J4 * currentOverride, MAX_SPEED_J4 * currentOverride);
    float spd5 = constrain(vel5_deg * stepsPerDegAxis5, -MAX_SPEED_J5 * currentOverride, MAX_SPEED_J5 * currentOverride);
    float spd6 = constrain(vel6_deg * stepsPerDegAxis6, -MAX_SPEED_J6 * currentOverride, MAX_SPEED_J6 * currentOverride);

    lastSpdJ1 = spd1;
    lastSpdJ2 = spd2;
    lastSpdJ3 = spd3;
    lastSpdJ4 = spd4;
    lastSpdJ5 = spd5;
    lastSpdJ6 = spd6;

    motionTargetSpeed1 = spd1;
    motionTargetSpeed2 = spd2;
    motionTargetSpeed3 = spd3;
    motionTargetSpeed4 = spd4;
    motionTargetSpeed5 = spd5;
    motionTargetSpeed6 = spd6;

    stepper1.setSpeed(spd1);
    stepper2.setSpeed(spd2);
    stepper3.setSpeed(spd3);
    stepper4.setSpeed(spd4);
    stepper5.setSpeed(spd5);
    stepper6.setSpeed(spd6);

    if (splineT >= 1.0f) {
      currentVelJ1 = velNorm1 / activeSegmentDuration_s;
      currentVelJ2 = velNorm2 / activeSegmentDuration_s;
      currentVelJ3 = velNorm3 / activeSegmentDuration_s;
      currentVelJ4 = velNorm4 / activeSegmentDuration_s;
      currentVelJ5 = velNorm5 / activeSegmentDuration_s;
      currentVelJ6 = velNorm6 / activeSegmentDuration_s;
      currentAccJ1 = accNorm1 / (activeSegmentDuration_s * activeSegmentDuration_s);
      currentAccJ2 = accNorm2 / (activeSegmentDuration_s * activeSegmentDuration_s);
      currentAccJ3 = accNorm3 / (activeSegmentDuration_s * activeSegmentDuration_s);
      currentAccJ4 = accNorm4 / (activeSegmentDuration_s * activeSegmentDuration_s);
      currentAccJ5 = accNorm5 / (activeSegmentDuration_s * activeSegmentDuration_s);
      currentAccJ6 = accNorm6 / (activeSegmentDuration_s * activeSegmentDuration_s);

      qTail = (qTail + 1) % WAYPOINT_QUEUE_SIZE;

      if (qHead != qTail) {
        // Seamlessly continue into next waypoint without dropping into hold mode
        startSegmentFromQueue();
      } else {
        // Queue is empty: cleanly transition to hold mode
        isMovingQueue = false;
        wasContinuous = false;
        motionTargetSpeed1 = 0.0f;
        motionTargetSpeed2 = 0.0f;
        motionTargetSpeed3 = 0.0f;
        motionTargetSpeed4 = 0.0f;
        motionTargetSpeed5 = 0.0f;
        motionTargetSpeed6 = 0.0f;
        stepper1.setSpeed(0.0f);
        stepper2.setSpeed(0.0f);
        stepper3.setSpeed(0.0f);
        stepper4.setSpeed(0.0f);
        stepper5.setSpeed(0.0f);
        stepper6.setSpeed(0.0f);
        lastSpdJ1 = 0.0f; lastSpdJ2 = 0.0f; lastSpdJ3 = 0.0f;
        lastSpdJ4 = 0.0f; lastSpdJ5 = 0.0f; lastSpdJ6 = 0.0f;
        lastAccJ1 = 0.0f; lastAccJ2 = 0.0f; lastAccJ3 = 0.0f;
        lastAccJ4 = 0.0f; lastAccJ5 = 0.0f; lastAccJ6 = 0.0f;
        currentVelJ1 = 0.0f; currentVelJ2 = 0.0f; currentVelJ3 = 0.0f;
        currentVelJ4 = 0.0f; currentVelJ5 = 0.0f; currentVelJ6 = 0.0f;
        currentAccJ1 = 0.0f; currentAccJ2 = 0.0f; currentAccJ3 = 0.0f;
        currentAccJ4 = 0.0f; currentAccJ5 = 0.0f; currentAccJ6 = 0.0f;

        if (calTestActive) {
          calTestDone = true;
          calTestActive = false;
        }

        motionLoopActive = false;
        applyHoldCurrent();
      }
    }
  }

  // Active Following-Error Collision Guard (20 Hz in ISR)
  static uint16_t isrCollCheckTicks = 0;
  if (++isrCollCheckTicks >= 50) {
    isrCollCheckTicks = 0;
    if (useEncoders && cachedEncoderValid && (isMovingQueue || streamMode)) {
      float err1 = fabsf(commandedAngle1 - cachedEncAngle1);
      float err2 = fabsf(commandedAngle2 - cachedEncAngle2);
      float err3 = fabsf(commandedAngle3 - cachedEncAngle3);
      if (err1 > 180.0f) err1 = 360.0f - err1;
      if (err2 > 180.0f) err2 = 360.0f - err2;
      if (err3 > 180.0f) err3 = 360.0f - err3;
      if ((encEnabled[0] && err1 > 20.0f) ||
          (encEnabled[1] && err2 > 20.0f) ||
          (encEnabled[2] && err3 > 20.0f)) {
        // Severe following error during motion -> motor stall or mechanical collision
        estopActive = true;
      }
    }
  }

  if (useEncoders) {
    updateClosedLoop();
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200); // ESP32 Serial
  SERIAL_PORT.begin(115200, SERIAL_8N1 | SERIAL_HALF_DUPLEX);
  myusb.begin(); // USB Host Init
  delay(100);
  while (!Serial && millis() < 3000)
    ;

  // ========== BANNER ==========
  Serial.println("\n\n");
  Serial.println("╔════════════════════════════════════════════════════╗");
  Serial.println("║   stppe_zlozenie_nowe 6-DOF ROBOT ARM CONTROLLER   ║");
  Serial.println("║      Inverse Kinematics + Closed Loop             ║");
  Serial.println("╚════════════════════════════════════════════════════╝");
  Serial.println();
  Serial.print("Firmware Version: ");
  Serial.println(FIRMWARE_VERSION);
  Serial.print("Build Date: ");
  Serial.print(BUILD_DATE);
  Serial.print(" ");
  Serial.println(BUILD_TIME);
  Serial.println();

  // ========== PIN SETUP ==========
  Serial.println("━━━━ HARDWARE INITIALIZATION ━━━━");
  Serial.print("[1/5] Configuring GPIO pins... ");
  pinMode(PIN_AXIS1_STEP, OUTPUT);
  pinMode(PIN_AXIS1_DIR, OUTPUT);
  pinMode(PIN_AXIS1_EN, OUTPUT);
  pinMode(PIN_AXIS2_STEP, OUTPUT);
  pinMode(PIN_AXIS2_DIR, OUTPUT);
  pinMode(PIN_AXIS2_EN, OUTPUT);
  pinMode(PIN_AXIS3_STEP, OUTPUT);
  pinMode(PIN_AXIS3_DIR, OUTPUT);
  pinMode(PIN_AXIS3_EN, OUTPUT);
  pinMode(PIN_MICRO_X_STEP, OUTPUT);
  pinMode(PIN_MICRO_X_DIR, OUTPUT);
  pinMode(PIN_MICRO_Y_STEP, OUTPUT);
  pinMode(PIN_MICRO_Y_DIR, OUTPUT);
  pinMode(PIN_MICRO_EN, OUTPUT);
  digitalWrite(PIN_MICRO_EN, LOW); // Enabled by default (Pin 12 shared with PIN_WRIST_EN)
  Serial.println("OK");

  // Limit switch pins
  pinMode(PIN_J1_LIMIT, INPUT_PULLUP);
  pinMode(PIN_J2_LIMIT, INPUT_PULLUP);
  pinMode(PIN_J3_LIMIT, INPUT_PULLUP);
  pinMode(PIN_MICRO_X_LIMIT, INPUT_PULLUP);
  pinMode(PIN_MICRO_Y_LIMIT, INPUT_PULLUP);

  // Enable Drivers
  digitalWrite(PIN_AXIS1_EN, LOW);
  digitalWrite(PIN_AXIS2_EN, LOW);
  digitalWrite(PIN_AXIS3_EN, LOW);

  // ========== TMC2209 DRIVERS ==========
  Serial.print("[2/5] Initializing TMC2209 Drivers... ");
  auto initDriver = [](TMC2209Stepper &d) {
    d.begin();
    delay(50);
    d.pdn_disable(true);
    delay(10);
    d.mstep_reg_select(true);
    delay(10);
    d.toff(4);
    delay(10);
    d.rms_current(1400); // 1400mA RMS: balanced strong torque without thermal throttling
    delay(10);
    d.ihold(16);
    delay(10);
    d.irun(28);
    delay(10);
    d.iholddelay(6);
    delay(10);
    d.microsteps(MICROSTEPS); // 16 microsteps base
    delay(10);
    d.intpol(true);           // Hardware 256-microstep interpolation (MicroPlyer) for butter-smooth rotation
    delay(10);
    d.en_spreadCycle(false);  // StealthChop2 enabled
    delay(10);
    d.pwm_autoscale(true);
    d.pwm_autograd(true);
    d.TPWMTHRS(0);            // 0 = pure StealthChop across full velocity spectrum (no chopper transition clicks)
  };
  initDriver(driver1);
  initDriver(driver2);
  initDriver(driver3);
  Serial.println("OK");

  // ========== STEPPERS & SERVOS ==========
  Serial.print("[3/5] Configuring Steppers & Servos... ");
  // Main axes: safe physical limits for hold mode and position corrections.
  pinMode(PIN_WRIST_EN, OUTPUT);
  digitalWrite(PIN_WRIST_EN, LOW); // Enable TMC2209 drivers for wrist J4, J5, J6

  stepper1.setMaxSpeed(MAX_SPEED_J1);    stepper1.setAcceleration(MAX_ACCEL_J1);
  stepper2.setMaxSpeed(MAX_SPEED_J2);    stepper2.setAcceleration(MAX_ACCEL_J2);
  stepper3.setMaxSpeed(MAX_SPEED_J3);    stepper3.setAcceleration(MAX_ACCEL_J3);
  stepper4.setMaxSpeed(MAX_SPEED_J4);    stepper4.setAcceleration(MAX_ACCEL_J4);
  stepper5.setMaxSpeed(MAX_SPEED_J5);    stepper5.setAcceleration(MAX_ACCEL_J5);
  stepper6.setMaxSpeed(MAX_SPEED_J6);    stepper6.setAcceleration(MAX_ACCEL_J6);

  stepperX.setMaxSpeed(500.0);
  stepperX.setAcceleration(500.0);
  stepperY.setMaxSpeed(500.0);
  stepperY.setAcceleration(500.0);

  loadBacklashFromEEPROM();
  Serial.println("OK");

  // ========== AS5600 ENCODERS ==========
  Serial.print("[4/5] Initializing AS5600 Encoders... ");

  // Wire2 → ENC1 (Base) (SDA=25, SCL=24) — Initialized but disabled by default
  Wire2.setSDA(25);
  Wire2.setSCL(24);
  Wire2.begin();
  Wire2.setClock(1000000); // 1MHz Fast Mode Plus
  // encEnabled[0] = false; // already set in global

  // Wire → ENC2 (Shoulder) (SDA=18, SCL=19)
  Wire.setSDA(18);
  Wire.setSCL(19);
  Wire.begin();
  Wire.setClock(1000000); // 1MHz Fast Mode Plus

  // Wire1 → ENC3 (Elbow) (SDA=44, SCL=45) — Initialized but disabled by default
  // FIX R-4: poprzednio komentarz błędnie opisywał tę oś jako "Shoulder" (to jest Elbow/J3).
  Wire1.setSDA(44);
  Wire1.setSCL(45);
  Wire1.begin();
  Wire1.setClock(1000000); // 1MHz Fast Mode Plus
  // encEnabled[2] = false; // already set in global

  delay(50);
  Serial.println("OK");

  // ========== PID CONTROLLERS ==========
  // Gentle proportional and integral correction; feedforward is disabled for position offset
  pid1.init(0.3f, 0.02f, 0.0f, 0.0f, 0.0f);
  pid2.init(0.4f, 0.02f, 0.0f, 0.0f, 0.15f);
  pid3.init(0.4f, 0.02f, 0.0f, 0.0f, 0.15f);

  uint16_t enc1Raw = encEnabled[0] ? readAS5600(Wire2) : 0xFFFF;
  uint16_t enc2Raw = readAS5600(Wire);
  uint16_t enc3Raw = encEnabled[2] ? readAS5600(Wire1) : 0xFFFF;

  bool setupAnyError = false;
  if (encEnabled[0] && enc1Raw == 0xFFFF) setupAnyError = true;
  if (encEnabled[1] && enc2Raw == 0xFFFF) setupAnyError = true;
  if (encEnabled[2] && enc3Raw == 0xFFFF) setupAnyError = true;

  if (setupAnyError) {
    Serial.println("WARN");
    Serial.println("      \u26a0 One or more ENABLED encoders not responding!");
    Serial.println("      Check magnet alignment and I2C connections");
  } else {
    Serial.println("OK");
  }

  enc1Offset = (encEnabled[0] && enc1Raw != 0xFFFF) ? (enc1Raw / 4096.0f) * 360.0f : 0.0f;
  enc2Offset = (enc2Raw != 0xFFFF) ? (enc2Raw / 4096.0f) * 360.0f : 0.0f;
  enc3Offset = (encEnabled[2] && enc3Raw != 0xFFFF) ? (enc3Raw / 4096.0f) * 360.0f : 0.0f;

  Serial.print("      Encoder Offsets: ");
  if (encEnabled[0]) { Serial.print(enc1Offset, 1); Serial.print(" deg"); } else { Serial.print("N/A"); }
  Serial.print(" | ");
  Serial.print(enc2Offset, 1); Serial.print(" deg");
  Serial.print(" | ");
  if (encEnabled[2]) { Serial.print(enc3Offset, 1); Serial.print(" deg"); } else { Serial.print("N/A"); }
  Serial.println();

  // ========== COMMUNICATION CHECK ==========
  Serial.print("[5/5] Checking Communication Links... ");
  Serial.println();

  // USB Host
  Serial.print("      USB Host (handheld): ");
  if (userial) {
    Serial.println("CONNECTED");
  } else {
    Serial.println("NOT DETECTED");
  }

  // ESP32 Serial
  Serial.print("      ESP32 (Serial1):      ");
  if (Serial1) {
    Serial.println("READY");
  } else {
    Serial.println("NOT READY");
  }

  Serial.println();

  // ========== SYSTEM HEALTH CHECK ==========
  Serial.println("━━━━ SYSTEM HEALTH CHECK ━━━━");
  printDriver("Axis 1", driver1);
  printDriver("Axis 2", driver2);
  printDriver("Axis 3", driver3);
  Serial.println();

  // ========== CONFIGURATION SUMMARY ==========
  Serial.println("━━━━ CONFIGURATION ━━━━");
  Serial.print("Robot Geometry: BASE_H=");
  Serial.print(BASE_H, 1);
  Serial.print("mm, L1=");
  Serial.print(L1, 1);
  Serial.print("mm, L2=");
  Serial.print(L2, 1);
  Serial.print("mm, L3=");
  Serial.print(L3, 1);
  Serial.println("mm");

  Serial.print("Axis 1 Gearing:    ");
  Serial.print(gearboxRatioAxis1, 2);
  Serial.print(":1 (");
  Serial.print(stepsPerDegAxis1, 2);
  Serial.println(" steps/deg)");

  Serial.print("Axis 2/3 Gearing:  ");
  Serial.print(gearboxRatio, 1);
  Serial.print(":1 (");
  Serial.print(stepsPerDegGeared, 2);
  Serial.println(" steps/deg)");

  Serial.print("Microsteps:        ");
  Serial.println(MICROSTEPS);

  Serial.print("TMC Current:       1800mA");
  Serial.println();

  Serial.println("Safety Limits:");
  Serial.print("  J1: ");
  Serial.print(LIMIT_J1_MIN, 0);
  Serial.print("° to ");
  Serial.print(LIMIT_J1_MAX, 0);
  Serial.println("°");

  Serial.print("  J2: ");
  Serial.print(LIMIT_J2_MIN, 0);
  Serial.print("° to ");
  Serial.print(LIMIT_J2_MAX, 0);
  Serial.println("°");

  Serial.print("  J3: ");
  Serial.print(LIMIT_J3_MIN, 0);
  Serial.print("° to ");
  Serial.print(LIMIT_J3_MAX, 0);
  Serial.println("°");
  Serial.println();

  Serial.print("Closed Loop:       ");
  Serial.println(useEncoders ? "ENABLED" : "DISABLED");
  Serial.println();

  // ========== COMMAND REFERENCE ==========
  Serial.println("━━━━ AVAILABLE COMMANDS ━━━━");
  Serial.println();
  Serial.println("MOTION CONTROL:");
  Serial.println("  MOV X# Y# Z# [W<0-6>] [J4#] [J5#] [D#ms]");
  Serial.println("  MOVL X# Y# Z# [W<0-6>] [J4#] [J5#] [D#ms] - Linear Move");
  Serial.println("    W0=poziomo  W1=dol_0  W2=dol_90");
  Serial.println("    W3=pitch0   W4=pitch90  W5=pitch180  W6=manual");
  Serial.println("  D1 <degrees>              - Move Axis 1 (Base) relative");
  Serial.println(
      "  D2 <degrees>              - Move Axis 2 (Shoulder) relative");
  Serial.println("  D3 <degrees>              - Move Axis 3 (Elbow) relative");
  Serial.println("  DX <steps>                - Move Gantry X relative");
  Serial.println("  DY <steps>                - Move Gantry Y relative");
  Serial.println("  A4 <angle>                - Set Servo 4 position (0-180)");
  Serial.println("  A5 <angle>                - Set Servo 5 position (0-180)");
  Serial.println();
  Serial.println("CALIBRATION & SETUP:");
  Serial.println("  HOME [SWITCH|ENCODER|MANUAL] - Zero all stepper positions (default SWITCH)");
  Serial.println("  HOMEJ1/HOMEJ2/HOMEJ3     - Home individual axes (switches)");
  Serial.println(
      "  CAL                       - Calibrate encoders to current position");
  Serial.println("  CL_ON                     - Enable closed-loop control");
  Serial.println("  CL_OFF                    - Disable closed-loop control");
  Serial.println("  REENABLE                  - Recover drivers from ESTOP");
  Serial.println();
  Serial.println("TEACH & PLAYBACK:");
  Serial.println();
  Serial.println("DIAGNOSTICS:");
  Serial.println("  ENC                       - Read encoder positions");
  Serial.println("  DIAG                      - TMC driver diagnostics");
  Serial.println("  ENCCHECK                  - Manual collision detection");
  Serial.println();
  Serial.println("CONFIGURATION:");
  Serial.println("  MS <speed>                - Set Gantry max speed");
  Serial.println("  SPEED <percent>           - Set speed override (5-100%)");
  Serial.println("  SAVECFG                   - Save config to EEPROM");
  Serial.println();
  Serial.println("PACKET FORMAT (from controller):");
  Serial.println("  <pos1,pos2,pos3,pos4,dx,dy>");
  Serial.println();

  // ========== READY ==========
  Serial.println("╔════════════════════════════════════════════════════╗");
  Serial.println("║              SYSTEM READY                         ║");
  Serial.println("╚════════════════════════════════════════════════════╝");
  Serial.println();
  Serial.println("Waiting for commands...");
  Serial.println();

  // Load persisted config from EEPROM
  eeprom_load_all();
  applySpeedOverride();

  // ihold default
  applyHoldCurrent();

  // SD support removed

  // Gripper servo
  gripper.attach(PIN_GRIPPER_SERVO);
  gripper.write(90);
  gripperAngle = 90.0f;

  // Watchdog — 8 second timeout; loop() must call watchdog.feed() every cycle
  WDT_timings_t wdtCfg;
  wdtCfg.trigger = 5;   // warning interrupt at 5 s
  wdtCfg.timeout = 8;   // hard reset at 8 s
  watchdog.begin(wdtCfg);

  Serial.println("Watchdog armed (8s timeout).");

  motionLoopActive = true;
  motionTimer.begin(motionLoopISR, 1000); // 1 kHz trajectory loop
  motionTimer.priority(64);

  stepTimer.begin(stepLoopISR, 30);       // ~33.3 kHz high-priority pulse generation
  stepTimer.priority(32);
}

// ================= LOOP =================
void loop() {
  unsigned long loopStart = micros();
  watchdog.feed();   // kick watchdog every loop
  myusb.Task();      // USB Host task

  // run/hold current switching — active during velocity-mode spline or holding
  bool anyRunning = (motionLoopActive && isMovingQueue) || streamMode ||
                    stepper1.isRunning() ||
                    stepper2.isRunning() ||
                    stepper3.isRunning() ||
                    stepper4.isRunning() ||
                    stepper5.isRunning() ||
                    stepper6.isRunning();
  if (anyRunning) applyRunCurrent();
  else            applyHoldCurrent();

  stepperX.run();
  stepperY.run();

  // --- ENCODER POLLING (50Hz non-blocking to prevent step pulse starvation) ---
  static unsigned long lastEncPollMs = 0;
  if (millis() - lastEncPollMs >= 20) {
    lastEncPollMs = millis();
    refreshEncoderCache();
  }

  // Process Inputs
  unsigned long inputStart = micros();
  processInput(Serial, inputBuffer, inputBufferPos, sizeof(inputBuffer));
  if (userial)
    processInput(userial, usbBuffer, usbBufferPos, sizeof(usbBuffer));
  processInput(Serial1, espBuffer, espBufferPos, sizeof(espBuffer));

  if (micros() - inputStart > MAX_INPUT_TIME_US) {
    Serial.println("WARN: Input processing slow");
  }

  // --- TELEMETRY STREAM (10Hz) ---
  static unsigned long lastTelem = 0;
  if (millis() - lastTelem >= 100) {
    lastTelem = millis();
    if (useTelemetry)     sendRobotPos();
    if (useJsonTelemetry) sendJSONTelemetry();
  }

  // Non-blocking homing state machine
  updateHoming();

  // Teach mode playback
  updateTeachPlayback();

  // --- EEPROM WRITE COALESCING (3-second debounce, only when idle) ---
  if (eepromDirty && !anyRunning && !isMovingQueue && (millis() - eepromDirtyTime >= EEPROM_DEBOUNCE_MS)) {
    eeprom_save_all();
    eepromDirty = false;
  }

  // --- CALIBRATION TEST RESULTS CHECK (deferred from ISR) ---
  if (calTestDone) {
    calTestDone = false;
    checkCalTestResults();
  }

  // --- WAYPOINT PROCESSING / HOLD / STREAM MODE ---
  if (homingState != HOMING_IDLE) {
    // Non-blocking homing active: stepper motors driven exclusively by updateHoming()
  } else if (motionLoopActive && isMovingQueue) {
    // Speeds and pulses generated by ISRs
  } else {
    // ── HOLD / STREAM MODE ──
    if (streamMode) {
      if (lastStreamMicros == 0) lastStreamMicros = micros();
      unsigned long nowUs = micros();
      float dt = (nowUs - lastStreamMicros) / 1000000.0f;
      lastStreamMicros = nowUs;
      if (dt > 0.05f) dt = 0.05f;

      // Acceleration limit (deg/s^2) safely within TMC2209 torque limits
      float maxAcc = 140.0f;

      auto updateTracking = [&](float target, volatile float &pos, float &vel, float maxVel) {
        float error = target - pos;
        if (fabsf(error) < 0.005f && fabsf(vel) < 0.05f) {
          pos = target;
          vel = 0.0f;
          return;
        }

        float sgn = (error > 0.0f) ? 1.0f : -1.0f;
        float absErr = fabsf(error);
        float targetSpeed = sgn * sqrtf(2.0f * maxAcc * absErr);
        targetSpeed = constrain(targetSpeed, -maxVel, maxVel);

        float maxVelDelta = maxAcc * dt;
        vel = constrain(targetSpeed, vel - maxVelDelta, vel + maxVelDelta);
        pos += vel * dt;
      };

      updateTracking(streamTargetJ1, commandedAngle1, streamVelJ1, MAX_SPEED_J1 / stepsPerDegAxis1);
      updateTracking(streamTargetJ2, commandedAngle2, streamVelJ2, MAX_SPEED_J2 / stepsPerDegGeared);
      updateTracking(streamTargetJ3, commandedAngle3, streamVelJ3, MAX_SPEED_J3 / stepsPerDegGeared);
      updateTracking(streamTargetJ4, commandedAngle4, streamVelJ4, MAX_SPEED_J4 / stepsPerDegAxis4);
      updateTracking(streamTargetJ5, commandedAngle5, streamVelJ5, MAX_SPEED_J5 / stepsPerDegAxis5);
      updateTracking(streamTargetJ6, commandedAngle6, streamVelJ6, MAX_SPEED_J6 / stepsPerDegAxis6);

      stepper1.setSpeed(streamVelJ1 * stepsPerDegAxis1);
      stepper2.setSpeed(streamVelJ2 * stepsPerDegGeared);
      stepper3.setSpeed(streamVelJ3 * stepsPerDegGeared * J3_STEP_SIGN);
      stepper4.setSpeed(streamVelJ4 * stepsPerDegAxis4);
      stepper5.setSpeed(streamVelJ5 * stepsPerDegAxis5);
      stepper6.setSpeed(streamVelJ6 * stepsPerDegAxis6);

      if (millis() - streamLastUpdateMs > STREAM_TIMEOUT_MS) {
        stopStreamMode(true);
      }
    } else {
      float p1 = useEncoders && jointEnabled[0] && encEnabled[0] ? pidAdj1 : 0.0f;
      float p2 = useEncoders && jointEnabled[1] && encEnabled[1] ? pidAdj2 : 0.0f;
      float p3 = useEncoders && jointEnabled[2] && encEnabled[2] ? pidAdj3 : 0.0f;

      long tgt1 = (long)roundf((commandedAngle1 + p1) * stepsPerDegAxis1) + (long)backlashOffsetSteps[0];
      long tgt2 = (long)roundf((commandedAngle2 + p2) * stepsPerDegGeared) + (long)backlashOffsetSteps[1];
      long tgt3 = j3AngleToSteps(commandedAngle3 + p3) + (long)backlashOffsetSteps[2];
      long tgt4 = j4AngleToSteps(commandedAngle4) + (long)backlashOffsetSteps[3];
      long tgt5 = j5AngleToSteps(commandedAngle5) + (long)backlashOffsetSteps[4];
      long tgt6 = j6AngleToSteps(commandedAngle6) + (long)backlashOffsetSteps[5];

      noInterrupts();
      if (tgt1 != stepper1.targetPosition()) stepper1.moveTo(tgt1);
      if (tgt2 != stepper2.targetPosition()) stepper2.moveTo(tgt2);
      if (tgt3 != stepper3.targetPosition()) stepper3.moveTo(tgt3);
      if (tgt4 != stepper4.targetPosition()) stepper4.moveTo(tgt4);
      if (tgt5 != stepper5.targetPosition()) stepper5.moveTo(tgt5);
      if (tgt6 != stepper6.targetPosition()) stepper6.moveTo(tgt6);
      interrupts();
    }
  }

  // ── Post-move encoder collision check with 100ms settling time ─────────────────
  static bool wasRunning = false;
  static unsigned long stopTimestamp = 0;
  static bool checkPending = false;
  if (wasRunning && !anyRunning) {   // just stopped
    stopTimestamp = millis();
    checkPending = true;
  }
  wasRunning = anyRunning;

  if (checkPending && !anyRunning && (millis() - stopTimestamp >= 100)) {
    checkPending = false;
    checkEncoders();
    sendRobotPos();                  // AR4-style position reply
  }

  // Loop Timing Check (rate-limited to 1Hz warning to prevent serial spam)
  static unsigned long lastOverrunWarn = 0;
  if (micros() - loopStart > MAX_LOOP_TIME_US) {
    if (millis() - lastOverrunWarn >= 1000) {
      lastOverrunWarn = millis();
      Serial.print("WARN: Loop overrun: ");
      Serial.println(micros() - loopStart);
    }
  }
}

// ================= IK — Analytical 2-Link Planar =================
void calculateIK(float x, float y, float z, unsigned long duration,
                 WristMode wristMode, float j4_manual_deg, float j5_deg, bool isMovL, float j6_deg) {
    x -= workOffset.x;  y -= workOffset.y;  z -= workOffset.z;
    x += toolOffset.x;  y += toolOffset.y;  z += toolOffset.z;

    Serial.print("IK Target: X="); Serial.print(x);
    Serial.print(" Y="); Serial.print(y);
    Serial.print(" Z="); Serial.println(z);

    // 1. J1 — rotation in XY plane
    float theta1 = atan2f(y, x);
    float deg1 = theta1 * RAD_TO_DEG;

    // 2. Horizontal reach from J1 axis
    float r = sqrtf(x*x + y*y);

    bool was_clamped = false;
    // Floor collision clamp
    if (z < FLOOR_CLR) {
        z = FLOOR_CLR;
        was_clamped = true;
    }
    // Base exclusion zone clamp
    if (r*r < EXCL_R2 && z < BASE_H) {
        r = sqrtf(EXCL_R2);
        was_clamped = true;
    }

    // 3. Decouple Wrist Center J5 based on wrist orientation
    float r5 = r;
    float z5 = z;
    if (wristMode == WristMode::DOWN_0 || wristMode == WristMode::DOWN_90 || wristMode == WristMode::PITCH_90) {
        z5 = z + TOOL_TOTAL_LEN;
    } else if (wristMode == WristMode::PITCH_180) {
        z5 = z - TOOL_TOTAL_LEN;
    } else if (wristMode == WristMode::MANUAL) {
        float pitch_rad = j4_manual_deg * DEG_TO_RAD;
        r5 = fmaxf(0.0f, r - TOOL_TOTAL_LEN * cosf(pitch_rad));
        z5 = z - TOOL_TOTAL_LEN * sinf(pitch_rad);
    } else {
        r5 = fmaxf(0.0f, r - TOOL_TOTAL_LEN);
    }

    float dr = r5;
    float dz = z5 - BASE_H;
    float D2 = dr*dr + dz*dz;
    float D = sqrtf(D2);

    float C_gamma = (L1*L1 + L2*L2 - D2) / (2.0f * L1 * L2);
    if (C_gamma < -1.0f) { C_gamma = -1.0f; was_clamped = true; }
    if (C_gamma >  1.0f) { C_gamma =  1.0f; was_clamped = true; }

    float gamma = acosf(C_gamma);
    float j3_1_deg = (gamma - GAMMA_0) * RAD_TO_DEG;
    float j3_2_deg = (-gamma - GAMMA_0) * RAD_TO_DEG;

    float beta = atan2f(dr, dz);
    float C_alpha = (L1*L1 + D2 - L2*L2) / (2.0f * L1 * fmaxf(1e-6f, D));
    C_alpha = fmaxf(-1.0f, fminf(1.0f, C_alpha));
    float alpha = acosf(C_alpha);

    float j2_1_deg = (TH1_0 - (beta - alpha)) * RAD_TO_DEG;
    float j2_2_deg = (TH1_0 - (beta + alpha)) * RAD_TO_DEG;

    // Joint limit clamping
    if (deg1 < LIMIT_J1_MIN) { deg1 = LIMIT_J1_MIN; was_clamped = true; }
    if (deg1 > LIMIT_J1_MAX) { deg1 = LIMIT_J1_MAX; was_clamped = true; }

    auto clampJ = [](float j, float minJ, float maxJ, bool &clamped) {
        if (j < minJ) { clamped = true; return minJ; }
        if (j > maxJ) { clamped = true; return maxJ; }
        return j;
    };

    bool sol1_clamped = was_clamped;
    float c_j2_1 = clampJ(j2_1_deg, LIMIT_J2_MIN, LIMIT_J2_MAX, sol1_clamped);
    float c_j3_1 = clampJ(j3_1_deg, LIMIT_J3_MIN, LIMIT_J3_MAX, sol1_clamped);

    bool sol2_clamped = was_clamped;
    float c_j2_2 = clampJ(j2_2_deg, LIMIT_J2_MIN, LIMIT_J2_MAX, sol2_clamped);
    float c_j3_2 = clampJ(j3_2_deg, LIMIT_J3_MIN, LIMIT_J3_MAX, sol2_clamped);

    bool sol1_valid = checkWorkspace(deg1, c_j2_1, c_j3_1, 0.0f, 0.0f, 0.0f);
    bool sol2_valid = checkWorkspace(deg1, c_j2_2, c_j3_2, 0.0f, 0.0f, 0.0f);

    float best_j2 = 0, best_j3 = 0;
    bool found_solution = false;

    if (sol1_valid && sol2_valid) {
        float d1 = fabsf(c_j2_1 - commandedAngle2) + fabsf(c_j3_1 - commandedAngle3);
        float d2 = fabsf(c_j2_2 - commandedAngle2) + fabsf(c_j3_2 - commandedAngle3);
        if (d1 < d2) {
            best_j2 = c_j2_1; best_j3 = c_j3_1; was_clamped = sol1_clamped;
        } else {
            best_j2 = c_j2_2; best_j3 = c_j3_2; was_clamped = sol2_clamped;
        }
        found_solution = true;
    } else if (sol1_valid) {
        best_j2 = c_j2_1; best_j3 = c_j3_1; was_clamped = sol1_clamped;
        found_solution = true;
    } else if (sol2_valid) {
        best_j2 = c_j2_2; best_j3 = c_j3_2; was_clamped = sol2_clamped;
        found_solution = true;
    }

    if (!found_solution) {
        Serial.println("ERR:WORKSPACE_VIOLATION");
        return;
    }

    if (was_clamped) {
        Serial.println("WARN: Out of workspace - clamped to closest valid");
    }

    Serial.print("IK Solution: J1="); Serial.print(deg1,2);
    Serial.print(" J2="); Serial.print(best_j2,2);
    Serial.print(" J3="); Serial.println(best_j3,2);

    // Wrist J4, J5, J6 computation matching RobotKinematics.js
    float ang34_deg = best_j2 + best_j3;
    float calc_j4 = 0.0f, calc_j5 = 0.0f, calc_j6 = j6_deg;
    const float basePitchOffsetDeg = -(TOOL_REST_ANGLE * RAD_TO_DEG);

    switch (wristMode) {
        case WristMode::LEVEL:
            calc_j4 = 0.0f;
            calc_j5 = basePitchOffsetDeg - ang34_deg;
            calc_j6 = 0.0f;
            break;
        case WristMode::DOWN_0:
            calc_j4 = 0.0f;
            calc_j5 = basePitchOffsetDeg - ang34_deg - 90.0f;
            calc_j6 = 0.0f;
            break;
        case WristMode::DOWN_90:
            calc_j4 = 0.0f;
            calc_j5 = basePitchOffsetDeg - ang34_deg - 90.0f;
            calc_j6 = 90.0f;
            break;
        case WristMode::PITCH_0:
            calc_j4 = 0.0f;
            calc_j5 = basePitchOffsetDeg - ang34_deg;
            calc_j6 = j6_deg;
            break;
        case WristMode::PITCH_90:
            calc_j4 = 0.0f;
            calc_j5 = basePitchOffsetDeg - ang34_deg - 90.0f;
            calc_j6 = j6_deg;
            break;
        case WristMode::PITCH_180:
            calc_j4 = 0.0f;
            calc_j5 = basePitchOffsetDeg - ang34_deg + 90.0f;
            calc_j6 = j6_deg;
            break;
        case WristMode::MANUAL:
        default:
            calc_j4 = j4_manual_deg;
            calc_j5 = j5_deg - (TOOL_REST_ANGLE * RAD_TO_DEG) - ang34_deg;
            calc_j6 = j6_deg;
            break;
    }

    calc_j4 = constrain(calc_j4, LIMIT_J4_MIN, LIMIT_J4_MAX);
    calc_j5 = constrain(calc_j5, LIMIT_J5_MIN, LIMIT_J5_MAX);
    calc_j6 = constrain(calc_j6, LIMIT_J6_MIN, LIMIT_J6_MAX);

    if (!checkWorkspace(deg1, best_j2, best_j3, calc_j4, calc_j5, calc_j6)) {
        Serial.println("ERR:WORKSPACE_VIOLATION (wrist)");
        return;
    }

    Serial.print("  Wrist W"); Serial.print(static_cast<uint8_t>(wristMode));
    Serial.print(" J4="); Serial.print(calc_j4,1);
    Serial.print("deg J5="); Serial.print(calc_j5,1);
    Serial.print("deg J6="); Serial.print(calc_j6,1); Serial.println("deg");

    moveToAngles(deg1, best_j2, best_j3, duration, false, calc_j4, calc_j5, isMovL, calc_j6);
}

// ================= 6-DOF INVERSE KINEMATICS WITH SINGULARITY DAMPENING =================
void calculateIK6DOF(float x, float y, float z, float roll_deg, float pitch_deg, float yaw_deg,
                     unsigned long duration, bool isMovL) {
    x -= workOffset.x;  y -= workOffset.y;  z -= workOffset.z;
    x += toolOffset.x;  y += toolOffset.y;  z += toolOffset.z;

    Serial.print("IK6DOF Target: X="); Serial.print(x);
    Serial.print(" Y="); Serial.print(y);
    Serial.print(" Z="); Serial.print(z);
    Serial.print(" R="); Serial.print(roll_deg);
    Serial.print(" P="); Serial.print(pitch_deg);
    Serial.print(" YW="); Serial.println(yaw_deg);

    float r = roll_deg * DEG_TO_RAD;
    float p = pitch_deg * DEG_TO_RAD;
    float yw = yaw_deg * DEG_TO_RAD;

    float cr = cosf(r), sr = sinf(r);
    float cp = cosf(p), sp = sinf(p);
    float cy = cosf(yw), sy = sinf(yw);

    // R06 = Rz(yaw) * Ry(pitch) * Rx(roll)
    float R06[3][3] = {
      { cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr },
      { sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr },
      { -sp,     cp * sr,                cp * cr }
    };

    float ux = R06[0][0];
    float uy = R06[1][0];
    float uz = R06[2][0];

    // 1. Decouple Wrist Center P5
    float x5 = x - TOOL_TOTAL_LEN * ux;
    float y5 = y - TOOL_TOTAL_LEN * uy;
    float z5 = z - TOOL_TOTAL_LEN * uz;

    // 2. Base J1 (Shoulder singularity damping when r5 ~ 0)
    float r5 = sqrtf(x5 * x5 + y5 * y5);
    float deg1 = (r5 < 0.5f) ? commandedAngle1 : (atan2f(y5, x5) * RAD_TO_DEG);

    bool was_clamped = false;
    float zc = z5;
    if (zc < FLOOR_CLR) {
        zc = FLOOR_CLR;
        was_clamped = true;
    }
    float rc = r5;
    if (rc * rc < EXCL_R2 && zc < BASE_H) {
        rc = sqrtf(EXCL_R2);
        was_clamped = true;
    }

    // 3. Planar 2-Link IK for J2, J3
    float dr = rc;
    float dz = zc - BASE_H;
    float D2 = dr * dr + dz * dz;
    float D = sqrtf(D2);

    // Reach boundary singularity dampening
    const float maxReach = L1 + L2;
    if (D > maxReach * 0.999f) {
        D = maxReach * 0.999f;
        D2 = D * D;
        was_clamped = true;
    }

    float C_gamma = (L1 * L1 + L2 * L2 - D2) / (2.0f * L1 * L2);
    C_gamma = fmaxf(-1.0f, fminf(1.0f, C_gamma));
    float gamma = acosf(C_gamma);

    float beta = atan2f(dr, dz);
    float C_alpha = (L1 * L1 + D2 - L2 * L2) / (2.0f * L1 * fmaxf(1e-6f, D));
    C_alpha = fmaxf(-1.0f, fminf(1.0f, C_alpha));
    float alpha = acosf(C_alpha);

    float cand1_j2 = (TH1_0 - (beta - alpha)) * RAD_TO_DEG;
    float cand1_j3 = (gamma - GAMMA_0) * RAD_TO_DEG;

    float cand2_j2 = (TH1_0 - (beta + alpha)) * RAD_TO_DEG;
    float cand2_j3 = (-gamma - GAMMA_0) * RAD_TO_DEG;

    auto clampJ = [](float j, float minJ, float maxJ, bool &clamped) {
        if (j < minJ) { clamped = true; return minJ; }
        if (j > maxJ) { clamped = true; return maxJ; }
        return j;
    };

    if (deg1 < LIMIT_J1_MIN) { deg1 = LIMIT_J1_MIN; was_clamped = true; }
    if (deg1 > LIMIT_J1_MAX) { deg1 = LIMIT_J1_MAX; was_clamped = true; }

    bool sol1_clamped = was_clamped;
    float c_j2_1 = clampJ(cand1_j2, LIMIT_J2_MIN, LIMIT_J2_MAX, sol1_clamped);
    float c_j3_1 = clampJ(cand1_j3, LIMIT_J3_MIN, LIMIT_J3_MAX, sol1_clamped);

    bool sol2_clamped = was_clamped;
    float c_j2_2 = clampJ(cand2_j2, LIMIT_J2_MIN, LIMIT_J2_MAX, sol2_clamped);
    float c_j3_2 = clampJ(cand2_j3, LIMIT_J3_MIN, LIMIT_J3_MAX, sol2_clamped);

    bool sol1_valid = checkWorkspace(deg1, c_j2_1, c_j3_1, 0.0f, 0.0f, 0.0f);
    bool sol2_valid = checkWorkspace(deg1, c_j2_2, c_j3_2, 0.0f, 0.0f, 0.0f);

    float best_j2 = 0.0f, best_j3 = 0.0f;
    bool found_solution = false;

    if (sol1_valid && sol2_valid) {
        float d1 = fabsf(c_j2_1 - commandedAngle2) + fabsf(c_j3_1 - commandedAngle3);
        float d2 = fabsf(c_j2_2 - commandedAngle2) + fabsf(c_j3_2 - commandedAngle3);
        if (d1 < d2) {
            best_j2 = c_j2_1; best_j3 = c_j3_1; was_clamped = sol1_clamped;
        } else {
            best_j2 = c_j2_2; best_j3 = c_j3_2; was_clamped = sol2_clamped;
        }
        found_solution = true;
    } else if (sol1_valid) {
        best_j2 = c_j2_1; best_j3 = c_j3_1; was_clamped = sol1_clamped;
        found_solution = true;
    } else if (sol2_valid) {
        best_j2 = c_j2_2; best_j3 = c_j3_2; was_clamped = sol2_clamped;
        found_solution = true;
    }

    if (!found_solution) {
        Serial.println("ERR:WORKSPACE_VIOLATION");
        return;
    }

    // 4. Forearm Frame R03
    float j1r = deg1 * DEG_TO_RAD;
    float j2r = best_j2 * DEG_TO_RAD;
    float j3r = best_j3 * DEG_TO_RAD;
    float th2 = TH2_0 - (j2r + j3r);
    float forearm_pitch = HALF_PI - th2;

    float c1_ = cosf(j1r), s1_ = sinf(j1r);
    float cp_ = cosf(forearm_pitch), sp_ = sinf(forearm_pitch);

    float R03[3][3] = {
      { c1_ * cp_, -s1_, c1_ * sp_ },
      { s1_ * cp_,  c1_, s1_ * sp_ },
      { -sp_,        0,  cp_ }
    };

    // 5. Wrist Rotation Matrix: R36 = R03^T * R06
    float R36[3][3];
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        R36[i][j] = R03[0][i] * R06[0][j] + R03[1][i] * R06[1][j] + R03[2][i] * R06[2][j];
      }
    }

    // Decompose R36 into J4, J5, J6 with singularity dampening
    float cosJ5 = fmaxf(-1.0f, fminf(1.0f, R36[0][0]));
    float sinJ5_sq = R36[1][0] * R36[1][0] + R36[2][0] * R36[2][0];
    float sinJ5 = sqrtf(sinJ5_sq);

    float calc_j4, calc_j5, calc_j6;
    const float WRIST_SINGULARITY_EPS = 0.0436f; // ~2.5 deg

    if (sinJ5 < WRIST_SINGULARITY_EPS) {
        // Wrist Singularity: J4 and J6 are collinear!
        calc_j5 = acosf(cosJ5) * RAD_TO_DEG;
        calc_j4 = commandedAngle4;
        float totalRoll = atan2f(R36[2][1] - R36[1][2], R36[1][1] + R36[2][2]) * RAD_TO_DEG;
        calc_j6 = totalRoll - calc_j4;
        while (calc_j6 > 180.0f) calc_j6 -= 360.0f;
        while (calc_j6 < -180.0f) calc_j6 += 360.0f;
    } else {
        float j5_1 = atan2f(sinJ5, cosJ5) * RAD_TO_DEG;
        float j4_1 = atan2f(R36[1][0], -R36[2][0]) * RAD_TO_DEG;
        float j6_1 = atan2f(R36[0][1], R36[0][2]) * RAD_TO_DEG;

        float j5_2 = atan2f(-sinJ5, cosJ5) * RAD_TO_DEG;
        float j4_2 = atan2f(-R36[1][0], R36[2][0]) * RAD_TO_DEG;
        float j6_2 = atan2f(-R36[0][1], -R36[0][2]) * RAD_TO_DEG;

        bool b1_valid = (j4_1 >= LIMIT_J4_MIN - 0.01f && j4_1 <= LIMIT_J4_MAX + 0.01f &&
                         j5_1 >= LIMIT_J5_MIN - 0.01f && j5_1 <= LIMIT_J5_MAX + 0.01f &&
                         j6_1 >= LIMIT_J6_MIN - 0.01f && j6_1 <= LIMIT_J6_MAX + 0.01f);
        bool b2_valid = (j4_2 >= LIMIT_J4_MIN - 0.01f && j4_2 <= LIMIT_J4_MAX + 0.01f &&
                         j5_2 >= LIMIT_J5_MIN - 0.01f && j5_2 <= LIMIT_J5_MAX + 0.01f &&
                         j6_2 >= LIMIT_J6_MIN - 0.01f && j6_2 <= LIMIT_J6_MAX + 0.01f);

        if (b1_valid && !b2_valid) {
            calc_j4 = j4_1; calc_j5 = j5_1; calc_j6 = j6_1;
        } else if (b2_valid && !b1_valid) {
            calc_j4 = j4_2; calc_j5 = j5_2; calc_j6 = j6_2;
        } else {
            float dist1 = fabsf(j4_1 - commandedAngle4) + fabsf(j5_1 - commandedAngle5) + fabsf(j6_1 - commandedAngle6);
            float dist2 = fabsf(j4_2 - commandedAngle4) + fabsf(j5_2 - commandedAngle5) + fabsf(j6_2 - commandedAngle6);
            if (dist2 < dist1) {
                calc_j4 = j4_2; calc_j5 = j5_2; calc_j6 = j6_2;
            } else {
                calc_j4 = j4_1; calc_j5 = j5_1; calc_j6 = j6_1;
            }
        }
    }

    calc_j4 = clampJ(calc_j4, LIMIT_J4_MIN, LIMIT_J4_MAX, was_clamped);
    calc_j5 = clampJ(calc_j5, LIMIT_J5_MIN, LIMIT_J5_MAX, was_clamped);
    calc_j6 = clampJ(calc_j6, LIMIT_J6_MIN, LIMIT_J6_MAX, was_clamped);

    if (!checkWorkspace(deg1, best_j2, best_j3, calc_j4, calc_j5, calc_j6)) {
        Serial.println("ERR:WORKSPACE_VIOLATION (wrist)");
        return;
    }

    if (was_clamped) {
        Serial.println("WARN: Out of workspace - clamped to closest valid");
    }

    Serial.print("IK6DOF Solution: J1="); Serial.print(deg1, 2);
    Serial.print(" J2="); Serial.print(best_j2, 2);
    Serial.print(" J3="); Serial.print(best_j3, 2);
    Serial.print(" J4="); Serial.print(calc_j4, 2);
    Serial.print(" J5="); Serial.print(calc_j5, 2);
    Serial.print(" J6="); Serial.println(calc_j6, 2);

    moveToAngles(deg1, best_j2, best_j3, duration, false, calc_j4, calc_j5, isMovL, calc_j6);
}

void moveToAngles(float t1, float t2, float t3, unsigned long duration_ms,
                  bool skipWorkspaceCheck, float t4, float t5, bool isMovL, float t6) {
    if (estopActive) {
        Serial.println("ERR: Motion rejected - ESTOP is active");
        return;
    }
    t1 = constrain(t1, LIMIT_J1_MIN, LIMIT_J1_MAX);
    t2 = constrain(t2, LIMIT_J2_MIN, LIMIT_J2_MAX);
    t3 = constrain(t3, LIMIT_J3_MIN, LIMIT_J3_MAX);

    float cur_t4 = (t4 >= -998.0f) ? constrain(t4, LIMIT_J4_MIN, LIMIT_J4_MAX) : commandedAngle4;
    float cur_t5 = (t5 >= -998.0f) ? constrain(t5, LIMIT_J5_MIN, LIMIT_J5_MAX) : commandedAngle5;
    float cur_t6 = (t6 >= -998.0f) ? constrain(t6, LIMIT_J6_MIN, LIMIT_J6_MAX) : commandedAngle6;

    if (!skipWorkspaceCheck) {
        if (!checkWorkspace(t1, t2, t3, cur_t4, cur_t5, cur_t6)) {
            Serial.println("ERR:WORKSPACE_VIOLATION");
            return;
        }
    }

    if (duration_ms == 0) {
        // Activate critically damped live stream mode (bypasses spline queue)
        streamTargetJ1 = t1;
        streamTargetJ2 = t2;
        streamTargetJ3 = t3;
        streamTargetJ4 = cur_t4;
        streamTargetJ5 = cur_t5;
        streamTargetJ6 = cur_t6;
        if (!streamMode) {
            streamVelJ1 = streamVelJ2 = streamVelJ3 = streamVelJ4 = streamVelJ5 = streamVelJ6 = 0.0f;
            lastStreamMicros = 0;
            streamMode = true;
        }
        qHead = qTail;
        isMovingQueue = false;
        wasContinuous = false;
        motionLoopActive = false;
        return;
    }

    // Any standard queued waypoint MUST cancel live stream mode
    if (streamMode) stopStreamMode(false);

    noInterrupts();
    uint8_t nextHead = (qHead + 1) % WAYPOINT_QUEUE_SIZE;
    if (nextHead == qTail) {
        if (isMovL) {
            interrupts();
            while ( ((volatile uint8_t)qHead + 1) % WAYPOINT_QUEUE_SIZE == (volatile uint8_t)qTail ) {
                if (estopActive) break;
                watchdog.feed();
                myusb.Task();
                delayMicroseconds(500);
            }
            noInterrupts();
            nextHead = (qHead + 1) % WAYPOINT_QUEUE_SIZE;
        } else {
            uint8_t lastSlot = (qHead > 0) ? (qHead - 1) : (WAYPOINT_QUEUE_SIZE - 1);
            motionQueue[lastSlot] = { t1, t2, t3, cur_t4, cur_t5, cur_t6, duration_ms };
            interrupts();
            return;
        }
    }

    // Enqueue waypoint into FIFO lookahead buffer
    motionQueue[qHead] = { t1, t2, t3, cur_t4, cur_t5, cur_t6, duration_ms };
    qHead = nextHead;
    motionLoopActive = true;
    interrupts();
}

// ================= SIMPLE CONTROL COMMANDS =================

// REFACTOR R-1: wspólna funkcja dla D1/D2/D3 — eliminuje 3 identyczne bloki.
// Poprzednio każda oś miała 10-liniowy kopipejst różniący się tylko numerem.
static void handleStreamJog(int axis, float delta) {
  if (axis < 1 || axis > 6) return;
  if (!jointEnabled[axis - 1]) {
    Serial.printf("WARN: J%d disabled\n", axis);
    return;
  }

  if (!streamMode) {
    streamVelJ1 = streamVelJ2 = streamVelJ3 = streamVelJ4 = streamVelJ5 = streamVelJ6 = 0.0f;
    lastStreamMicros = 0;
  }

  float current, limitMin, limitMax;
  switch (axis) {
    case 1: current = commandedAngle1; limitMin = LIMIT_J1_MIN; limitMax = LIMIT_J1_MAX; break;
    case 2: current = commandedAngle2; limitMin = LIMIT_J2_MIN; limitMax = LIMIT_J2_MAX; break;
    case 3: current = commandedAngle3; limitMin = LIMIT_J3_MIN; limitMax = LIMIT_J3_MAX; break;
    case 4: current = commandedAngle4; limitMin = LIMIT_J4_MIN; limitMax = LIMIT_J4_MAX; break;
    case 5: current = commandedAngle5; limitMin = LIMIT_J5_MIN; limitMax = LIMIT_J5_MAX; break;
    default: current = commandedAngle6; limitMin = LIMIT_J6_MIN; limitMax = LIMIT_J6_MAX; break;
  }

  float req = constrain(current + delta, limitMin, limitMax);

  streamTargetJ1 = commandedAngle1;
  streamTargetJ2 = commandedAngle2;
  streamTargetJ3 = commandedAngle3;
  streamTargetJ4 = commandedAngle4;
  streamTargetJ5 = commandedAngle5;
  streamTargetJ6 = commandedAngle6;

  if      (axis == 1) streamTargetJ1 = req;
  else if (axis == 2) streamTargetJ2 = req;
  else if (axis == 3) streamTargetJ3 = req;
  else if (axis == 4) streamTargetJ4 = req;
  else if (axis == 5) streamTargetJ5 = req;
  else if (axis == 6) streamTargetJ6 = req;

  noInterrupts();
  streamMode = true;
  streamLastUpdateMs = millis();
  qHead = qTail; isMovingQueue = false; wasContinuous = false;
  interrupts();
}

void parseCommand(const char *command, float value) {
  if (estopActive) {
    if (strcmp(command, "REENABLE") != 0 && strcmp(command, "ESTOP") != 0 &&
        strcmp(command, "STATUS") != 0 && strcmp(command, "POS") != 0 &&
        strcmp(command, "CALDUMP") != 0 && strcmp(command, "VER") != 0 &&
        strncmp(command, "GET", 3) != 0) {
      Serial.println("ERR: Motion rejected - ESTOP is active. Send REENABLE first.");
      return;
    }
  }

  if (strcmp(command, "MOV") == 0) {
    // MOV command is handled in processInput; no action needed here.
    return;
  }
  else if (strcmp(command, "D1") == 0) {
    handleStreamJog(1, value);
  } else if (strcmp(command, "D2") == 0) {
    handleStreamJog(2, value);
  } else if (strcmp(command, "D3") == 0) {
    handleStreamJog(3, value);
  } else if (strcmp(command, "D4") == 0) {
    handleStreamJog(4, value);
  } else if (strcmp(command, "D5") == 0) {
    handleStreamJog(5, value);
  } else if (strcmp(command, "D6") == 0) {
    handleStreamJog(6, value);
  } else if (strcmp(command, "DX") == 0) {
    stepperX.move((long)value);
  } else if (strcmp(command, "DY") == 0) {
    stepperY.move((long)value);
  } else if (strcmp(command, "A4") == 0) {
    float v = constrain(value, LIMIT_J4_MIN, LIMIT_J4_MAX);
    moveToAngles(commandedAngle1, commandedAngle2, commandedAngle3, 1000, false, v, commandedAngle5, false, commandedAngle6);
  } else if (strcmp(command, "A5") == 0) {
    float v = constrain(value, LIMIT_J5_MIN, LIMIT_J5_MAX);
    moveToAngles(commandedAngle1, commandedAngle2, commandedAngle3, 1000, false, commandedAngle4, v, false, commandedAngle6);
  } else if (strcmp(command, "A6") == 0) {
    float v = constrain(value, LIMIT_J6_MIN, LIMIT_J6_MAX);
    moveToAngles(commandedAngle1, commandedAngle2, commandedAngle3, 1000, false, commandedAngle4, commandedAngle5, false, v);
  } else if (strcmp(command, "BACKLASH") == 0 || strncmp(command, "BACKLASH ", 9) == 0) {
    if (strncmp(command, "BACKLASH ", 9) == 0) {
      parseBacklashCommand(command + 9);
    } else {
      printBacklash();
    }
  } else if (strcmp(command, "MS") == 0) {
    stepperX.setMaxSpeed(value);
    stepperY.setMaxSpeed(value);
  } else if (strcmp(command, "HOMEJ1") == 0) {
    startHoming(true, false, false);
  } else if (strcmp(command, "HOMEJ2") == 0) {
    startHoming(false, true, false);
  } else if (strcmp(command, "HOMEJ3") == 0) {
    startHoming(false, false, true);
  } else if (strcmp(command, "HOME") == 0) {
    startHoming(true, true, true);  // bare HOME = SWITCH (non-blocking)
  } else if (strncmp(command, "HOME ", 5) == 0) {
    const char *method = command + 5;
    while (*method == ' ') method++;
    if (strcmp(method, "SWITCH") == 0) {
      startHoming(true, true, true);  // non-blocking
    } else if (strcmp(method, "MICRO") == 0 || strcmp(method, "GANTRY") == 0) {
      homeGantry(true, true);
    } else if (strcmp(method, "ENCODER") == 0 || strcmp(method, "MANUAL") == 0) {
      // For encoder/manual, assume current position is the calibrated HOME position (0, 0, 0, 0, 0, 0)
      stepper1.setCurrentPosition(0); stepper1.moveTo(0);
      stepper2.setCurrentPosition(0); stepper2.moveTo(0);
      stepper3.setCurrentPosition(0); stepper3.moveTo(0);
      stepper4.setCurrentPosition(0); stepper4.moveTo(0);
      stepper5.setCurrentPosition(0); stepper5.moveTo(0);
      stepper6.setCurrentPosition(0); stepper6.moveTo(0);

      noInterrupts();
      commandedAngle1 = 0.0f;
      commandedAngle2 = 0.0f;
      commandedAngle3 = 0.0f;
      commandedAngle4 = 0.0f;
      commandedAngle5 = 0.0f;
      commandedAngle6 = 0.0f;
      isHomed = true;
      qHead = 0; qTail = 0; isMovingQueue = false; wasContinuous = false;
      motionLoopActive = false;
      currentVelJ1 = 0.0f; currentVelJ2 = 0.0f; currentVelJ3 = 0.0f;
      currentVelJ4 = 0.0f; currentVelJ5 = 0.0f; currentVelJ6 = 0.0f;
      currentAccJ1 = 0.0f; currentAccJ2 = 0.0f; currentAccJ3 = 0.0f;
      currentAccJ4 = 0.0f; currentAccJ5 = 0.0f; currentAccJ6 = 0.0f;
      lastSpdJ1 = 0.0f; lastSpdJ2 = 0.0f; lastSpdJ3 = 0.0f;
      lastSpdJ4 = 0.0f; lastSpdJ5 = 0.0f; lastSpdJ6 = 0.0f;
      lastAccJ1 = 0.0f; lastAccJ2 = 0.0f; lastAccJ3 = 0.0f;
      lastAccJ4 = 0.0f; lastAccJ5 = 0.0f; lastAccJ6 = 0.0f;
      for (int i = 0; i < 6; ++i) {
        backlashOffsetSteps[i] = 0.0f;
        lastDirection[i] = 0;
      }
      interrupts();

      if (strcmp(method, "ENCODER") == 0) {
        // Update encoder offsets assuming current is home
        uint16_t r1 = readAS5600(Wire2);
        if (r1 != 0xFFFF) enc1Offset = (r1 / 4096.0f) * 360.0f - commandedAngle1;
        uint16_t r2 = readAS5600(Wire);
        if (r2 != 0xFFFF) enc2Offset = (r2 / 4096.0f) * 360.0f - commandedAngle2;
        uint16_t r3 = readAS5600(Wire1);
        if (r3 != 0xFFFF) enc3Offset = (r3 / 4096.0f) * 360.0f - commandedAngle3;
      }
      applySpeedOverride();
      sendACK();
      Serial.println("HOMING: Manual/Encoder - positions set to home");
    } else {
      Serial.println("ERR: Unknown HOME method (use SWITCH, ENCODER, or MANUAL)");
    }
  } else if (strcmp(command, "CAL") == 0) {
    uint16_t r1 = readAS5600(Wire2);
    if (r1 != 0xFFFF) enc1Offset = (r1 / 4096.0f) * 360.0f - commandedAngle1;
    uint16_t r2 = readAS5600(Wire);
    if (r2 != 0xFFFF) enc2Offset = (r2 / 4096.0f) * 360.0f - commandedAngle2;
    uint16_t r3 = readAS5600(Wire1);
    if (r3 != 0xFFFF) enc3Offset = (r3 / 4096.0f) * 360.0f - commandedAngle3;

    Serial.println("CALIBRATED: Encoder Offsets synced to current commanded angles");
  } else if (strcmp(command, "DIAG") == 0) {
    Serial.println("\n=== DIAGNOSTICS ===");
    printDriver("Axis 1", driver1);
    printDriver("Axis 2", driver2);
    printDriver("Axis 3", driver3);
  } else if (strcmp(command, "ENC") == 0) {
    Serial.println("\n=== ENCODER POSITIONS ===");
    Serial.print("Encoder 1 (Base):     ");
    if (encEnabled[0]) Serial.println(readAS5600(Wire2)); else Serial.println("DISABLED");
    Serial.print("Encoder 2 (Shoulder): ");
    if (encEnabled[1]) Serial.println(readAS5600(Wire)); else Serial.println("DISABLED");
    Serial.print("Encoder 3 (Elbow):    ");
    if (encEnabled[2]) Serial.println(readAS5600(Wire1)); else Serial.println("DISABLED");
  } else if (strcmp(command, "CL_ON") == 0) {
    useEncoders = true;
    manualCL = true;
    wasContinuous = false;
    Serial.println("Closed Loop: ENABLED (manual)");
  } else if (strcmp(command, "CL_OFF") == 0) {
    if (streamMode) stopStreamMode(false);
    noInterrupts();
    qHead = qTail;
    isMovingQueue = false;
    wasContinuous = false;
    currentVelJ1 = 0.0f; currentVelJ2 = 0.0f; currentVelJ3 = 0.0f;
    currentAccJ1 = 0.0f; currentAccJ2 = 0.0f; currentAccJ3 = 0.0f;
    lastSpdJ1 = 0.0f; lastSpdJ2 = 0.0f; lastSpdJ3 = 0.0f;
    lastAccJ1 = 0.0f; lastAccJ2 = 0.0f; lastAccJ3 = 0.0f;
    syncCommandedToStepperPosition();
    pidAdj1 = 0.0f;
    pidAdj2 = 0.0f;
    pidAdj3 = 0.0f;
    useEncoders = false;
    manualCL = false;
    interrupts();
    Serial.println("Closed Loop: DISABLED");
  } else if (strcmp(command, "ENC1") == 0) {
    encEnabled[0] = (value != 0.0f);
    Serial.print("Encoder 1: ");
    Serial.println(encEnabled[0] ? "ENABLED" : "DISABLED");
  } else if (strcmp(command, "ENC2") == 0) {
    encEnabled[1] = (value != 0.0f);
    Serial.print("Encoder 2: ");
    Serial.println(encEnabled[1] ? "ENABLED" : "DISABLED");
  } else if (strcmp(command, "ENC3") == 0) {
    encEnabled[2] = (value != 0.0f);
    Serial.print("Encoder 3: ");
    Serial.println(encEnabled[2] ? "ENABLED" : "DISABLED");
  } else if (strcmp(command, "JNT1") == 0) {
    jointEnabled[0] = (value != 0.0f);
    Serial.print("Joint 1: ");
    Serial.println(jointEnabled[0] ? "ENABLED" : "DISABLED");
  } else if (strcmp(command, "JNT2") == 0) {
    jointEnabled[1] = (value != 0.0f);
    Serial.print("Joint 2: ");
    Serial.println(jointEnabled[1] ? "ENABLED" : "DISABLED");
  } else if (strcmp(command, "JNT3") == 0) {
    jointEnabled[2] = (value != 0.0f);
    Serial.print("Joint 3: ");
    Serial.println(jointEnabled[2] ? "ENABLED" : "DISABLED");
  } else if (strcmp(command, "JNT4") == 0) {
    jointEnabled[3] = (value != 0.0f);
    Serial.print("Joint 4: ");
    Serial.println(jointEnabled[3] ? "ENABLED" : "DISABLED");
  } else if (strcmp(command, "JNT5") == 0) {
    jointEnabled[4] = (value != 0.0f);
    Serial.print("Joint 5: ");
    Serial.println(jointEnabled[4] ? "ENABLED" : "DISABLED");
  } else if (strcmp(command, "JNT6") == 0) {
    jointEnabled[5] = (value != 0.0f);
    Serial.print("Joint 6: ");
    Serial.println(jointEnabled[5] ? "ENABLED" : "DISABLED");
  } else if (strcmp(command, "GEAR1") == 0) {
    if (value <= 0.0f) {
      Serial.println("ERR: Gear ratio must be > 0");
      return;
    }
    gearboxRatioAxis1 = value;
    stepsPerDegAxis1 = (STEPS_PER_MOTOR_REV * gearboxRatioAxis1) / 360.0f;
    Serial.print("Gearbox Ratio Axis 1: ");
    Serial.println(gearboxRatioAxis1, 2);
  } else if (strcmp(command, "GEAR2") == 0) {
    if (value <= 0.0f) {
      Serial.println("ERR: Gear ratio must be > 0");
      return;
    }
    gearboxRatio = value;
    stepsPerDegGeared = (STEPS_PER_MOTOR_REV * gearboxRatio) / 360.0f;
    Serial.print("Gearbox Ratio Axis 2/3: ");
    Serial.println(gearboxRatio, 2);
  } else if (strcmp(command, "TELEM") == 0) {
    useTelemetry = (value != 0.0f);
    Serial.print("Telemetry: ");
    Serial.println(useTelemetry ? "ENABLED" : "DISABLED");
  } else if (strcmp(command, "REENABLE") == 0) {
    // Resynchronize commanded positions and clear PID filters before re-energizing drivers
    syncCommandedToStepperPosition();
    noInterrupts();
    pid1.integral = 0; pid1.lastError = 0;
    pid2.integral = 0; pid2.lastError = 0;
    pid3.integral = 0; pid3.lastError = 0;
    pidAdj1 = 0.0f; pidAdj2 = 0.0f; pidAdj3 = 0.0f;
    lastSpdJ1 = 0.0f; lastSpdJ2 = 0.0f; lastSpdJ3 = 0.0f;
    lastAccJ1 = 0.0f; lastAccJ2 = 0.0f; lastAccJ3 = 0.0f;
    wasContinuous = false;
    estopActive = false;
    interrupts();

    digitalWrite(PIN_AXIS1_EN, LOW);
    digitalWrite(PIN_AXIS2_EN, LOW);
    digitalWrite(PIN_AXIS3_EN, LOW);
    digitalWrite(PIN_WRIST_EN, LOW);
    applyHoldCurrent();
    if (useEncoders && cachedEncoderValid) {
      refreshEncoderCache();
      commandedAngle1 = cachedEncAngle1;
      commandedAngle2 = cachedEncAngle2;
      commandedAngle3 = cachedEncAngle3;
      stepper1.setCurrentPosition((long)roundf(commandedAngle1 * stepsPerDegAxis1));
      stepper2.setCurrentPosition((long)roundf(commandedAngle2 * stepsPerDegGeared));
      stepper3.setCurrentPosition(j3AngleToSteps(commandedAngle3));
      isHomed = true;
      Serial.println("Positions resynchronized from absolute encoders.");
    } else {
      isHomed = false;
      Serial.println("WARN: Re-homing recommended after ESTOP.");
    }
    Serial.println("Drivers re-enabled");
  } else if (strcmp(command, "ESTOP") == 0) {
    estopActive = true;
    calTestActive = false;
    // EMERGENCY STOP: Halt all motion immediately
    if (streamMode) stopStreamMode(false);
    stepper1.stop();
    stepper2.stop();
    stepper3.stop();
    stepper4.stop();
    stepper5.stop();
    stepper6.stop();
    stepperX.stop();
    stepperY.stop();

    // Physically disable drivers (release holding torque)
    digitalWrite(PIN_AXIS1_EN, HIGH);
    digitalWrite(PIN_AXIS2_EN, HIGH);
    digitalWrite(PIN_AXIS3_EN, HIGH);
    digitalWrite(PIN_WRIST_EN, HIGH);
    digitalWrite(PIN_MICRO_EN, HIGH);

    noInterrupts();
    // Clear Queue and reset continuous spline state
    qHead = 0;
    qTail = 0;
    isMovingQueue = false;
    wasContinuous = false;
    motionLoopActive = false;
    currentVelJ1 = 0.0f; currentVelJ2 = 0.0f; currentVelJ3 = 0.0f;
    currentVelJ4 = 0.0f; currentVelJ5 = 0.0f; currentVelJ6 = 0.0f;
    currentAccJ1 = 0.0f; currentAccJ2 = 0.0f; currentAccJ3 = 0.0f;
    currentAccJ4 = 0.0f; currentAccJ5 = 0.0f; currentAccJ6 = 0.0f;
    lastSpdJ1 = 0.0f; lastSpdJ2 = 0.0f; lastSpdJ3 = 0.0f;
    lastSpdJ4 = 0.0f; lastSpdJ5 = 0.0f; lastSpdJ6 = 0.0f;
    lastAccJ1 = 0.0f; lastAccJ2 = 0.0f; lastAccJ3 = 0.0f;
    lastAccJ4 = 0.0f; lastAccJ5 = 0.0f; lastAccJ6 = 0.0f;
    motionTargetSpeed1 = 0.0f; motionTargetSpeed2 = 0.0f; motionTargetSpeed3 = 0.0f;
    motionTargetSpeed4 = 0.0f; motionTargetSpeed5 = 0.0f; motionTargetSpeed6 = 0.0f;
    pid1.integral = 0; pid2.integral = 0; pid3.integral = 0;
    pidAdj1 = 0.0f; pidAdj2 = 0.0f; pidAdj3 = 0.0f;
    interrupts();
    Serial.println("!!! EMERGENCY STOP: MOTION HALTED, DRIVERS DISABLED !!!");
  }
  // ── SPEED OVERRIDE ──────────────────────────────────────────────────────
  else if (strcmp(command, "SPEED") == 0) {
    speedOverride = constrain(value / 100.0f, 0.05f, 1.0f);
    applySpeedOverride();
    scheduleEepromSave();
    Serial.print("Speed override: ");
    Serial.print(speedOverride * 100.0f, 0);
    Serial.println("%");
  }
  // ── POSITION QUERY (AR4 compatible) ─────────────────────────────────────
  else if (strcmp(command, "RP") == 0) {
    sendRobotPos();
  }
  // ── ENCODER CHECK MANUAL TRIGGER ────────────────────────────────────────
  else if (strcmp(command, "ENCCHECK") == 0) {
    checkEncoders();
  }
  // ── SAVE CONFIG TO EEPROM ───────────────────────────────────────────────
  else if (strcmp(command, "SAVECFG") == 0) {
    eeprom_save_all();
    eepromDirty = false;
    Serial.println("Config saved to EEPROM");
  }
  // ── CALIBRATION SPEED / ACCEL TUNING ───────────────────────────────────
  else if (strcmp(command, "MAXSPD") == 0 || strncmp(command, "MAXSPD ", 7) == 0) {
    if (strncmp(command, "MAXSPD ", 7) == 0) {
      int axis = 0;
      float speed = 0.0f;
      if (sscanf(command + 7, "%d %f", &axis, &speed) == 2) {
        if (speed <= 0.0f) { Serial.println("ERR: Speed must be > 0"); return; }
        if (axis == 1) {
          MAX_SPEED_J1 = speed;
          stepper1.setMaxSpeed(MAX_SPEED_J1);
          stepper1.setAcceleration(MAX_ACCEL_J1);
          scheduleEepromSave();
          Serial.printf("J1 Max Speed set to: %.1f\n", MAX_SPEED_J1);
        } else if (axis == 2) {
          MAX_SPEED_J2 = speed;
          stepper2.setMaxSpeed(MAX_SPEED_J2);
          stepper2.setAcceleration(MAX_ACCEL_J2);
          scheduleEepromSave();
          Serial.printf("J2 Max Speed set to: %.1f\n", MAX_SPEED_J2);
        } else if (axis == 3) {
          MAX_SPEED_J3 = speed;
          stepper3.setMaxSpeed(MAX_SPEED_J3);
          stepper3.setAcceleration(MAX_ACCEL_J3);
          scheduleEepromSave();
          Serial.printf("J3 Max Speed set to: %.1f\n", MAX_SPEED_J3);
        } else if (axis == 4) {
          MAX_SPEED_J4 = speed;
          stepper4.setMaxSpeed(MAX_SPEED_J4);
          stepper4.setAcceleration(MAX_ACCEL_J4);
          scheduleEepromSave();
          Serial.printf("J4 Max Speed set to: %.1f\n", MAX_SPEED_J4);
        } else if (axis == 5) {
          MAX_SPEED_J5 = speed;
          stepper5.setMaxSpeed(MAX_SPEED_J5);
          stepper5.setAcceleration(MAX_ACCEL_J5);
          scheduleEepromSave();
          Serial.printf("J5 Max Speed set to: %.1f\n", MAX_SPEED_J5);
        } else if (axis == 6) {
          MAX_SPEED_J6 = speed;
          stepper6.setMaxSpeed(MAX_SPEED_J6);
          stepper6.setAcceleration(MAX_ACCEL_J6);
          scheduleEepromSave();
          Serial.printf("J6 Max Speed set to: %.1f\n", MAX_SPEED_J6);
        } else {
          Serial.println("ERR: Invalid axis (1-6)");
        }
      } else {
        Serial.println("ERR: Format: MAXSPD <axis> <speed>");
      }
    } else {
      Serial.println("ERR: Format: MAXSPD <axis> <speed>");
    }
  }
  else if (strcmp(command, "MAXACC") == 0 || strncmp(command, "MAXACC ", 7) == 0) {
    if (strncmp(command, "MAXACC ", 7) == 0) {
      int axis = 0;
      float accel = 0.0f;
      if (sscanf(command + 7, "%d %f", &axis, &accel) == 2) {
        if (accel <= 0.0f) { Serial.println("ERR: Accel must be > 0"); return; }
        if (axis == 1) {
          MAX_ACCEL_J1 = accel;
          MAX_JERK_J1 = MAX_ACCEL_J1 * 10.0f;
          stepper1.setAcceleration(MAX_ACCEL_J1);
          scheduleEepromSave();
          Serial.printf("J1 Max Accel set to: %.1f\n", MAX_ACCEL_J1);
        } else if (axis == 2) {
          MAX_ACCEL_J2 = accel;
          MAX_JERK_J2 = MAX_ACCEL_J2 * 10.0f;
          stepper2.setAcceleration(MAX_ACCEL_J2);
          scheduleEepromSave();
          Serial.printf("J2 Max Accel set to: %.1f\n", MAX_ACCEL_J2);
        } else if (axis == 3) {
          MAX_ACCEL_J3 = accel;
          MAX_JERK_J3 = MAX_ACCEL_J3 * 10.0f;
          stepper3.setAcceleration(MAX_ACCEL_J3);
          scheduleEepromSave();
          Serial.printf("J3 Max Accel set to: %.1f\n", MAX_ACCEL_J3);
        } else if (axis == 4) {
          MAX_ACCEL_J4 = accel;
          MAX_JERK_J4 = MAX_ACCEL_J4 * 10.0f;
          stepper4.setAcceleration(MAX_ACCEL_J4);
          scheduleEepromSave();
          Serial.printf("J4 Max Accel set to: %.1f\n", MAX_ACCEL_J4);
        } else if (axis == 5) {
          MAX_ACCEL_J5 = accel;
          MAX_JERK_J5 = MAX_ACCEL_J5 * 10.0f;
          stepper5.setAcceleration(MAX_ACCEL_J5);
          scheduleEepromSave();
          Serial.printf("J5 Max Accel set to: %.1f\n", MAX_ACCEL_J5);
        } else if (axis == 6) {
          MAX_ACCEL_J6 = accel;
          MAX_JERK_J6 = MAX_ACCEL_J6 * 10.0f;
          stepper6.setAcceleration(MAX_ACCEL_J6);
          scheduleEepromSave();
          Serial.printf("J6 Max Accel set to: %.1f\n", MAX_ACCEL_J6);
        } else {
          Serial.println("ERR: Invalid axis (1-6)");
        }
      } else {
        Serial.println("ERR: Format: MAXACC <axis> <accel>");
      }
    } else {
      Serial.println("ERR: Format: MAXACC <axis> <accel>");
    }
  }
  else if (strcmp(command, "SAFETY") == 0) {
    if (value >= 1.0f) {
      SPLINE_SAFETY_MARGIN = value;
      scheduleEepromSave();
      Serial.printf("Spline safety margin set to: %.2f\n", SPLINE_SAFETY_MARGIN);
    } else {
      Serial.println("ERR: Safety margin must be >= 1.0");
    }
  }
  else if (strcmp(command, "CALDUMP") == 0) {
    Serial.println("\n=== CALIBRATION CONFIG ===");
    Serial.printf("MAXSPD J1 (Base):     %.1f steps/s\n", MAX_SPEED_J1);
    Serial.printf("MAXSPD J2 (Shoulder): %.1f steps/s\n", MAX_SPEED_J2);
    Serial.printf("MAXSPD J3 (Elbow):    %.1f steps/s\n", MAX_SPEED_J3);
    Serial.printf("MAXACC J1 (Base):     %.1f steps/s^2\n", MAX_ACCEL_J1);
    Serial.printf("MAXACC J2 (Shoulder): %.1f steps/s^2\n", MAX_ACCEL_J2);
    Serial.printf("MAXACC J3 (Elbow):    %.1f steps/s^2\n", MAX_ACCEL_J3);
    Serial.printf("SPLINE_SAFETY_MARGIN: %.2f\n", SPLINE_SAFETY_MARGIN);
    Serial.println("==========================");
  }
  else if (strcmp(command, "CALTEST") == 0) {
    Serial.println("\n--- CALIBRATION TEST INITIATED ---");
    Serial.println("Starting round-trip sweep to detect step loss.");
    Serial.println("Sequence:");
    Serial.println("  1. J1: 0 -> 80 -> -80 -> 0");
    Serial.println("  2. J2: -90 -> -10 -> -90");
    Serial.println("  3. J3: 90 -> 10 -> 90");
    Serial.println("Please inspect the arm visually. Each joint must return exactly to home.");

    noInterrupts();
    qHead = 0; qTail = 0; isMovingQueue = false; wasContinuous = false;
    currentVelJ1 = 0.0f; currentVelJ2 = 0.0f; currentVelJ3 = 0.0f;
    currentAccJ1 = 0.0f; currentAccJ2 = 0.0f; currentAccJ3 = 0.0f;
    interrupts();

    calTestActive = true;
    calTestStartEnc1 = encEnabled[0] ? getEncoderAngle(readAS5600(Wire2), enc1Offset) : CAL_ENC_NO_DATA;
    calTestStartEnc2 = encEnabled[1] ? getEncoderAngle(readAS5600(Wire),  enc2Offset) : CAL_ENC_NO_DATA;
    calTestStartEnc3 = encEnabled[2] ? getEncoderAngle(readAS5600(Wire1), enc3Offset) : CAL_ENC_NO_DATA;

    queueCalibrationWaypoint(80.0f, -90.0f, 90.0f, 2500);
    queueCalibrationWaypoint(-80.0f, -90.0f, 90.0f, 4000);
    queueCalibrationWaypoint(0.0f, -90.0f, 90.0f, 2500);

    queueCalibrationWaypoint(0.0f, -10.0f, 90.0f, 2500);
    queueCalibrationWaypoint(0.0f, -90.0f, 90.0f, 2500);

    queueCalibrationWaypoint(0.0f, -90.0f, 10.0f, 2500);
    queueCalibrationWaypoint(0.0f, -90.0f, 90.0f, 2500);
  }
  else if (strcmp(command, "JTELEM") == 0) {
    useJsonTelemetry = (value != 0.0f);
    Serial.print("JSON Telemetry: ");
    Serial.println(useJsonTelemetry ? "ENABLED" : "DISABLED");
  }
  else if (strcmp(command, "STOP") == 0) {
    calTestActive = false;
    if (streamMode) stopStreamMode(false);
    noInterrupts();
    qHead = qTail;
    isMovingQueue = false;
    isTeachPlaying = false;
    wasContinuous = false;
    motionLoopActive = false;
    currentVelJ1 = 0.0f; currentVelJ2 = 0.0f; currentVelJ3 = 0.0f;
    currentVelJ4 = 0.0f; currentVelJ5 = 0.0f; currentVelJ6 = 0.0f;
    currentAccJ1 = 0.0f; currentAccJ2 = 0.0f; currentAccJ3 = 0.0f;
    currentAccJ4 = 0.0f; currentAccJ5 = 0.0f; currentAccJ6 = 0.0f;
    lastSpdJ1 = 0.0f; lastSpdJ2 = 0.0f; lastSpdJ3 = 0.0f;
    lastSpdJ4 = 0.0f; lastSpdJ5 = 0.0f; lastSpdJ6 = 0.0f;
    lastAccJ1 = 0.0f; lastAccJ2 = 0.0f; lastAccJ3 = 0.0f;
    lastAccJ4 = 0.0f; lastAccJ5 = 0.0f; lastAccJ6 = 0.0f;
    motionTargetSpeed1 = 0.0f; motionTargetSpeed2 = 0.0f; motionTargetSpeed3 = 0.0f;
    motionTargetSpeed4 = 0.0f; motionTargetSpeed5 = 0.0f; motionTargetSpeed6 = 0.0f;
    interrupts();
    Serial.println("Motion stopped (queue cleared)");
    sendACK();
  }
  else if (strcmp(command, "STATUS") == 0) {
    sendJSONTelemetry();
  }
  else if (strcmp(command, "GRIP") == 0) {
    setGripper(value);
    Serial.print("Gripper: "); Serial.println((int)gripperAngle);
  }
  else if (strcmp(command, "TOOLX") == 0) { toolOffset.x = value; scheduleEepromSave(); Serial.print("Tool X: "); Serial.println(value); }
  else if (strcmp(command, "TOOLY") == 0) { toolOffset.y = value; scheduleEepromSave(); Serial.print("Tool Y: "); Serial.println(value); }
  else if (strcmp(command, "TOOLZ") == 0) { toolOffset.z = value; scheduleEepromSave(); Serial.print("Tool Z: "); Serial.println(value); }
  else if (strcmp(command, "WORKX") == 0) { workOffset.x = value; scheduleEepromSave(); Serial.print("Work X: "); Serial.println(value); }
  else if (strcmp(command, "WORKY") == 0) { workOffset.y = value; scheduleEepromSave(); Serial.print("Work Y: "); Serial.println(value); }
  else if (strcmp(command, "WORKZ") == 0) { workOffset.z = value; scheduleEepromSave(); Serial.print("Work Z: "); Serial.println(value); }
  else if (strcmp(command, "TEACH") == 0) {
    int mode = (int)value;
    if (mode == 0) {
      isTeachMode = true;
      teachCount = 0;
      teachPlayIdx = 0;
      Serial.println("TEACH: Recording started. Send TEACH 1 to record each point.");
    } else if (mode == 1) {
      if (!isTeachMode) { Serial.println("WARN: Not in teach mode (send TEACH 0 first)"); return; }
      if (teachCount >= MAX_TEACH_POINTS) { Serial.println("WARN: Teach buffer full"); return; }
      teachPoints[teachCount++] = { commandedAngle1, commandedAngle2, commandedAngle3,
                                    commandedAngle4, commandedAngle5, commandedAngle6, gripperAngle };
      Serial.print("TEACH: Point "); Serial.print(teachCount); Serial.println(" recorded");
    } else if (mode == 2) {
      isTeachMode = false;
      isTeachPlaying = false;
      scheduleEepromSave();
      Serial.print("TEACH: "); Serial.print(teachCount); Serial.println(" points stored (saved to EEPROM)");
    } else if (mode == 3) {
      if (teachCount == 0) { Serial.println("WARN: No teach points"); return; }
      isTeachPlaying = true;
      teachPlayIdx = 0;
      Serial.println("TEACH: Playback started");
    } else if (mode == 4) {
      teachCount = 0;
      teachPlayIdx = 0;
      isTeachMode = false;
      isTeachPlaying = false;
      scheduleEepromSave();
      Serial.println("TEACH: Buffer cleared");
    }
  }
  else if (strcmp(command, "VER") == 0) {
    Serial.printf("VER %s %s %s\n", FIRMWARE_VERSION, BUILD_DATE, BUILD_TIME);
  }
}

// ================= DRIVER DIAGNOSTICS =================
void printDriver(const char *name, TMC2209Stepper &driver) {
  Serial.print("--- ");
  Serial.print(name);
  Serial.print(" --- ");
  uint8_t ifcnt = driver.IFCNT();

  if (ifcnt > 0) {
    uint32_t status = driver.DRV_STATUS();
    Serial.println("Connection: OK");
    Serial.printf("  [INFO] Microsteps: %d, CS_ACTUAL: %d\n", driver.microsteps(), driver.cs_actual());
    if (status & 0x01)       Serial.println("  [WARN] Over-Temperature Pre-Warning!");
    if (status & 0x02)       Serial.println("  [ERR]  Over-Temperature SHUTDOWN!");
    if (status & 0x04)       Serial.println("  [ERR]  Short to Ground (Phase A)!");
    if (status & 0x08)       Serial.println("  [ERR]  Short to Ground (Phase B)!");
    if (status & 0x10)       Serial.println("  [ERR]  Short to Supply (Phase A)!");
    if (status & 0x20)       Serial.println("  [ERR]  Short to Supply (Phase B)!");
    if (status & 0x40)       Serial.println("  [WARN] Open Load (Phase A) - Disconnected?");
    if (status & 0x80)       Serial.println("  [WARN] Open Load (Phase B) - Disconnected?");
    if (status & 0x80000000) Serial.println("  [INFO] Standstill Indicator");
  } else {
    Serial.println("FAIL (No UART Response)");
  }
}

// ================= HARDWARE I2C FOR AS5600 =================
uint16_t readAS5600(TwoWire &wire) {
  const uint8_t AS5600_ADDR = 0x36;
  const uint8_t AS5600_ANGLE_H = 0x0C;

  wire.beginTransmission(AS5600_ADDR);
  wire.write(AS5600_ANGLE_H);
  if (wire.endTransmission(false) != 0)
    return 0xFFFF; // no ACK

  if (wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2) != 2)
    return 0xFFFF;

  uint8_t hi = wire.read();
  uint8_t lo = wire.read();
  return ((uint16_t)hi << 8) | lo;
}

uint16_t readEncoderRetry(TwoWire &wire, uint8_t maxAttempts) {
  for (uint8_t i = 0; i < maxAttempts; ++i) {
    uint16_t val = readAS5600(wire);
    if (val != 0xFFFF) return val;
    delayMicroseconds(50);
  }
  return 0xFFFF;
}

// ================= INPUT PROCESSING (PC + USB) =================

// Helper to parse Cartesian coordinates, wrist/orientation, and duration params without dynamic allocation
static bool parseMovParams(const char *str, float &x, float &y, float &z,
                           WristMode &wMode, float &j4m, float &j5m, unsigned long &dur, bool isMovL, float &j6m,
                           bool &hasOrientation, float &roll, float &pitch, float &yaw) {
  const char *pX = strchr(str, 'X');
  const char *pY = strchr(str, 'Y');
  const char *pZ = strchr(str, 'Z');
  if (!pX || !pY || !pZ) return false;

  x = strtof(pX + 1, NULL);
  y = strtof(pY + 1, NULL);
  z = strtof(pZ + 1, NULL);
  if (isnan(x) || isinf(x) || isnan(y) || isinf(y) || isnan(z) || isinf(z)) return false;

  wMode = WristMode::LEVEL;
  j4m = 0.0f;
  j5m = 0.0f;
  j6m = 0.0f;
  hasOrientation = false;
  roll = 0.0f;
  pitch = 0.0f;
  yaw = 0.0f;
  dur = isMovL ? 2000UL : 1000UL;

  // Check for R (Roll), P (Pitch), YW (Yaw) orientation parameters
  const char *pR = strchr(str, 'R');
  if (pR && (pR == str || *(pR - 1) == ' ' || *(pR - 1) == '\t')) {
    roll = strtof(pR + 1, NULL);
    hasOrientation = true;
  }
  const char *pP = strchr(str, 'P');
  if (pP && (pP == str || *(pP - 1) == ' ' || *(pP - 1) == '\t')) {
    pitch = strtof(pP + 1, NULL);
    hasOrientation = true;
  }
  const char *pYW = strstr(str, "YW");
  if (!pYW) pYW = strstr(str, "YAW");
  if (pYW) {
    yaw = strtof(pYW + (strncmp(pYW, "YAW", 3) == 0 ? 3 : 2), NULL);
    hasOrientation = true;
  }

  const char *pW = strchr(str, 'W');
  if (pW && (pW == str || *(pW - 1) == ' ' || *(pW - 1) == '\t')) {
    int wVal = (int)strtof(pW + 1, NULL);
    wMode = static_cast<WristMode>(constrain(wVal, 0, 6));
  }

  const char *pJ4 = strstr(str, "J4");
  if (pJ4) {
    j4m = strtof(pJ4 + 2, NULL);
  }

  const char *pJ5 = strstr(str, "J5");
  if (pJ5) {
    j5m = strtof(pJ5 + 2, NULL);
  }

  const char *pJ6 = strstr(str, "J6");
  if (pJ6) {
    j6m = strtof(pJ6 + 2, NULL);
  }

  const char *pD = str;
  while ((pD = strchr(pD, 'D')) != NULL) {
    if (pD > str && (*(pD - 1) == ' ' || *(pD - 1) == '\t')) {
      break;
    }
    pD++;
  }
  if (pD) {
    long dVal = strtol(pD + 1, NULL, 10);
    if (dVal > 0) dur = (unsigned long)dVal;
  }

  return true;
}

void processInput(Stream &stream, char *buffer, uint8_t &pos, size_t bufSize) {
  while (stream.available() > 0) {
    char c = stream.read();

    // Packet Start
    if (c == '<') {
      pos = 0;
      buffer[pos] = '\0';
      continue;
    }
    // Packet End
    else if (c == '>') {
      buffer[pos] = '\0';
      parsePacket((const char*)buffer);
      pos = 0;
      continue;
    }
    // Newline (Legacy/IK Commands)
    else if (c == '\n') {
      buffer[pos] = '\0';
      char *p = buffer;
      while (*p == ' ' || *p == '\r' || *p == '\t') p++;
      char *end = p + strlen(p) - 1;
      while (end >= p && (*end == ' ' || *end == '\r' || *end == '\t')) {
        *end = '\0';
        end--;
      }

      if (*p != '\0') {
        if (strchr(p, ',') != NULL) {
          Serial.println("ERR: CSV data received without framing '<...>' — use packet format");
        } else {
          // Convert command token to uppercase in-place
          for (char *cptr = p; *cptr != '\0'; cptr++) {
            if (*cptr >= 'a' && *cptr <= 'z') *cptr = (char)(*cptr - 'a' + 'A');
          }

          if (estopActive) {
            if (strncmp(p, "REENABLE", 8) != 0 && strncmp(p, "ESTOP", 5) != 0 &&
                strncmp(p, "STATUS", 6) != 0 && strncmp(p, "POS", 3) != 0 &&
                strncmp(p, "CALDUMP", 7) != 0 && strncmp(p, "VER", 3) != 0 &&
                strncmp(p, "GET", 3) != 0) {
              Serial.println("ERR: Motion rejected - ESTOP is active. Send REENABLE first.");
              pos = 0;
              continue;
            }
          }

          if (strncmp(p, "SAVE ", 5) == 0 || strncmp(p, "LOAD ", 5) == 0 || strncmp(p, "RUN ", 4) == 0) {
            Serial.println("ERR: SD support disabled");
          } else if (strncmp(p, "GRIP", 4) == 0) {
            if (strcmp(p, "GRIP OPEN") == 0) {
              setGripper(0.0f);
            } else if (strcmp(p, "GRIP CLOSE") == 0) {
              setGripper(180.0f);
            } else if (strlen(p) > 5) {
              setGripper(strtof(p + 5, NULL));
            }
            Serial.print("Gripper: "); Serial.println((int)gripperAngle);
          } else if (strncmp(p, "TEACH", 5) == 0) {
            const char *arg = p + 5;
            while (*arg == ' ') arg++;
            if (strcmp(arg, "START") == 0) parseCommand("TEACH", 0.0f);
            else if (strcmp(arg, "RECORD") == 0) parseCommand("TEACH", 1.0f);
            else if (strcmp(arg, "STOP") == 0) parseCommand("TEACH", 2.0f);
            else if (strcmp(arg, "PLAY") == 0) parseCommand("TEACH", 3.0f);
            else if (strcmp(arg, "CLEAR") == 0) parseCommand("TEACH", 4.0f);
            else if (*arg != '\0' && (*arg >= '0' && *arg <= '9')) {
              parseCommand("TEACH", strtof(arg, NULL));
            } else {
              Serial.println("ERR: TEACH [START|RECORD|STOP|PLAY|CLEAR] or TEACH <0-4>");
            }
          } else if (strncmp(p, "MOVL", 4) == 0) {
            float x = 0.0f, y = 0.0f, z = 0.0f, j4m = 0.0f, j5m = 0.0f, j6m = 0.0f;
            WristMode wMode = WristMode::LEVEL;
            unsigned long dur = 2000UL;
            bool hasOrientation = false;
            float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
            if (parseMovParams(p, x, y, z, wMode, j4m, j5m, dur, true, j6m, hasOrientation, roll, pitch, yaw)) {
              if (hasOrientation) {
                movL6DOF(x, y, z, roll, pitch, yaw, dur);
              } else {
                movL(x, y, z, dur, wMode, j4m, j5m, j6m);
              }
            } else {
              Serial.println("ERR: Invalid MOVL format (Expect MOVL X# Y# Z#)");
            }
          } else if (strncmp(p, "MOV", 3) == 0 && (p[3] == ' ' || p[3] == '\0' || p[3] == 'X')) {
            float x = 0.0f, y = 0.0f, z = 0.0f, j4m = 0.0f, j5m = 0.0f, j6m = 0.0f;
            WristMode wMode = WristMode::LEVEL;
            unsigned long dur = 1000UL;
            bool hasOrientation = false;
            float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
            if (parseMovParams(p, x, y, z, wMode, j4m, j5m, dur, false, j6m, hasOrientation, roll, pitch, yaw)) {
              if (hasOrientation) {
                calculateIK6DOF(x, y, z, roll, pitch, yaw, dur, false);
              } else {
                calculateIK(x, y, z, dur, wMode, j4m, j5m, false, j6m);
              }
            } else {
              Serial.println("ERR: Invalid MOV format (Expect MOV X# Y# Z#)");
            }
          } else if (strncmp(p, "HOME ", 5) == 0 || strncmp(p, "MAXSPD ", 7) == 0 || strncmp(p, "MAXACC ", 7) == 0) {
            parseCommand(p, 0.0f);
          } else {
            char *sp = strchr(p, ' ');
            if (sp) {
              *sp = '\0';
              float val = strtof(sp + 1, NULL);
              parseCommand(p, val);
            } else {
              parseCommand(p, 0.0f);
            }
          }
        }
      }
      pos = 0;
    }
    // Character
    else if (c != '\r') {
      if (pos < bufSize - 1u) {
        buffer[pos++] = c;
      } else {
        Serial.println("WARN: Input buffer overflow - command discarded");
        pos = 0;
        buffer[0] = '\0';
      }
    }
  }
}

// Parse Packet: "j1,j2,j3,j4,j5,dx,dy,duration"

// CRC-8/Dallas (Maxim) — polynomial 0x31 reflected (0x8C), initial 0x00
uint8_t crc8_maxim(const uint8_t *data, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; i++) {
    uint8_t inbyte = data[i];
    for (uint8_t j = 0; j < 8; j++) {
      uint8_t mix = (crc ^ inbyte) & 0x01;
      crc >>= 1;
      if (mix) crc ^= 0x8C;
      inbyte >>= 1;
    }
  }
  return crc;
}

void parsePacket(const char* data) {
  if (estopActive) return;

  const char* lastComma = strrchr(data, ',');
  if (!lastComma) {
    Serial.println("ERR: Malformed Packet (No CRC)");
    return;
  }

  // Reject empty CRC string (e.g. "<...,>")
  if (*(lastComma + 1) == '\0') {
    Serial.println("ERR: Malformed Packet (Empty CRC)");
    return;
  }

  // Validate CRC field format and range
  char *crcEndPtr = NULL;
  unsigned long parsedCRC = strtoul(lastComma + 1, &crcEndPtr, 10);
  if (crcEndPtr == lastComma + 1 || *crcEndPtr != '\0') {
    Serial.println("ERR: Malformed Packet (Invalid CRC format)");
    return;
  }
  if (parsedCRC > 255) {
    Serial.println("ERR: CRC out of range");
    return;
  }
  uint8_t receivedCRC = (uint8_t)parsedCRC;

  // Compute CRC over the payload (everything before last comma)
  size_t payloadLen = (size_t)(lastComma - data);
  uint8_t calculatedCRC = crc8_maxim((const uint8_t*)data, payloadLen);

  if (calculatedCRC != receivedCRC) {
    Serial.print("ERR: CRC Mismatch (Calc:");
    Serial.print(calculatedCRC);
    Serial.print(" Recv:");
    Serial.print(receivedCRC);
    Serial.println(")");
    return;
  }

  Serial.print("PKT (CRC OK): ");
  Serial.println(data);

  // Copy payload into local buffer so we can tokenize without mutating caller
  char payload[128];
  if (payloadLen >= sizeof(payload)) {
    Serial.println("ERR: Packet too long");
    return;
  }
  memcpy(payload, data, payloadLen);
  payload[payloadLen] = '\0';

  float values[10]; // j1, j2, j3, j4, j5, [j6], dx, dy, duration
  int idx = 0;
  char *cursor = payload;

  while (*cursor != '\0') {
    if (idx >= 10) {
      Serial.println("ERR: Bad Packet format (Too many values)");
      return;
    }

    char *nextComma = strchr(cursor, ',');
    if (nextComma != NULL) {
      *nextComma = '\0';
    }

    if (*cursor == '\0') {
      Serial.println("ERR: Empty field in packet");
      return;
    }

    char *endptr = NULL;
    float v = strtof(cursor, &endptr);
    if (endptr == cursor || (*endptr != '\0' && *endptr != ' ' && *endptr != '\t') || isnan(v) || isinf(v)) {
      Serial.println("ERR: Invalid numeric value in packet");
      return;
    }

    values[idx++] = v;

    if (nextComma == NULL) {
      break;
    }
    cursor = nextComma + 1;
  }

  float angle1 = 0, angle2 = 0, angle3 = 0, j4 = 0, j5 = 0, j6 = 0;
  float dx = 0, dy = 0;
  unsigned long duration = 1000;

  if (idx == 9) {
    // 6-DOF standard format: <j1,j2,j3,j4,j5,j6,dx,dy,dur,crc>
    angle1 = constrain(values[0], LIMIT_J1_MIN, LIMIT_J1_MAX);
    angle2 = constrain(values[1], LIMIT_J2_MIN, LIMIT_J2_MAX);
    angle3 = constrain(values[2], LIMIT_J3_MIN, LIMIT_J3_MAX);
    j4     = constrain(values[3], LIMIT_J4_MIN, LIMIT_J4_MAX);
    j5     = constrain(values[4], LIMIT_J5_MIN, LIMIT_J5_MAX);
    j6     = constrain(values[5], LIMIT_J6_MIN, LIMIT_J6_MAX);
    dx     = values[6];
    dy     = values[7];
    duration = (unsigned long)values[8];
  } else if (idx == 8) {
    // Legacy 5-axis packet compatibility format: <j1,j2,j3,j4,j5,dx,dy,dur,crc>
    angle1 = constrain(values[0], LIMIT_J1_MIN, LIMIT_J1_MAX);
    angle2 = constrain(values[1], LIMIT_J2_MIN, LIMIT_J2_MAX);
    angle3 = constrain(values[2], LIMIT_J3_MIN, LIMIT_J3_MAX);
    j4     = constrain(values[3], LIMIT_J4_MIN, LIMIT_J4_MAX);
    j5     = constrain(values[4], LIMIT_J5_MIN, LIMIT_J5_MAX);
    j6     = 0.0f;
    dx     = values[5];
    dy     = values[6];
    duration = (unsigned long)values[7];
  } else {
    Serial.println("ERR: Bad Packet format (Expect 8 or 9 values)");
    return;
  }

  moveToAngles(angle1, angle2, angle3, duration, false, j4, j5, false, j6);
  values[5] = dx;
  values[6] = dy;

  // Micro Manipulator (Relative/Incremental)
  if ((long)values[5] != 0) {
    stepperX.move((long)values[5]);
  }
  if ((long)values[6] != 0) {
    stepperY.move((long)values[6]);
  }
}

// ================= CLOSED LOOP CONTROL =================
// Helper to get normalized angle from encoder
// OPTIMIZED: Swapped iterative while loop for modular arithmetic (fmodf)
float getEncoderAngle(uint16_t raw, float offset) {
  // Calculate the absolute reading in degrees
  float rawAngle = (raw / 4096.0f) * 360.0f;
  float delta = rawAngle - offset;

  // Normalize angle to the range [-180, +180) using fmodf for O(1) performance.
  delta = fmodf(delta + 180.0f, 360.0f);
  if (delta < 0.0f) delta += 360.0f;
  return delta - 180.0f;
}

void refreshEncoderCache() {
  if (!useEncoders) {
    cachedEncoderValid = false;
    return;
  }

  uint16_t enc1 = encEnabled[0] ? readAS5600(Wire2) : 0xFFFF;
  uint16_t enc2 = encEnabled[1] ? readAS5600(Wire) : 0xFFFF;
  uint16_t enc3 = encEnabled[2] ? readAS5600(Wire1) : 0xFFFF;

  lastEnc2 = enc2;
  lastEnc3 = enc3;

  if (encEnabled[0] && enc1 != 0xFFFF) {
    cachedEncAngle1 = getEncoderAngle(enc1, enc1Offset);
  } else {
    cachedEncAngle1 = commandedAngle1;
  }

  if (encEnabled[1] && enc2 != 0xFFFF) {
    cachedEncAngle2 = getEncoderAngle(enc2, enc2Offset);
  } else {
    cachedEncAngle2 = commandedAngle2;
  }

  if (encEnabled[2] && enc3 != 0xFFFF) {
    cachedEncAngle3 = getEncoderAngle(enc3, enc3Offset);
  } else {
    cachedEncAngle3 = commandedAngle3;
  }

  cachedEncoderValid = true;
}

void updateClosedLoop() {
  if (!useEncoders || estopActive)
    return;

  static unsigned long lastLoopUs = 0;
  static float prevCommanded1 = 0.0f;
  static float prevCommanded2 = 0.0f;
  static float prevCommanded3 = 0.0f;

  unsigned long nowUs = micros();
  if (lastLoopUs == 0) {
    lastLoopUs = nowUs;
    return;
  }

  float dt = (nowUs - lastLoopUs) / 1000000.0f;
  lastLoopUs = nowUs;
  if (dt < 0.0005f) {
    dt = 0.0005f;
  }

  if (!cachedEncoderValid) {
    cachedEncAngle1 = commandedAngle1;
    cachedEncAngle2 = commandedAngle2;
    cachedEncAngle3 = commandedAngle3;
    cachedEncoderValid = true;
  }

  auto correctAxis = [&](float actualAngle, float commandedAngle, float prevCommandedAngle,
                         PIDController &pid, int axisIndex) -> float {
    float error = commandedAngle - actualAngle;
    error = fmodf(error + 180.0f, 360.0f);
    if (error < 0.0f) error += 360.0f;
    error -= 180.0f;

    float targetVelocity = (commandedAngle - prevCommandedAngle) / dt;
    float adjustment = pid.calculate(error, dt, targetVelocity);

    float gravityComp = 0.0f;
    if (axisIndex == 2) {
      // Joint 2 (Shoulder): torque proportional to cos(theta2)
      gravityComp = pid.kG * cosf(commandedAngle * DEG_TO_RAD);
    } else if (axisIndex == 3) {
      // Joint 3 (Elbow): torque proportional to physical world link angle cos(theta2 + theta3)
      gravityComp = pid.kG * cosf((commandedAngle2 + commandedAngle) * DEG_TO_RAD);
    }

    return adjustment + gravityComp;
  };

  pidAdj1 = correctAxis(cachedEncAngle1, commandedAngle1, prevCommanded1, pid1, 1);
  pidAdj2 = correctAxis(cachedEncAngle2, commandedAngle2, prevCommanded2, pid2, 2);
  pidAdj3 = correctAxis(cachedEncAngle3, commandedAngle3, prevCommanded3, pid3, 3);

  prevCommanded1 = commandedAngle1;
  prevCommanded2 = commandedAngle2;
  prevCommanded3 = commandedAngle3;
}

// ================= CALIBRATION TEST HELPERS =================
void queueCalibrationWaypoint(float t1, float t2, float t3, unsigned long duration_ms) {
  noInterrupts();
  uint8_t nextHead = (qHead + 1) % WAYPOINT_QUEUE_SIZE;
  if (nextHead == qTail) {
    interrupts();
    Serial.println("WARN: Waypoint queue full");
    return;
  }
  float t4 = commandedAngle4;
  float t5 = commandedAngle5;
  float t6 = commandedAngle6;
  motionQueue[qHead] = { t1, t2, t3, t4, t5, t6, duration_ms };
  qHead = nextHead;
  motionLoopActive = true;
  interrupts();
}

void checkCalTestResults() {
  Serial.println("\n--- CALIBRATION TEST COMPLETE ---");
  
  bool anyEnc = false;
  if (encEnabled[0] && calTestStartEnc1 != CAL_ENC_NO_DATA) {
    anyEnc = true;
    float end1 = getEncoderAngle(readAS5600(Wire2), enc1Offset);
    float drift = fabsf(end1 - calTestStartEnc1);
    Serial.printf("J1 (Base) Start: %.2f, End: %.2f, Drift: %.2f deg\n", calTestStartEnc1, end1, drift);
    if (drift > 1.0f) {
      Serial.println("  [WARN] J1 drift detected! Consider reducing MAX_SPEED_J1 or MAX_ACCEL_J1.");
    } else {
      Serial.println("  [OK] J1 returned cleanly.");
    }
  }
  
  if (encEnabled[1] && calTestStartEnc2 != CAL_ENC_NO_DATA) {
    anyEnc = true;
    float end2 = getEncoderAngle(readAS5600(Wire), enc2Offset);
    float drift = fabsf(end2 - calTestStartEnc2);
    Serial.printf("J2 (Shoulder) Start: %.2f, End: %.2f, Drift: %.2f deg\n", calTestStartEnc2, end2, drift);
    if (drift > 1.0f) {
      Serial.println("  [WARN] J2 drift detected! Consider reducing MAX_SPEED_J2 or MAX_ACCEL_J2.");
    } else {
      Serial.println("  [OK] J2 returned cleanly.");
    }
  }

  if (encEnabled[2] && calTestStartEnc3 != CAL_ENC_NO_DATA) {
    anyEnc = true;
    float end3 = getEncoderAngle(readAS5600(Wire1), enc3Offset);
    float drift = fabsf(end3 - calTestStartEnc3);
    Serial.printf("J3 (Elbow) Start: %.2f, End: %.2f, Drift: %.2f deg\n", calTestStartEnc3, end3, drift);
    if (drift > 1.0f) {
      Serial.println("  [WARN] J3 drift detected! Consider reducing MAX_SPEED_J3 or MAX_ACCEL_J3.");
    } else {
      Serial.println("  [OK] J3 returned cleanly.");
    }
  }
  
  if (!anyEnc) {
    Serial.println("[INFO] No encoders were active during the test.");
    Serial.println("Please visually verify if the arm returned precisely to its starting home position.");
  }
  Serial.println("---------------------------------");
}

