/*
 * Master Controller - Arduino Pro Micro
 *
 * Purpose:
 * Reads 5 Potentiometers and sends their values via USB Serial.
 * This device acts as a USB Peripheral that the Teensy 4.1 (Host) will read.
 *
 * WIRING:
 * - Pot 1 (Base J1):     Pin A0
 * - Pot 2 (Shoulder J2): Pin A1
 * - Pot 3 (Elbow J3):    Pin A2
 * - Pot 4 (Axis J4):     Pin A3
 * - Pot 5 (Axis J5):     Pin 10 (A10)
 *
 * OUTPUT FORMAT:
 * <J1,J2,J3,J4,J5>
 */

const int PIN_POT1 = A0;
const int PIN_POT2 = A1;
const int PIN_POT3 = A2;
const int PIN_POT4 = A3;
const int PIN_POT5 = 10; // A10
const int PIN_JOY_X = 4; // A6
const int PIN_JOY_Y = 6; // A7

// Smoothing (Simple Moving Average)
const int NUM_SENSORS = 7;
const int NUM_READINGS = 10;
int readIndex = 0;
int readings[NUM_SENSORS][NUM_READINGS];
int total[NUM_SENSORS];
int average[NUM_SENSORS];

void setup() {
  Serial.begin(115200); // USB Serial

  // Initialize smoothing arrays
  for (int i = 0; i < NUM_SENSORS; i++) {
    total[i] = 0;
    average[i] = 0;
    for (int k = 0; k < NUM_READINGS; k++) {
      readings[i][k] = 0;
    }
  }
}

void loop() {
  // Read Raw Values
  int raw[NUM_SENSORS];
  raw[0] = analogRead(PIN_POT1);
  raw[1] = analogRead(PIN_POT2);
  raw[2] = analogRead(PIN_POT3);
  raw[3] = analogRead(PIN_POT4);
  raw[4] = analogRead(PIN_POT5);
  raw[5] = analogRead(PIN_JOY_X);
  raw[6] = analogRead(PIN_JOY_Y);

  // Smooth Values
  for (int i = 0; i < NUM_SENSORS; i++) {
    total[i] = total[i] - readings[i][readIndex];
    readings[i][readIndex] = raw[i];
    total[i] = total[i] + readings[i][readIndex];
    average[i] = total[i] / NUM_READINGS;
  }

  readIndex = (readIndex + 1) % NUM_READINGS;

  // Send Data Packet
  // Format: <P1,P2,P3,P4,P5,JX,JY>
  Serial.print("<");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(average[i]);
    if (i < NUM_SENSORS - 1)
      Serial.print(",");
  }
  Serial.println(">");

  delay(20);
}
