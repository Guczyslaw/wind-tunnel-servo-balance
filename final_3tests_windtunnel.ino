#include <Servo.h>
#include "HX711.h"

// ---------------- SERVO ----------------
Servo myservo;
const int SERVO_PIN = 9;

const int STEP_DEG = 15;          // 15° intervals
const int HOLD_MS  = 4000;        // hold time at each angle for data collection
const int SETTLE_MS = 300;        // settle after moving
const int TESTS = 3;

#define DRAG_CLK  2
#define DRAG_DOUT 3
HX711 scaleDrag;

#define LIFT_CLK  4
#define LIFT_DOUT 5
HX711 scaleLift;

float calibDrag = -82300000;
float calibLift = -116700000;  

// Physics
const float G = 9.80665f; // N per kgf reading

// Angles: 0..180 step 15 => 13 points
const int N_ANGLES = (180 / STEP_DEG) + 1;

// Storage for 3 runs (only one measurement per angle per run)
float dragN[TESTS][N_ANGLES];
float liftN[TESTS][N_ANGLES];

// Convert servo angle to your measured AoA (degrees to horizontal)
float servoToAoA(int servoDeg) {
  // From your table: 0->30, 90->0, 180->-30
  return 30.0f - (servoDeg / 3.0f);
}

void zeroSystem() {
  Serial.println("========================================");
  Serial.println("WIND TUNNEL TEST");
  Serial.println("ZEROING SYSTEM (AoA = 0°, servo = 90°)");
  Serial.println("Fan OFF – establishing reference");
  Serial.println("========================================");

  // Put model at 0° AoA before taring to reduce bias
  myservo.write(90);
  delay(1200);

  Serial.println("ZEROING: fan OFF, no aerodynamic load. Taring both load cells...");

  scaleDrag.tare();
  scaleLift.tare();

  Serial.println("Zeroing complete.");
}

void takeMeasurement(int testIndex, int angleIndex, int servoDeg) {
  myservo.write(servoDeg);
  delay(SETTLE_MS);

  // Average 10 samples each
  float drag_kg = scaleDrag.get_units(10);
  float lift_kg = scaleLift.get_units(10);

  float drag_N = drag_kg * G;
  float lift_N = lift_kg * G;

  dragN[testIndex][angleIndex] = drag_N;
  liftN[testIndex][angleIndex] = lift_N;

  float aoa = servoToAoA(servoDeg);

  // Raw CSV output (easy for Excel)
  // test,servo_deg,aoa_deg,drag_N,lift_N,drag_kg,lift_kg
  Serial.print(testIndex + 1);
  Serial.print(",");
  Serial.print(servoDeg);
  Serial.print(",");
  Serial.print(aoa, 1);
  Serial.print(",");
  Serial.print(drag_N, 3);
  Serial.print(",");
  Serial.print(lift_N, 3);
  Serial.print(",");
  Serial.print(drag_kg, 3);
  Serial.print(",");
  Serial.println(lift_kg, 3);

  delay(HOLD_MS);
}

float meanOf(float arr[TESTS], int n) {
  float s = 0.0f;
  for (int i = 0; i < n; i++) s += arr[i];
  return s / n;
}

float stddevOf(float arr[TESTS], int n) {
  // sample standard deviation (n-1)
  if (n < 2) return 0.0f;
  float m = meanOf(arr, n);
  float s2 = 0.0f;
  for (int i = 0; i < n; i++) {
    float d = arr[i] - m;
    s2 += d * d;
  }
  return sqrt(s2 / (n - 1));
}

void setup() {
  Serial.begin(9600);

  myservo.attach(SERVO_PIN);

  scaleDrag.begin(DRAG_DOUT, DRAG_CLK);
  scaleLift.begin(LIFT_DOUT, LIFT_CLK);

  scaleDrag.set_scale(calibDrag);
  scaleLift.set_scale(calibLift);

  Serial.println("test,servo_deg,aoa_deg,drag_N,lift_N,drag_kg,lift_kg");

  // Zero once at the beginning
  zeroSystem();

  delay(8000);

  // -------- Run tests --------
  for (int t = 0; t < TESTS; t++) {
    Serial.print("START_TEST,");
    Serial.println(t + 1);


    // Up sweep: 0 → 180 (record one point per angle)
    int idx = 0;
    for (int angle = 0; angle <= 180; angle += STEP_DEG) {
      takeMeasurement(t, idx, angle);
      idx++;
    }

    // Down sweep: 180 → 0 (motion only, no recording, to complete "full sweep")
    for (int angle = 180; angle >= 0; angle -= STEP_DEG) {
      myservo.write(angle);
      delay(200); // small motion delay; no 4s hold here
    }

    Serial.print("END_TEST,");
    Serial.println(t + 1);
  }
  Serial.println("ALL TESTS COMPLETE.");
}

void loop() {
  while (true) {
    delay(1000);
  }
}