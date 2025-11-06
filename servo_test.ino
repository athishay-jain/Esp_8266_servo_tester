/*  ESP8266 + 2× MG996R slow, slew-limited control
    - Servo1: controlled by potentiometer (0–90°, reversed)
    - Servo2: controlled by a push button/switch (reversed logic)
        - Switch pressed  →  Servo = 0°
        - Switch released →  Servo = 90°

    Wiring:
    - Servo RED  -> 5–6V external power (NOT ESP8266 3.3V)
    - Servo BROWN -> GND (common with ESP8266)
    - Servo1 ORANGE -> D4
    - Servo2 ORANGE -> D5
    - Pot: one end 3.3V, other GND, wiper -> A0
    - Switch: one side -> GND, other side -> D6 (with INPUT_PULLUP)
*/

#include <Arduino.h>
#include <Servo.h>

#if defined(ARDUINO_ARCH_ESP8266)
  // ok
#else
  #error "This sketch targets ESP8266 (NodeMCU/Wemos)."
#endif

// ---------- Pins ----------
const int SERVO1_PIN = D4;   // Pot-controlled servo
const int SERVO2_PIN = D5;   // Switch-controlled servo
const int POT1_PIN   = A0;
const int SWITCH_PIN = D6;   // Button (active LOW)

// ---------- Motion settings ----------
const float MAX_SPEED_DEG_PER_SEC = 60.0f;
const uint32_t UPDATE_INTERVAL_MS  = 20;
const float POT_SMOOTH_ALPHA       = 0.15f;

// MG996R pulse range
const int SERVO_US_MIN = 500;
const int SERVO_US_MAX = 2500;

// Angle limits
const float ANGLE_MIN = 0.0f;
const float ANGLE_MAX = 90.0f;

// ADC calibration
const int ADC_MIN_CAL = 0;
const int ADC_MAX_CAL = 1023;

Servo servo1, servo2;

float target1 = 0.0f, current1 = 0.0f, pot1Ema = 0.0f;
float target2 = 0.0f, current2 = 0.0f;

uint32_t lastUpdateMs = 0;

static inline float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  if (in_max == in_min) return out_min;
  float t = (x - in_min) / (in_max - in_min);
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  return out_min + t * (out_max - out_min);
}

void setup() {
  Serial.begin(115200);
  delay(100);

  servo1.attach(SERVO1_PIN, SERVO_US_MIN, SERVO_US_MAX);
  servo2.attach(SERVO2_PIN, SERVO_US_MIN, SERVO_US_MAX);

  pinMode(SWITCH_PIN, INPUT_PULLUP); // switch pressed = LOW

  current1 = current2 = (ANGLE_MIN + ANGLE_MAX) * 0.5f;
  servo1.write(current1);
  servo2.write(current2);

  int raw = analogRead(POT1_PIN);
  pot1Ema = raw;

  Serial.println(F("ESP8266 Dual MG996R: Servo1 via Pot, Servo2 via Switch (reversed logic)"));
}

void loop() {
  // ---------- Servo 1 (Pot-controlled) ----------
  int raw = analogRead(POT1_PIN);
  pot1Ema = POT_SMOOTH_ALPHA * raw + (1.0f - POT_SMOOTH_ALPHA) * pot1Ema;
  float mapped = fmap(pot1Ema, ADC_MIN_CAL, ADC_MAX_CAL, ANGLE_MAX, ANGLE_MIN); // reversed pot
  target1 = constrain(mapped, ANGLE_MIN, ANGLE_MAX);

  // ---------- Servo 2 (Switch-controlled, reversed logic) ----------
  bool pressed = (digitalRead(SWITCH_PIN) == LOW);
  target2 = pressed ? ANGLE_MIN : ANGLE_MAX;  // reversed behavior

  // ---------- Slew update ----------
  uint32_t now = millis();
  if (now - lastUpdateMs >= UPDATE_INTERVAL_MS) {
    lastUpdateMs = now;
    float maxStep = MAX_SPEED_DEG_PER_SEC * (UPDATE_INTERVAL_MS / 1000.0f);

    // Servo 1
    float diff1 = target1 - current1;
    if (fabs(diff1) <= maxStep) current1 = target1;
    else current1 += (diff1 > 0 ? maxStep : -maxStep);
    servo1.write((int)roundf(current1));

    // Servo 2
    float diff2 = target2 - current2;
    if (fabs(diff2) <= maxStep) current2 = target2;
    else current2 += (diff2 > 0 ? maxStep : -maxStep);
    servo2.write((int)roundf(current2));
  }
}
