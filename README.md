 ESP8266 + 2× MG996R slow, slew-limited control
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
