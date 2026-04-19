#include <Servo.h>

#define NUM_DEVICES 5

#define DIRECTION_CW  0
#define DIRECTION_CCW 1
#define PRECISION 5

// ================= PIN CONFIG =================

// Encoder pins (CLK, DT pairs)
int CLK_PINS[NUM_DEVICES] = {2, 4, 6, 8, 10};
int DT_PINS[NUM_DEVICES]  = {3, 5, 7, 9, 11};

// Servo pins
int SERVO_PINS[NUM_DEVICES] = {A0, A1, A2, A3, A4};

// Initial servo positions (modifiable later)
int initialPositions[NUM_DEVICES] = {90, 180, 150, 120, 0};

// ================= VARIABLES =================

int counter[NUM_DEVICES];
int direction[NUM_DEVICES];

int CLK_state[NUM_DEVICES];
int prev_CLK_state[NUM_DEVICES];

Servo servos[NUM_DEVICES];

// ================= SETUP =================

void setup() {
  Serial.begin(9600);

  for (int i = 0; i < NUM_DEVICES; i++) {
    pinMode(CLK_PINS[i], INPUT);
    pinMode(DT_PINS[i], INPUT);

    prev_CLK_state[i] = digitalRead(CLK_PINS[i]);

    servos[i].attach(SERVO_PINS[i]);

    counter[i] = initialPositions[i];
    servos[i].write(counter[i]);
  }
}

// ================= LOOP =================

void loop() {
  for (int i = 0; i < NUM_DEVICES; i++) {

    CLK_state[i] = digitalRead(CLK_PINS[i]);

    // Detect rising edge
    if (CLK_state[i] != prev_CLK_state[i] && CLK_state[i] == HIGH) {

      if (digitalRead(DT_PINS[i]) == HIGH) {
        counter[i]=counter[i]-PRECISION;
        direction[i] = DIRECTION_CCW;
      } else {
        counter[i]=counter[i]+PRECISION;
        direction[i] = DIRECTION_CW;
      }

      // Clamp between 0–180
      if (counter[i] < 0) counter[i] = 0;
      if (counter[i] > 180) counter[i] = 180;

      servos[i].write(counter[i]);

      // Debug output
      Serial.print("Servo ");
      Serial.print(i);
      Serial.print(" | Dir: ");
      Serial.print(direction[i] == DIRECTION_CW ? "CW" : "CCW");
      Serial.print(" | Angle: ");
      Serial.println(counter[i]);
    }

    prev_CLK_state[i] = CLK_state[i];
  }
}