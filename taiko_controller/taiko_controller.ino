#include <Arduino.h>
#include <stdlib.h>

#define ENABLE_NS_JOYSTICK

#ifdef ENABLE_NS_JOYSTICK
#include "Joystick.h"
const int led_pin[4] = {8, 9, 10, 11};
const int sensor_button[4] = {SWITCH_BTN_ZL, SWITCH_BTN_LCLICK, SWITCH_BTN_RCLICK, SWITCH_BTN_ZR};
#endif

// time units are microseconds
#define TRIGGER_COOLDOWN 10000
#define OUTPUT_HOLD_TIME 1000
#define N_READINGS 8
#define DERIVATIVE_GAIN 400.0

const int pin[4] = {A4, A3, A5, A1};

float threshold[4] = {1, 1, 1, 1};
float min_threshold[4] = {1, 1, 1, 1};
float threshold_gain[4] = {0.8, 0.8, 0.8, 0.8};

int raw[N_READINGS][4] = {0}; 
int raw_index = 0;

float level[4] = {0, 0, 0, 0};
float derivatives[4] = {0, 0, 0, 0};
long cooldown[4] = {0, 0, 0, 0};
int up[4] = {0, 0, 0, 0};

typedef unsigned long time_t;
time_t t = 0.0;
time_t dt = 1.0;

void setup() {
  // put your setup code here, to run once:
  analogReference(DEFAULT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  for (uint8_t pin_index = 0; pin_index < 4; pin_index++) {
    pinMode(pin[pin_index], INPUT);
  }

  #ifdef ENABLE_NS_JOYSTICK
    for (int i = 0; i < 8; ++i) pinMode(i, INPUT_PULLUP);
    for (int i = 0; i < 4; ++i) {  digitalWrite(led_pin[i], HIGH); pinMode(led_pin[i], OUTPUT); }
  #endif

  t = micros();
  Serial.begin(9600);

}

void computeLevels() {
  for (uint8_t pin_index = 0; pin_index < 4; pin_index++) {
    derivatives[pin_index] = level[pin_index];
    level[pin_index] = 0.0;
    for (uint8_t i = 0; i < N_READINGS; i++) {
      level[pin_index] += raw[i][pin_index];
    }
    level[pin_index] /= N_READINGS;

    derivatives[pin_index] =  (level[pin_index] - derivatives[pin_index]) * DERIVATIVE_GAIN / dt;
  }
}

void readAll() {
  dt = micros() - t;
  t += dt;
  for (uint8_t pin_index = 0; pin_index < 4; pin_index++) {
    raw[raw_index][pin_index] = analogRead(pin[pin_index]);
  }
  raw_index = (raw_index + 1) % N_READINGS;
}

void logRaw() {
  int last_write_index = (raw_index + (N_READINGS-1)) % N_READINGS;
  for (uint8_t pin_index = 0; pin_index < 4; pin_index++) {
    Serial.print(raw[last_write_index][pin_index]);
    Serial.print(" ");
  }
  Serial.println("");
}

void logLevels() {
  for (uint8_t pin_index = 0; pin_index < 4; pin_index++) {
    Serial.print(level[pin_index]);
    Serial.print(" ");
  }
  Serial.println("");
}

void logDerivatives() {
  for (uint8_t pin_index = 0; pin_index < 4; pin_index++) {
    Serial.print(derivatives[pin_index]);
    Serial.print(" ");
  }
  Serial.println("");
}

void onUp(int pin_index) {
  up[pin_index] = OUTPUT_HOLD_TIME;
  cooldown[pin_index] = t + TRIGGER_COOLDOWN;
  Serial.print("[");
  Serial.print(t);
  Serial.print("] ");

  Serial.print("UP ");
  Serial.print(pin_index);
  Serial.println("");

  #ifdef ENABLE_NS_JOYSTICK
  setSwitchState(pin_index, true);
  #endif

  digitalWrite(LED_BUILTIN, HIGH);
}

void onDown(int pin_index) {
  Serial.print("[");
  Serial.print(t);
  Serial.print("] ");
  Serial.print("DOWN ");
  Serial.print(pin_index);
  Serial.println("");

  #ifdef ENABLE_NS_JOYSTICK
  setSwitchState(pin_index, false);
  #endif

  digitalWrite(LED_BUILTIN, LOW);
}

void setSwitchState(int sensor_index, bool state) {
    Joystick.Button |= (state ? sensor_button[sensor_index] : SWITCH_BTN_NONE);
    digitalWrite(led_pin[sensor_index], state ? LOW : HIGH);
}


void triggerEvents() {
  
  for (uint8_t pin_index = 0; pin_index < 4; pin_index++) {
    if (up[pin_index] > 0) {
      up[pin_index] -= dt;
      if (up[pin_index] <= 0) {
        onDown(pin_index);
      }
    }
    if (derivatives[pin_index] > threshold[pin_index] && cooldown[pin_index] < t) {
      threshold[pin_index] = threshold[pin_index] * threshold_gain[pin_index] + derivatives[pin_index] * (1.0 - threshold_gain[pin_index]);
      onUp(pin_index);
    } else {
      threshold[pin_index] = threshold[pin_index] * threshold_gain[pin_index] + min_threshold[pin_index] * (1.0 - threshold_gain[pin_index]);
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  readAll();
  computeLevels();
  //logDerivatives();
  triggerEvents();
}
