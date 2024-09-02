#include <Arduino.h>
#include <stdlib.h>

#define LOG_EVENTS

#define ENABLE_NS_JOYSTICK

#ifdef ENABLE_NS_JOYSTICK
#include "Joystick.h"
const int led_pin[4] = {8, 9, 10, 11};
const int sensor_button[4] = {SWITCH_BTN_ZL, SWITCH_BTN_LCLICK, SWITCH_BTN_RCLICK, SWITCH_BTN_ZR};
#endif

// time units are microseconds
#define TRIGGER_COOLDOWN 20000
#define OUTPUT_HOLD_TIME 32000
#define N_READINGS 10
#define DERIVATIVE_GAIN 500.0

const int pin[4] = {A4, A3, A5, A1};

float threshold[4] = {10, 6, 6, 10};
float min_threshold[4] = {10, 6, 6, 10};
float threshold_gain[4] = {0.8, 0.8, 0.8, 0.8};

template <typename T>
class RingBuffer {
public:
    T* values;
    int size;
    int write_index;

    RingBuffer(int bufferSize) : size(bufferSize), write_index(0) {
        values = new T[size];
    }

    ~RingBuffer() {
        delete[] values;
    }

    // void write(const T& value) {
    //     values[write_index] = value;
    //     write_index = (write_index + 1) % size;
    // }

    T& read(int index) {
      
        return values[(write_index + index) % size];
    }

    T& operator[](int index) {
        return read(index);
    }

    T& readLast() {
        return read(size - 1);
    }

    // const T& operator[](int index) const {
    //     return read(index);
    // }

    void incrementWriteIndex() {
        write_index = (write_index + 1) % size;
    }
};

RingBuffer<int[4]> raw(N_READINGS);
RingBuffer<float[4]> smoothed(2);
RingBuffer<float[4]> derivative(3);

float level[4] = {0, 0, 0, 0};
long cooldown = 0;
int up[4] = {0, 0, 0, 0};

typedef unsigned long time_t;
time_t t = 0.0;
time_t dt = 1.0;

time_t debug_t = 0.0;

void setup() {
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

void computeSmoothed() {
  for (int pin_index = 0; pin_index < 4; pin_index++) {
    float* smoothed_values = smoothed[0];
    smoothed_values[pin_index] = 0.0;
    for (uint8_t i = 0; i < N_READINGS; i++) {
      smoothed_values[pin_index] += raw[i][pin_index];
    }
    smoothed_values[pin_index] /= N_READINGS;
  }
  // Move to the next slot after updating all pins
  smoothed.incrementWriteIndex();
}
// void compute_smoothed(int pin_index) {
//   float* smoothed_values = smoothed[0];
//   smoothed_values[pin_index] = 0;
//   for (uint8_t i = 0; i < N_READINGS; i++) {
//     smoothed_values[pin_index] += raw[i][pin_index];
//     // Serial.print(raw[i][pin_index]);
//     // Serial.print(" ");
//   }
//   // Serial.print(smoothed_values[pin_index]);
//   // Serial.print(" ");
//   smoothed_values[pin_index] /= N_READINGS;
//   if (pin_index == 0) {
//     Serial.print(smoothed[0][pin_index]);
//     Serial.print(" ");
//   }
//   smoothed.incrementWriteIndex();
//   if (pin_index == 0) {
    
//     Serial.print(smoothed[1][pin_index]);
//     Serial.print(" ");

//     Serial.println(smoothed[0][pin_index]);
//   }
// }

// void compute_derivative(int pin_index) {
//   float* derivative_values = derivative[0];
//   derivative_values[pin_index] =  smoothed[1][pin_index] - smoothed[0][pin_index];
//   derivative.incrementWriteIndex();
// }

void computeDerivative() {
  for (int pin_index = 0; pin_index < 4; pin_index++) {
    float* derivative_values = derivative[0];
    derivative_values[pin_index] = smoothed[1][pin_index] - smoothed[0][pin_index];
  }
  // Move to the next slot after updating all pins
  derivative.incrementWriteIndex();
}

void computeLevels() {
  for (uint8_t pin_index = 0; pin_index < 4; pin_index++) {
    level[pin_index] = 2*derivative[1][pin_index] - (derivative[2][pin_index] + derivative[0][pin_index]);
  }
  // Serial.println("");
}

void readAll() {
  dt = micros() - t;
  t += dt;
  int* raw_values = raw[0];
  for (uint8_t pin_index = 0; pin_index < 4; pin_index++) {
    raw_values[pin_index] = analogRead(pin[pin_index]);
  }
  raw.incrementWriteIndex();
}

// void logBuffer(Ringbuffer *buffer) {
//   Serial.print(t);
//   Serial.print(" ");
//   for (uint8_t i = 0; i <4; i++) {
//     Serial.print(buffer.read_last()[i]);
//     Serial.print(" ");
//   }
//   Serial.println("");
// }

void logFullRaw(int pin_index) {
  for (uint8_t i = 0; i < N_READINGS; i++) {
    Serial.print(raw[i][pin_index]);
    Serial.print(" ");
  }
  Serial.println("");
}

void logRaw() {
  Serial.print(t);
  Serial.print(" ");
  for (uint8_t pin_index = 0; pin_index < 4; pin_index++) {
    Serial.print(raw.readLast()[pin_index]);
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

void logFullSmoothed(int pin_index) {
  for (uint8_t i = 0; i < 2; i++) {
    Serial.print(smoothed[i][pin_index]);
    Serial.print(" ");
  }
  Serial.println("");
}

void logSmoothed() {
  for (uint8_t pin_index = 0; pin_index < 4; pin_index++) {
    Serial.print(smoothed.readLast()[pin_index]);
    Serial.print(" ");
  }
  Serial.println("");
}

void logDerivatives() {
  for (uint8_t pin_index = 0; pin_index < 4; pin_index++) {
    Serial.print(derivative.readLast()[pin_index]);
    Serial.print(" ");
  }
  Serial.println("");
}

void onUp(int pin_index) {
  up[pin_index] = OUTPUT_HOLD_TIME;
  cooldown = t + TRIGGER_COOLDOWN;

  #ifdef LOG_EVENTS
  Serial.print("[");
  Serial.print(t);
  Serial.print("] ");

  Serial.print("UP ");
  Serial.print(pin_index);
  Serial.println("");
  #endif

  #ifdef ENABLE_NS_JOYSTICK
  setSwitchState(pin_index, true);
  #endif

  digitalWrite(LED_BUILTIN, HIGH);
}

void onDown(int pin_index) {
  #ifdef LOG_EVENTS
  Serial.print("[");
  Serial.print(t);
  Serial.print("] ");
  Serial.print("DOWN ");
  Serial.print(pin_index);
  Serial.println("");
  #endif

  #ifdef ENABLE_NS_JOYSTICK
  setSwitchState(pin_index, false);
  #endif

  digitalWrite(LED_BUILTIN, LOW);
}

#ifdef ENABLE_NS_JOYSTICK
void setSwitchState(int sensor_index, bool state) {
    Joystick.Button |= (state ? sensor_button[sensor_index] : SWITCH_BTN_NONE);
    Joystick.sendState();
    Joystick.Button = SWITCH_BTN_NONE;
}
#endif

void _max(float* a, int size, float* max, int* max_index) {
  *max = a[0];
  *max_index = 0;
  for (int i = 1; i < size; i++) {
    if (a[i] > *max) {
      *max = a[i];
      *max_index = i;
    }
  }
}

void triggerEvents() {
  
  for (uint8_t pin_index = 0; pin_index < 4; pin_index++) {
    if (up[pin_index] > 0) {
      up[pin_index] -= dt;
      if (up[pin_index] <= 0) {
        onDown(pin_index);
      }
    }
  }

  int pin_index;
  float max_level;

  _max(level, 4, &max_level, &pin_index);
  
  if (level[pin_index] > threshold[pin_index] && cooldown < t) {
    threshold[pin_index] = level[pin_index];
    onUp(pin_index);
  } else {
    threshold[pin_index] = threshold[pin_index] * threshold_gain[pin_index] + min_threshold[pin_index] * (1.0 - threshold_gain[pin_index]);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  readAll();
  computeSmoothed();
  computeDerivative();
  computeLevels();
  // logFullRaw(0);
  // logRaw();
  // logSmoothed();
  // logFullSmoothed(0);
  // logDerivatives();
  // logLevels();
  triggerEvents();

}
