#include <Arduino.h>

float UltrasonicSensorReading(int trig_pin, int echo_pin);
float medianDistance(int trigPin, int echoPin);

// PCB layout
#define TRIG_PIN_1    40
#define ECHO_PIN_1    39
#define TRIG_PIN_2    8
#define ECHO_PIN_2    10
#define TRIG_PIN_3    11
#define ECHO_PIN_3    12

#define NO_PIN_1      38
#define LED_PIN       6
#define Solenoid_pin  15 

// Sampling rate (~10Hz telemetry for ROS 2)
#define DELAY 100

const int trigPins[3] = { TRIG_PIN_1, TRIG_PIN_2, TRIG_PIN_3 };
const int echoPins[3] = { ECHO_PIN_1, ECHO_PIN_2, ECHO_PIN_3 };

void setup() {
  // Sensors setup
  for (int i = 0; i < 3; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
    digitalWrite(trigPins[i], LOW);
  }

  // Solenoid & LED setup
  pinMode(Solenoid_pin, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  // Closing sensor
  pinMode(NO_PIN_1, INPUT_PULLUP);

  // Serial setup (Must match monitor_speed in platformio.ini)
  Serial.begin(115200);
}

void loop() {
  float detection_filtered[3];
  int lock_state = 0;

  // Read all 3 ultrasonic sensors
  for (int i = 0; i < 3; i++) {
    detection_filtered[i] = medianDistance(trigPins[i], echoPins[i]);
  }

  // Read closing sensor state (1 for closed, 0 for open)
  lock_state = (digitalRead(NO_PIN_1) == LOW) ? 1 : 0;

  // PRINT CSV TELEMETRY STREAM FOR ROS 2
  // Format: dist1,dist2,dist3,door_state
  for (int i = 0; i < 3; i++) {
    Serial.print(detection_filtered[i], 1);
    Serial.print(",");
  }
  Serial.println(lock_state);

  // Onboard LED Debugging (Lights up if ANY sensor sees an obstacle under 1 meter)
  bool obstacleDetected = false;
  for (int i = 0; i < 3; i++) {
    if (detection_filtered[i] > 0 && detection_filtered[i] < 100) {
      obstacleDetected = true;
    }
  }

  if (obstacleDetected) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }

  delay(DELAY);
}

float UltrasonicSensorReading(int trig_pin, int echo_pin) {
  long duration;
  float distance;

  // Pulse trigger
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);

  // Reading echo with 30ms timeout (~5 meters max range)
  duration = pulseIn(echo_pin, HIGH, 30000);

  if (duration == 0) {
    return -1.0;
  }

  // Convert to cm
  distance = duration * 0.017;
  return distance;
}

float medianDistance(int trigPin, int echoPin) {
  float values[5];

  // Store 5 samples
  for (int i = 0; i < 5; i++) {
    values[i] = UltrasonicSensorReading(trigPin, echoPin);
    delay(20);
  }

  // Bubble sort
  for (int i = 0; i < 4; i++) {
    for (int j = i + 1; j < 5; j++) {
      if (values[j] < values[i]) {
        float temp = values[i];
        values[i] = values[j];
        values[j] = temp;
      }
    }
  }

  // Return median value
  return values[2];
}