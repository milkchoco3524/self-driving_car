#include <NewPing.h>

#define FRONT_TRIG_PIN 2
#define FRONT_ECHO_PIN 3
#define RIGHT_TRIG_PIN 4
#define RIGHT_ECHO_PIN 5
#define LEFT_TRIG_PIN 6
#define LEFT_ECHO_PIN 7
#define MAX_DISTANCE 150
#define NUM_SAMPLES 5

NewPing frontSonar(FRONT_TRIG_PIN, FRONT_ECHO_PIN, MAX_DISTANCE);
NewPing rightSonar(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN, MAX_DISTANCE);
NewPing leftSonar(LEFT_TRIG_PIN, LEFT_ECHO_PIN, MAX_DISTANCE);

float front_distances[NUM_SAMPLES];
float right_distances[NUM_SAMPLES];
float left_distances[NUM_SAMPLES];

void setup() {
  Serial.begin(9600);
}

float movingAverage(float *distances) {
  float sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sum += distances[i];
  }
  return sum / NUM_SAMPLES;
}

void loop() {
  float front_distance = 0.0;
  float right_distance = 0.0;
  float left_distance = 0.0;
  delay(50);
  front_distance = frontSonar.ping_cm();
  right_distance = rightSonar.ping_cm();
  left_distance = leftSonar.ping_cm();
  
  if (front_distance == 0) front_distance = MAX_DISTANCE;
  if (right_distance == 0) right_distance = MAX_DISTANCE;
  if (left_distance == 0) left_distance = MAX_DISTANCE;
  
  // 이동평균 필터링
  for (int i = NUM_SAMPLES - 1; i > 0; i--) {
    front_distances[i] = front_distances[i - 1];
    right_distances[i] = right_distances[i - 1];
    left_distances[i] = left_distances[i - 1];
  }
  front_distances[0] = front_distance;
  right_distances[0] = right_distance;
  left_distances[0] = left_distance;
  
  float front_avg = movingAverage(front_distances);
  float right_avg = movingAverage(right_distances);
  float left_avg = movingAverage(left_distances);
  
  Serial.print("Front distance: ");
  Serial.print(front_avg);
  Serial.print(" cm  ");
  Serial.print("Right distance: ");
  Serial.print(right_avg);
  Serial.print(" cm  ");
  Serial.print("Left distance: ");
  Serial.print(left_avg);
  Serial.println(" cm");
}
