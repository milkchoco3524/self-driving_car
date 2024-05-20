#include <Wire.h>
#include <LSM303.h>

LSM303 compass;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  compass.init();
  compass.enableDefault();

  compass.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
  compass.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};
}

void loop() {
  compass.read();

  float heading = compass.heading();
  float target = 90.0;
  float error = 0.0;
  error = heading - target;
  Serial.print(error);  Serial.print(" = "); Serial.print(heading); Serial.print(" - "); Serial.println(target);
  delay(100);
}
