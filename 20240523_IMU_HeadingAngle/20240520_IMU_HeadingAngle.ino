#include <PID_v1.h>

#include <Wire.h>
#include <LSM303.h>
#include <PID_v1.h>

double Setpoint, Input, Output;
//Specify the links and initial tuning parameters
double Kp=2, Ki=0, Kd=0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

LSM303 compass;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  compass.init();
  compass.enableDefault();

  compass.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
  compass.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};

  //PID Setup
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  compass.read();

  float heading = compass.heading();
  float target = 30.0;
  float error = 0.0;

  heading = 360 - heading;
  
  error = target - heading;
  if (error > 180) {
    error -= 360;
  } else if (error < -180) {
    error += 360;
  }
  
  Input = heading;
  Setpoint = target;
  myPID.Compute();
  
  Serial.print(error);  Serial.print(" = "); Serial.print(heading); Serial.print(" - "); Serial.println(target);
  Serial.print("output = "); Serial.println(Output);
  Serial.println(" ");

  
//  delay(100);
}
