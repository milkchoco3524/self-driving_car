#include <NewPing.h>

#define IN1 9 // 핀 번호는 자신의 환경에 맞도록 설정
#define IN2 8
#define ENR 12
#define IN3 11
#define IN4 10
#define ENL 13
#define SONAR_NUM 3            // 초음파 센서 개수
#define MAX_DISTANCE 150       // 최대 거리 (cm)
#define FRONT_TRIG_PIN 2
#define FRONT_ECHO_PIN 3
#define RIGHT_TRIG_PIN 4
#define RIGHT_ECHO_PIN 5
#define LEFT_TRIG_PIN 6
#define LEFT_ECHO_PIN 7


void setup()
{
    pinMode (IN1, OUTPUT);
    pinMode (IN2, OUTPUT);
    pinMode (IN3, OUTPUT);
    pinMode (IN4, OUTPUT);
    pinMode (ENL, OUTPUT);
    pinMode (ENR, OUTPUT);
}

void motor_l(int speed)
{
  if (speed >=0)
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite (ENL, speed); // 0-255
  }
  else
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite (ENL, -speed);
  }
}

void motor_r(int speed)
{
  if (speed >=0)
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite (ENR, speed); // 0-255
  }
  else
  {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite (ENR, -speed);
  }
}

void robot_control( int left_motor_speed, int right_motor_speed)

{

   // 아래에 왼쪽 모터 제어 함수

      motor_l(left_motor_speed);  

   // 아래에 오른쪽 모터 제어 함수

      motor_r(right_motor_speed);


}

void loop()
{
   // 3초정지

   robot_control(0, 0);

   delay(3000);  
   
   // 직진 

   robot_control(-150, 150);

   delay(3000);   // 직진시간

   robot_control(0, 0);  // 정지

   delay(1000);

   // 오른쪽 회전 , 계수값 조정할 것 제 자리서 돌도록)

   robot_control(-200, -200);

   delay(375);   // 회전하는 시간

   robot_control(0, 0);  // 정지

   delay(1000);


   // 직진 

   robot_control(-150, 150);

   delay(2500);   // 직진시간

   robot_control(0, 0);  // 정지
   
   delay(1000);


  // 왼쪽 회전

   robot_control(200, 200);

   delay(420);   // 회전하는 시간

   robot_control(0, 0);  // 정지
   
   
   // 직진 

   robot_control(-150, 150);

   delay(3000);   // 직진시간

   robot_control(0, 0);  // 정지
}
