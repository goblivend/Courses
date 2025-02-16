//Arduino PWM Speed Control：

#include <Servo.h>

#define FORWARD LOW
#define BACKWARD HIGH

int car_speed_pin = 6;
int car_way_pin = 7;
int direction_servo_pin = 13;

Servo servo;

int r = 345; // [cm]
int L = 9.5; // [cm] Distance between front and back wheels

float turningAngle = 2 * degrees(atan2(L, r));
float convertionAngleRight = 20.0f/12.0f;
float convertionAngleLeft = 20.0f/8.65f;

int timeLeft = 6000;
int timeRight = 5690;
int speedLeft = 255;
int speedRight = speedLeft;

void turn(float angle) {
  Serial.print(angle);
  Serial.print(ceil(angle));
  if (angle < 50) {
    angle = 70 ;
  } else if (angle > 110) {
    angle = 110;
  }

  servo.write(ceil(angle));
  delay(15);
}

void turnLeft(float turnAngle) {
  turn(90 - turnAngle * convertionAngleLeft);
}

void turnRight(float turnAngle) {
  turn(90 + turnAngle * convertionAngleRight);
}

void move(int way, int speed, int time) {
  digitalWrite(car_way_pin, way);
  analogWrite(car_speed_pin, speed);
  delay(time);
  digitalWrite(car_speed_pin ,0);
}

void setup()
{
    pinMode(car_way_pin, OUTPUT);
    servo.attach(direction_servo_pin);
}

void loop()
{

  
  // Turn Left for X seconds
  // turnLeft(turningAngle);
  // move(BACKWARD, speedLeft, timeLeft);
  // Turn Right for Y seconds
  // turnRight(turningAngle);
  // move(BACKWARD, speedRight, timeRight);
  // turn(90);



  // Turn Left for X seconds
  // turn(90);
  // delay(5000);
  move(FORWARD, 128, 100000);
  // Turn Right for Y seconds
  // turn(110);
  // move(FORWARD, 128, 10000);
  // turn(90);
}
