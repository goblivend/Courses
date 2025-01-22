#include <Servo.h>
#include <SoftwareSerial.h>

#define FORWARD LOW
#define BACKWARD HIGH

int car_speed_pin = 6;
int car_way_pin = 7;
int direction_servo_pin = 13;

SoftwareSerial e5(0, 1); // (RX, TX)
Servo servo;

int r = 345; // [cm]
int L = 9.5; // [cm] Distance between front and back wheels

float turningAngle = 2 * degrees(atan2(L, r));
float convertionAngleRight = 20.0f / 12.0f;
float convertionAngleLeft = 20.0f / 8.65f;

int timeLeft = 6000;
int timeRight = 5690;
int speedLeft = 255;
int speedRight = speedLeft;

void turn(float angle)
{
    if (angle < 70)
    {
        angle = 70;
    }
    else if (angle > 110)
    {
        angle = 110;
    }

    servo.write(ceil(angle));
    delay(15);
}

void turnLeft(float turnAngle)
{
    turn(90 - turnAngle * convertionAngleLeft);
}

void turnRight(float turnAngle)
{
    turn(90 + turnAngle * convertionAngleRight);
}

void move(int way, int speed, int time)
{
    digitalWrite(car_way_pin, way);
    analogWrite(car_speed_pin, speed);
    delay(time);
    digitalWrite(car_speed_pin, 0);
}

void setup()
{
    e5.begin(115200);
    pinMode(car_way_pin, OUTPUT);
    pinMode(car_speed_pin, OUTPUT);
    servo.attach(direction_servo_pin);
}

void loop()
{
    while (e5.available() == 0)
        ;

    String out = e5.readStringUntil('\n');
    Serial.println(out);
    int dir = atoi(out.c_str());

    float direction = 90.f + ((float)dir) / 128.f * 20.f;

    turn(direction);
    move(FORWARD, 100, 300);
}
