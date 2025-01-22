
#include <Servo.h>

Servo myservo;// direction de la voiture
int shift = 10; //le servo est decentre par rapport a la direction
int rightMaxPos = 120  + shift; 
int leftMaxpos = 60  + shift;
int neutralPos = 90 + shift;//90 for mid-course position
int pos = neutralPos;// variable to store the servo position

void setup() 
{ 
    myservo.attach(13);// attaches the servo on pin 13 to the servo object
} 

void loop() 
{ 
    //turn right
    for (pos = neutralPos; pos <= rightMaxPos; pos += 1) {
        myservo.write(pos);
        delay(15);
    }    
    delay(300);
    for (pos = rightMaxPos; pos >= neutralPos; pos -= 1) {
        myservo.write(pos);
        delay(15);
    }
    delay(300);
    //turn left
    for (pos = neutralPos; pos >= leftMaxpos; pos -= 1) {
        myservo.write(pos);
        delay(15);
    }
    delay(300);    
    for (pos = leftMaxpos; pos <= neutralPos; pos += 1) {
      myservo.write(pos);
        delay(15);
    }
    delay(1000);

}
