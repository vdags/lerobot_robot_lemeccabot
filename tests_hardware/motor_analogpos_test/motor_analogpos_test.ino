/*
  This code snippet is used to control a servo motor to scan from 0 to 180 degrees 
  and then back from 180 to 0 degrees repeatedly.
  
  Board: Arduino Uno R3 (or R4)
  Component: Servo motor(SG90)
*/

#include <Servo.h>

const int servoPin = 13;  // Define the servo pin
const int powerPin = 2; 
int angle = 0;           // Initialize the angle variable to 0 degrees
Servo servo;             // Create a servo object
const int analogPin = A0;

void setup() {
  servo.attach(servoPin);
  Serial.begin(9600);
  pinMode(2,OUTPUT);
  pinMode(A0, INPUT);
}

void loop() {
  // scan from 0 to 180 degrees
  
  for (angle = 0; angle < 180; angle++) {
    servo.write(angle);
    delay(15);
    Serial.println(analogRead(analogPin));
  }
  // now scan back from 180 to 0 degrees
  for (angle = 180; angle > 0; angle--) {
    servo.write(angle);
    delay(15);
    Serial.println(analogRead(analogPin));
  }
  
  //digitalWrite(2,LOW);
  //delay(1000);
  /*
  servo.write(100);
  Serial.print("Angle: ");
  Serial.println(servo.read());
  digitalWrite(2,HIGH);
  */
  //delay(1000);
}