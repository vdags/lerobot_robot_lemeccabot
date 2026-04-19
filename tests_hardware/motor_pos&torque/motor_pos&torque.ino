/*
  This code snippet is used to control a servo motor to scan from 0 to 180 degrees 
  and then back from 180 to 0 degrees repeatedly with position and torque monitoring.
  
  Board: Arduino Uno R3 (or R4)
  Component: Servo motor(SG90)
*/

#include <Servo.h>

const int servoPin = 13;  // Define the servo pin
int angle = 0;           // Initialize the angle variable to 0 degrees
Servo servo;             // Create a servo object
const int TorquePin = A0;
const int PotPin = A1;
int Torque = 0;
int Pot = 0;

void setup() {
  servo.attach(servoPin);
  Serial.begin(9600);
  pinMode(2,OUTPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
}

void loop() {
  // scan from 0 to 180 degrees
  
  for (angle = 0; angle < 180; angle++) {
    servo.write(angle);
    delay(15);
    Torque = analogRead(TorquePin);
    Serial.print("Torque :");
    Serial.println(Torque);
    Serial.print("Pot :");
    Pot  = analogRead(PotPin);
    Serial.println(Pot);
  }
  // now scan back from 180 to 0 degrees
  for (angle = 180; angle > 0; angle--) {
    servo.write(angle);
    delay(15);
    Torque = analogRead(TorquePin);
    Serial.print("Torque :");
    Serial.println(Torque);
    Serial.print(" Pot :");
    Pot  = analogRead(PotPin);
    Serial.println(Pot);
  }
  
}