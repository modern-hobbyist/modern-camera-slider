/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include "credentials.h"

// Include the AccelStepper Library
#include <AccelStepper.h>

// Define pin connections
const int dirPin = 14;
const int stepPin = 12;
#define MS1 25       // Pin 25 connected to MS1 pin
#define MS2 26       // Pin 26 connected to MS2 pin
#define MS3 27       // Pin 27 connected to MS2 pin
#define STEPS_PER_REVOLUTION 3200 //A4988 has a max steps per revolution of 3200 with all microsteps activated.

#define MIN_ENDSTOP_PIN 33
#define MAX_ENDSTOP_PIN 32

// Define motor interface type
#define motorInterfaceType 1

// Creates an instance
AccelStepper myStepper(motorInterfaceType, stepPin, dirPin);
int target = 10000;
int maxPosition = 10000000; //arbitrarily large endstop number;

BLYNK_WRITE(V1) {
  long startTimeInSecs = param[0].asLong();
  Serial.println(startTimeInSecs);
  Serial.println();
}

void setup()
{
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(MIN_ENDSTOP_PIN, INPUT_PULLUP);
  pinMode(MAX_ENDSTOP_PIN, INPUT_PULLUP);
  
  Serial.begin(9600);

  Blynk.begin(auth, ssid, pass);

  //Set the microstep resolution to 1/16 (3200 steps per revolution)
  digitalWrite(MS1, HIGH);
  digitalWrite(MS2, HIGH);
  digitalWrite(MS3, HIGH);

  myStepper.setMaxSpeed(10000);
  myStepper.setAcceleration(1000);
  myStepper.setSpeed(1000);
  setEndstops();
  myStepper.stop();
  myStepper.moveTo(0);
}

void loop()
{
  //handle endstop pins
  Blynk.run();
  if(myStepper.currentPosition() == 0){
    myStepper.moveTo(maxPosition);
  }else if(myStepper.currentPosition() == maxPosition){
    myStepper.moveTo(0);
  }
  myStepper.setSpeed(2000);
  myStepper.runSpeedToPosition();
}

void setEndstops(){
  setMinEndstop();
  setMaxEndstop();
}

//Might need to switch the direction of these functions
void setMinEndstop(){
  myStepper.setSpeed(-2000);
  while(digitalRead(MIN_ENDSTOP_PIN) == HIGH){
    myStepper.runSpeed();
  }
  
  myStepper.setCurrentPosition(0); //Set the current position to 0 for now
  myStepper.moveTo(200); //One rotation backwards

  //Move back one rotation
  while(myStepper.distanceToGo() != 0){
    myStepper.run();
  }

  myStepper.setSpeed(-1000); //Reduce the speed by 50%;

  while(digitalRead(MIN_ENDSTOP_PIN) == HIGH){
//    Serial.println("moving to min again");
    myStepper.runSpeed();
  }

  myStepper.setCurrentPosition(0);
  Serial.println("Setting current position to 0");
  Serial.println(myStepper.currentPosition());
}

void setMaxEndstop(){
  //TODO move the gantry towards the far end until the endstop is hit
  //TODO move back a certain number of steps
  //TODO slow down and move back until triggered again
  myStepper.setSpeed(2000);
  while(digitalRead(MAX_ENDSTOP_PIN) == HIGH){
//    Serial.println("moving to max");
    myStepper.runSpeed();
  }
  
  maxPosition = myStepper.currentPosition(); //Set the maxPosition to the current position of the stepper.
  Serial.print("Max Position 1: ");
  Serial.println(maxPosition);
  myStepper.moveTo(myStepper.currentPosition()-200); //One rotation backwards
  Serial.print("Target: ");
  Serial.println(myStepper.targetPosition());

  //Move back one rotation
  Serial.println(myStepper.distanceToGo());
  while(myStepper.distanceToGo() != 0){
    myStepper.run();
  }

  myStepper.setSpeed(1000); //Reduce the speed by 50%;

  while(digitalRead(MAX_ENDSTOP_PIN) == HIGH){
//    Serial.println("moving to max again");
    myStepper.runSpeed();
  }

  maxPosition = myStepper.currentPosition();
  Serial.print("Max Position 1: ");
  Serial.println(myStepper.currentPosition());
}
