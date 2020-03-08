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

// Define motor interface type
#define motorInterfaceType 1

// Creates an instance
AccelStepper myStepper(motorInterfaceType, stepPin, dirPin);


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
  
  Serial.begin(9600);

  Blynk.begin(auth, ssid, pass);

  //Set the microstep resolution to 1/16 (3200 steps per millimeter)
  digitalWrite(MS1, HIGH);
  digitalWrite(MS2, HIGH);
  digitalWrite(MS3, HIGH);

  myStepper.setMaxSpeed(1000);
  myStepper.setAcceleration(50);
  myStepper.setSpeed(200);
  myStepper.moveTo(200);
  
}

void loop()
{
  Blynk.run();
}
