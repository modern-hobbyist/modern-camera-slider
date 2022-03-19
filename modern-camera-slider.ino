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
#define TRAVEL_SPEED 10000
#define REFINEMENT_SPEED 1000

// Define motor interface type
#define motorInterfaceType 1

// Creates an instance
AccelStepper myStepper(motorInterfaceType, stepPin, dirPin);
int targetPosition = 0;
int maxPosition = 0; //arbitrarily large endstop number;
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers
bool endstopsSet = false;
long travelTimeInSecs = 0;
int distancePercentage = 100;
int playSpeed = 0;
bool stopped = true;
bool play = false;
bool rightToLeft = false;
bool durationSet = false;
bool distanceSet = false;
bool sendHome = false;
bool continuous = false;

BLYNK_CONNECTED() {
  Blynk.virtualWrite(V0, 0);
  Blynk.virtualWrite(V1, 0);
  Blynk.virtualWrite(V2, distancePercentage);
  Blynk.virtualWrite(V3, 2);
  Blynk.virtualWrite(V4, 1);
  Blynk.virtualWrite(V5, 1);
  //Blynk.syncAll();
    
}

BLYNK_WRITE(V0) {
  if(param[0]){
    Serial.println("Homing Gantry");
    Blynk.virtualWrite(V0, 1);
    homeGantry();
    Blynk.virtualWrite(V0, 0);
  }
}

BLYNK_WRITE(V1) {
  travelTimeInSecs = param[0].asLong();
  Serial.print("Travel Time: ");
  Serial.print(travelTimeInSecs);
  Serial.println();
  Blynk.virtualWrite(V1, travelTimeInSecs);
  if(travelTimeInSecs > 0){
    durationSet = true;
  }
}

BLYNK_WRITE(V2) {
  distancePercentage = param[0];
  Serial.print("Distance Percentage: ");
  Serial.print(distancePercentage);
  Serial.println();
  Blynk.virtualWrite(V2, distancePercentage);
  if(distancePercentage > 0){
    distanceSet = true;
  }
}

BLYNK_WRITE(V3) {
  if(param[0] == 1){
    rightToLeft = true;
  }else{
    rightToLeft = false;
  }
  stopSlider();
}

BLYNK_WRITE(V4){
  if(!durationSet || !distanceSet){
    Blynk.virtualWrite(V4, 1);
  }
  
  if(param[0] == 1){
    stopSlider();
  }else if(param[0] == 2){
    pauseSlider();
  }else{
    playSlider();
  }
}

BLYNK_WRITE(V5){
  continuous = false;
  if(param[0] == 2){
     continuous = true;
  }
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
}

void loop()
{
  //handle endstop pins
  Blynk.run();
  if(sendHome){
    if(myStepper.targetPosition() == myStepper.currentPosition()){
      sendHome = false;
    }else if(myStepper.targetPosition() > myStepper.currentPosition()){
      myStepper.setSpeed(-1 * TRAVEL_SPEED);
    }else{
      myStepper.setSpeed(TRAVEL_SPEED);
    }
    myStepper.runSpeedToPosition();
  }

  if(play){
    if(myStepper.targetPosition() == myStepper.currentPosition()){
      //If Continuous, flip the target and return
      if(continuous){
        stopped = true;
        if(rightToLeft){
          rightToLeft = false;
        }else{
          rightToLeft = true;
        }
        playSlider();
      }else{
        stopSlider();
      }
    }
    myStepper.runSpeed();
  }
}

void homeGantry(){
  if(!endstopsSet){
    setEndstops();
  }
  sendHome = true;
  if(rightToLeft){
    myStepper.moveTo(maxPosition);
  }else{
    myStepper.moveTo(0);
  }
}

void stopSlider(){
  stopped = true;
  play = false;
  Blynk.virtualWrite(V4, 1);
  homeGantry();
}

void playSlider(){
  if(stopped){
    targetPosition = maxPosition * (distancePercentage / 100.0);
    playSpeed = targetPosition / travelTimeInSecs;

    if(rightToLeft){
      targetPosition = maxPosition - targetPosition;
      playSpeed = -1 * playSpeed;
    }

    myStepper.moveTo(targetPosition);
    myStepper.setSpeed(playSpeed);
    delay(3000);
  }
  Blynk.virtualWrite(V4, 3);
  
  stopped = false;
  play = true;
}

void pauseSlider(){
  play = false;
  Blynk.virtualWrite(V4, 2);
}

void setEndstops(){
  Serial.println("Setting Endstops");
  setEndstop(-TRAVEL_SPEED, -REFINEMENT_SPEED, MIN_ENDSTOP_PIN);
  myStepper.setCurrentPosition(0);
  Serial.println("Setting current position to 0");
  Serial.println(myStepper.currentPosition());

  setEndstop(TRAVEL_SPEED, REFINEMENT_SPEED, MAX_ENDSTOP_PIN);
  maxPosition = myStepper.currentPosition();
  Serial.print("Max Position 1: ");
  Serial.println(myStepper.currentPosition());

  endstopsSet = true;
}

void setEndstop(int travelSpeed, int refinementSpeed, int pin){
  int lastButtonState = HIGH;
  unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
  myStepper.setSpeed(travelSpeed);
  while((millis() - lastDebounceTime) > debounceDelay){
    int reading = digitalRead(pin);
    // If the switch changed, due to noise or pressing:
    if (reading != lastButtonState) {
      // reset the debouncing timer
      lastDebounceTime = millis();
    }
    myStepper.runSpeed();
  }

  if(pin == MAX_ENDSTOP_PIN){
    maxPosition = myStepper.currentPosition(); //Set the maxPosition to the current position of the stepper.
    myStepper.moveTo(myStepper.currentPosition()-200); //One rotation backwards
  }else{
    myStepper.setCurrentPosition(0); //Set the current position to 0 for now
    myStepper.moveTo(200); //One rotation backwards
  }

  //Move back one rotation
  while(myStepper.distanceToGo() != 0){
    myStepper.run();
  }

  myStepper.setSpeed(refinementSpeed); //Reduce the speed;

  while((millis() - lastDebounceTime) > debounceDelay){
    int reading = digitalRead(pin);
    // If the switch changed, due to noise or pressing:
    if (reading != lastButtonState) {
      // reset the debouncing timer
      lastDebounceTime = millis();
    }
    myStepper.runSpeed();
  }
}
