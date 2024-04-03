// ProportionalControl.pde
// -*- mode: C++ -*-
//
// Make a single stepper follow the analog value read from a pot or whatever
// The stepper will move at a constant speed to each newly set posiiton, 
// depending on the value of the pot.
//
// Copyright (C) 2012 Mike McCauley
// $Id: ProportionalControl.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $

/**
 * 
 * KXN test:
 *  1/16: maxSpeed = 14000; speed: 13000; a = 500, 6800 pulse pos; 0.2A
 *  1/8: maxSpeed = 7000; speed: 6800; a = 100, 3200 pulse pos; 0.2A~0.4A
 *  1/4: maxSpeed = 4000; speed: 3640; a = 50, 1600 pulse pos; 0.2A~0.4A
 *  1/1: maxSpeed = 2200; a = 10000, 1500 pulse pos; 0.2A~0.4A  
 *  1/1: maxSpeed = 2150; a = 10000, 1500 pulse pos; 0.2A~0.4A // 8/3/2024
 * 
 *  9/3/2024: Da chuyen C++ sang C, chay OK (kxn_stepper_lib_V3)
 * 
*/

#include "AccelStepper_kxn.h"
// #include "kxn_stepper_lib.h"
#include "kxn_stepper_lib_V3.h"
#include "var.h"
#include "kxnStepApp.h"


// Define a stepper and the pins it will use
AccelStepper stepper(MODE_DRIVER, STEP, DIR);
// kxnStep stepper(MODE_DRIVER, STEP, DIR);

float stepSpeedV = 1000;//3500;//SPEED;
float stepSpeedVM = 2150; //2150;//2200;//497000;//4000;//SPEED;
float stepSpeedA = 10000;//250000;//3500;//SPEED/3;
float stepPos = 1500;//20000;//1600;

kxnStepApp kxnStepApp1;

void setup()
{  
  Serial.begin(115200);
  Serial.println("Start Step");
  pinMode(PIN_HOME_SEN, INPUT_PULLUP);
  pinMode(EN, OUTPUT);
  digitalWrite(EN, 0);
  // pinMode(STEP, OUTPUT);
  // pinMode(DIR, OUTPUT);
  kxnStepApp1.stepPos = stepPos;
  kxnStepApp1.addStep(&stepper);
  kxnStepApp1.start();
  
  GotoHome();
  delay(2000);

  // Test_Stepper_C();

  showInfo();
}

void loop()
{
  CheckSerial();

  // if (stepper.currentPosition() <= START)
  // {
  //   delay(100);
  //   stepper.moveTo(stepPos);
  //   // stepper.setSpeed(stepSpeedV);
  // }
  // else if (stepper.currentPosition() >= stepPos)
  // {
  //   delay(100);
  //   stepper.moveTo(START);
  //   // stepper.setSpeed(stepSpeedV);
  // }

  
  kxnStepApp1.run(millis());

  // stepper.runSpeedToPosition();
  stepper.run();
  // 1/4   20000s: pos; 250000a;497000m




}

void CheckSerial(){
  if(Serial.available()){
    String kk = Serial.readStringUntil('\n');
    float value = kk.toFloat();
    Serial.println(kk);
    if(kk.indexOf("a") > -1){
      stepSpeedA = value;
      stepper.setAcceleration(stepSpeedA);
    }
    if(kk.indexOf("v") > -1){
      stepSpeedV = value;
      stepper.setSpeed(stepSpeedV);
    }
    if(kk.indexOf("m") > -1){
      stepSpeedVM = value;
      stepper.setMaxSpeed(stepSpeedVM);
    }
    if(kk.indexOf("s") > -1){
      stepPos = value;
    }
    if(kk.indexOf("b") > -1){
      kxnStepApp1.stop();
    }
    
    // showInfo();
    // stepper.moveTo(START);
  }
}

void showInfo(){
  Serial.println("Set a: " + String(stepper.acceleration()) + "\tspeed: " + String(stepper.speed()) + "\tspeed Max: " + String(stepper.maxSpeed()) + "\tCurrent Pos: " + String(stepper.currentPosition()));
}


void GotoHome(){
  stepper.setMaxSpeed(stepSpeedVM/3);
  stepper.setAcceleration(stepSpeedA);

  stepper.moveTo(-20000);

  while(digitalRead(PIN_HOME_SEN) != ACTIVE_HOME_SEN){
    stepper.run();
  }

  stepper.moveTo(20000);

  while(digitalRead(PIN_HOME_SEN) == ACTIVE_HOME_SEN){
    
    stepper.run();
  }

  stepper.setMaxSpeed(stepSpeedVM/3);
  stepper.setAcceleration(stepSpeedA);
  stepper.moveTo(-20000);
  while(digitalRead(PIN_HOME_SEN) != ACTIVE_HOME_SEN){
    
    stepper.run();
  }

  while(digitalRead(PIN_HOME_SEN) == ACTIVE_HOME_SEN){
    stepper.moveTo(20000);
    stepper.run();
  }

  stepper.setMaxSpeed(stepSpeedVM);
  stepper.setAcceleration(stepSpeedA);

  stepper.stop();
  stepper.setCurrentPosition(0);
}