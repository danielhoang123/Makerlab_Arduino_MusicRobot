#pragma once

#include "kxnTask.h"
#include "AccelStepper_kxn.h"

CREATE_TASK_STATE(kxnStepApp)
kxnStepApp_STATE_START,
    kxnStepApp_STATE_STOP,
    END

    CREATE_TASK(kxnStepApp)

// Define a stepper and the pins it will use
AccelStepper *stepper;
// kxnStep stepper(MODE_DRIVER, STEP, DIR);

float stepSpeedV = 1000;  // 3500;//SPEED;
float stepSpeedVM = 2150; // 2150;//2200;//497000;//4000;//SPEED;
float stepSpeedA = 10000; // 250000;//3500;//SPEED/3;
float stepPos = 1500;     // 20000;//1600;

void setup()
{
  setStateStop();
  kDelay(0);
}

void loop()
{
  switch (getState())
  {
  case kxnStepApp_STATE_START:
    if (stepper->currentPosition() <= START)
    {

      stepper->moveTo(stepPos);
      setState(kxnStepApp_STATE_STOP);
    }
    break;

  case kxnStepApp_STATE_STOP:
    if (stepper->currentPosition() >= stepPos)
    {
      stepper->moveTo(START);
      setState(kxnStepApp_STATE_START);
    }
    break;
  }
}

void start()
{
  setState(kxnStepApp_STATE_START);
}

void addStep(AccelStepper *stepper_pa)
{
  this->stepper = stepper_pa;
}

void stop()
{
  this->stepper->stop();
  setStateIdle();
}

END