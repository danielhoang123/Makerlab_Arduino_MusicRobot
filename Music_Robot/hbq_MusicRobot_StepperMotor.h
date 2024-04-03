#ifndef hbq_MusicRobot_StepperMotor_H
#define hbq_MusicRobot_StepperMotor_H

#include "AccelStepper.h"
#include "var.h"
#include "Arduino.h"
#include "kxnTask.h"

CREATE_TASK_STATE(MusicRobot_StepperMotor)
STEPPER_START_RUNNING,
    STEPPER_WHILE_RUNNING,
    STEPPER_FINISH_RUNNING,
    STEPPER_IDLE
END

CREATE_TASK(MusicRobot_StepperMotor)

AccelStepper *p_musicStepper;

float stepSpeedNormal = 1000; // 3500;//SPEED;
float stepSpeedMax = 2000;    // 2200;//497000;//4000;//SPEED;
float stepAccel = 20000;      // 250000;//3500;//SPEED/3        // 20000;//1600;
int stepPos = 1650;
int temp_Position;

//**********************SETUP**************************************************//
void addStepper(AccelStepper *pa_Stepper);

void setup(uint8_t pa_EN_PIN, uint8_t pa_LIMIT_SWITCH);

void start();

bool isStepperFinishRunning();

int setPosition(int pa_Position);
//*******************TESTING STATE MACHINE*************************************//
void loop();

private:

bool finishRunning = 0;

END

// typedef enum {
//     STEPPER_SET = 0,
//     STEPPER_START,
//     STEPPER_STOP,
//     STEPPER_IDLE
// }stepper_State;

// class MusicRobot_StepperMotor
// {
// public:

//     AccelStepper musicStepper;
//     stepper_State myStepperState;

//     float stepSpeedNormal = 1000; // 3500;//SPEED;
//     float stepSpeedMax = 3000;    // 2200;//497000;//4000;//SPEED;
//     float stepAccel = 20000;       // 250000;//3500;//SPEED/3        // 20000;//1600;
//     int temp_Position;
//     float temp_Speed;
//     float temp_Accel;

//     void init(uint8_t MODE, uint8_t pa_STEP_PIN, uint8_t pa_DIR_PIN);
//     void set_Position_Coordinate_Origin();
//     void position_Testing(float pa_Position);
//     void stepper_Run();

//     //*******************TESTING STATE MACHINE*************************************//
//     void stepper_Processing(uint8_t pa_Position);
//     uint8_t getState();
//     void setState(stepper_State pa_StepperState);

// private:
// };
#endif