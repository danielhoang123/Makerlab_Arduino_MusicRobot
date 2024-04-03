#ifndef hbq_MusicRobot_StepperServo_Controller_H
#define hbq_MusicRobot_StepperServo_Controller_H

#include "hbq_MusicRobot_Servo.h"
#include "hbq_MusicRobot_StepperMotor.h"
#include "kxnTask.h"
#include "var.h"
#include "debug.h"

CREATE_TASK_STATE(StepperServo_Controller)

CONTROLLER_STEPPER_START,
CONTROLLER_STEPPER_STOP,
CONTROLLER_SERVO_START,
CONTROLLER_SERVO_STOP,
CONTROLLER_IDLE

END

CREATE_TASK(StepperServo_Controller)

MusicRobot_Servo * Servo;

MusicRobot_StepperMotor * Stepper;

uint16_t tempNote;

void begin(MusicRobot_StepperMotor * pa_Stepper, MusicRobot_Servo * pa_Servo);

void start();

void musicStart(uint16_t note);

void loop();

END

#endif