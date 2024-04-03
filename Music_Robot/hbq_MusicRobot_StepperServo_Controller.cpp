#include "hbq_MusicRobot_StepperServo_Controller.h"

void StepperServo_Controller::begin(MusicRobot_StepperMotor *pa_Stepper, MusicRobot_Servo *pa_Servo)
{
    this->Stepper = pa_Stepper;
    this->Servo = pa_Servo;
}

void StepperServo_Controller::musicStart(uint16_t note)
{
    tempNote = note;
    setState(CONTROLLER_STEPPER_START);
}

void StepperServo_Controller::loop()
{

    switch (getState())
    {
    case CONTROLLER_STEPPER_START:

        DEBUG("CONTROLLER_STEPPER_START");

        Stepper->setPosition(tempNote);

        Stepper->start();

        setState(CONTROLLER_STEPPER_STOP);

        break;

    case CONTROLLER_STEPPER_STOP:

        DEBUG("CONTROLLER_STEPPER_STOP");

        if (Stepper->isStateIdle() == true)
        {
            DEBUG("CONTROLLER_STEPPER_IDLE");

            setState(CONTROLLER_SERVO_START);
        }

        break;

    case CONTROLLER_SERVO_START:

        Servo->start();

        setState(CONTROLLER_SERVO_STOP);

        DEBUG("CONTROLLER_SERVO_START");

        break;

    case CONTROLLER_SERVO_STOP:

        if (Servo->isStateIdle() == true)
        {
            setStateIdle();
            
            DEBUG("CONTROLLER_SERVO_STOP");
        }
        break;
    default:
        break;
    }
    // Stepper->setPosition(tempNote);

    // if(Stepper->isStepperIdle())
}

void StepperServo_Controller::start()
{
    setState(CONTROLLER_STEPPER_START);
}