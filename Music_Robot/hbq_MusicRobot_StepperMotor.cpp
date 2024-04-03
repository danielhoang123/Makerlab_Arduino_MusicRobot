// #include "hbq_MusicRobot_StepperMotor.h"

// void MusicRobot_StepperMotor::setState(enumStateMusicRobot_StepperMotor pa_StepperState)
// {
//     myStepperState = pa_StepperState;
// }

// uint8_t MusicRobot_StepperMotor::getState()
// {
//     return myStepperState;
// }

// void MusicRobot_StepperMotor::init(uint8_t MODE, uint8_t pa_STEP_PIN, uint8_t pa_DIR_PIN)
// {
//     AccelStepper musicStepper(MODE, pa_STEP_PIN, pa_DIR_PIN);

//     pinMode(Z_ENDSTOP_PIN, INPUT_PULLUP);
//     pinMode(EN_PIN, OUTPUT);
//     digitalWrite(EN_PIN, LOW);
// }

// void MusicRobot_StepperMotor::set_Position_Coordinate_Origin()
// {
//     // state 1
//     musicStepper.setMaxSpeed(stepSpeedMax / 2);
//     musicStepper.setAcceleration(stepAccel);
//     musicStepper.moveTo(-20000);
//     while (digitalRead(Z_ENDSTOP_PIN) != ACTIVE_HOME)
//     {
//         musicStepper.run();
//     }
//     musicStepper.moveTo(20000);
//     while (digitalRead(Z_ENDSTOP_PIN) == ACTIVE_HOME)
//     {
//         musicStepper.run();
//     }

//     // state 2
//     musicStepper.setMaxSpeed(stepSpeedMax);
//     musicStepper.setAcceleration(stepAccel);
//     musicStepper.stop();
//     musicStepper.setCurrentPosition(0);
// }

// void MusicRobot_StepperMotor::position_Testing(float pa_Position)
// {

//     if (musicStepper.currentPosition() <= pa_Position)
//     {
//         musicStepper.moveTo(pa_Position);
//     }
// }

// void MusicRobot_StepperMotor::stepper_Run()
// {
//     musicStepper.run();
// }

// void MusicRobot_StepperMotor::stepper_Processing(uint8_t pa_Position)
// {
//     if (myStepperState == STEPPER_SET)
//     {

//         temp_Position = pa_Position;

//         musicStepper.moveTo(temp_Position);

//         myStepperState = STEPPER_START;
//     }

//     else if (myStepperState == STEPPER_START)
//     {
//         musicStepper.run();

//         if (musicStepper.currentPosition() == temp_Position)
//         {
//             myStepperState = STEPPER_STOP;
//         }
//     }

//     else if (myStepperState == STEPPER_STOP)
//     {
//         musicStepper.stop();
//         myStepperState = STEPPER_IDLE;
//     }

//     else if (myStepperState == STEPPER_IDLE)
//     {
//         ;
//         ;
//     }
// }

//----------------------------------------------------------------------------------------------------------------
#include "hbq_MusicRobot_StepperMotor.h"

void MusicRobot_StepperMotor::addStepper(AccelStepper *pa_Stepper)
{
    this->p_musicStepper = pa_Stepper;
}

void MusicRobot_StepperMotor::setup(uint8_t pa_EN_PIN, uint8_t pa_LIMIT_SWITCH)
{
    // EN pin go to LOW
    pinMode(pa_EN_PIN, OUTPUT);
    digitalWrite(pa_EN_PIN, LOW);

    // Setup limit switch pin
    pinMode(pa_LIMIT_SWITCH, INPUT_PULLUP);

    // state 1
    this->p_musicStepper->setMaxSpeed(stepSpeedMax / 3);
    this->p_musicStepper->setAcceleration(stepAccel);
    this->p_musicStepper->moveTo(-10000);
    while (digitalRead(Z_ENDSTOP_PIN) != ACTIVE_HOME)
    {
        this->p_musicStepper->run();
    }
    this->p_musicStepper->moveTo(10000);
    while (digitalRead(Z_ENDSTOP_PIN) == ACTIVE_HOME)
    {
        this->p_musicStepper->run();
    }

    // state 2
    this->p_musicStepper->setMaxSpeed(stepSpeedMax);
    this->p_musicStepper->setAcceleration(stepAccel);
    this->p_musicStepper->stop();
    this->p_musicStepper->setCurrentPosition(START_POINT);
}

void MusicRobot_StepperMotor::start()
{
    setState(STEPPER_START_RUNNING);
}

int MusicRobot_StepperMotor::setPosition(int pa_Position)
{
    stepPos = pa_Position;
}

bool MusicRobot_StepperMotor::isStepperFinishRunning()
{
    if (finishRunning == 1)
    {
        Serial.println("Hehe");
        return true;
    }
    else
    {
        return false;
    }
}

void MusicRobot_StepperMotor::loop()
{
    switch (getState())
    {
    case STEPPER_START_RUNNING:

        this->p_musicStepper->moveTo(stepPos);

        setState(STEPPER_WHILE_RUNNING);

        // if (this->p_musicStepper->currentPosition() != stepPos)
        // {
        //     this->p_musicStepper->moveTo(stepPos);
        // }
        // else
        // {
        //     setState(STEPPER_IDLE);
        // }
        break;

    case STEPPER_WHILE_RUNNING:

        if (this->p_musicStepper->currentPosition() == stepPos)
        {
            setStateIdle();
        }

        break;

    // case STEPPER_FINISH_RUNNING:

    //     finishRunning = 1;

    //     setState(STEPPER_IDLE);

    //     break;

    // case STEPPER_IDLE:

    //     setStateIdle();

    //     break;
    default:
        break;
    }
}
