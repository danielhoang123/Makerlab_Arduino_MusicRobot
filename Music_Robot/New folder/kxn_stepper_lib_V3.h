#ifndef kxn_stepper_lib_h__
#define kxn_stepper_lib_h__
// #include <stdio.h>
// #include <stdlib.h>

#include "Arduino.h"
#include "var.h"

typedef enum
{
    DIRECTION_CCW = 0,
    DIRECTION_CW = 1
} Direction;

typedef struct
{
    float _speed;
    float _maxSpeed;
    float _cmin;

    long _n;

    float _acceleration;

    long _targetPos;

    long _currentPos;

    unsigned long _stepInterval;

    int _direction;

    float _cn;

    float _c0;

    unsigned long _lastStepTime;

    uint8_t _pin[2];

    unsigned int _minPulseWidth;
} kxnStep;

unsigned long kxnStep_computeNewSpeed(kxnStep *stepper);
void kxnStep_moveTo(kxnStep *stepper, long absolute);

void kxnStep_init(kxnStep *stepper, uint8_t pa1, uint8_t pa2_steppin, uint8_t pa3_dirpin)
{
    stepper->_minPulseWidth = 1;
    stepper->_pin[0] = pa2_steppin;
    stepper->_pin[1] = pa3_dirpin;
    pinMode(pa2_steppin, OUTPUT);
    pinMode(pa3_dirpin, OUTPUT);
}

void kxnStep_setSpeed(kxnStep *stepper, float speed)
{
    if (speed == stepper->_speed)
        return;
    speed = constrain(speed, -stepper->_maxSpeed, stepper->_maxSpeed);
    if (speed == 0.0)
        stepper->_stepInterval = 0;
    else
    {
        stepper->_stepInterval = fabs(1000000.0 / speed);
        stepper->_direction = (speed > 0.0) ? DIRECTION_CW : DIRECTION_CCW;
    }
    stepper->_speed = speed;
}

void kxnStep_setMaxSpeed(kxnStep *stepper, float speed)
{
    if (speed < 0.0)
        speed = -speed;
    if (stepper->_maxSpeed != speed)
    {
        stepper->_maxSpeed = speed;
        stepper->_cmin = 1000000.0 / speed;

        if (stepper->_n > 0)
        {
            stepper->_n = (long)((stepper->_speed * stepper->_speed) / (2.0 * stepper->_acceleration));
            kxnStep_computeNewSpeed(stepper);
        }
    }
}

void kxnStep_setAcceleration(kxnStep *stepper, float acceleration)
{
    if (acceleration == 0.0)
        return;
    if (acceleration < 0.0)
        acceleration = -acceleration;
    if (stepper->_acceleration != acceleration)
    {

        stepper->_n = stepper->_n * (stepper->_acceleration / acceleration);

        stepper->_c0 = 0.676 * sqrt(2.0 / acceleration) * 1000000.0;
        stepper->_acceleration = acceleration;
        kxnStep_computeNewSpeed(stepper);
    }
}

void kxnStep_setCurrentPosition(kxnStep *stepper, long position)
{
    stepper->_targetPos = stepper->_currentPos = position;
    stepper->_n = 0;
    stepper->_stepInterval = 0;
    stepper->_speed = 0.0;
}

long kxnStep_distanceToGo(kxnStep *stepper)
{
    return stepper->_targetPos - stepper->_currentPos;
}

unsigned long kxnStep_computeNewSpeed(kxnStep *stepper)
{
    long distanceTo = kxnStep_distanceToGo(stepper); // +ve is clockwise from curent location

    long stepsToStop = (long)((stepper->_speed * stepper->_speed) / (2.0 * stepper->_acceleration)); // Equation 16

    if (distanceTo == 0 && stepsToStop <= 1)
    {
        // We are at the target and its time to stop
        stepper->_stepInterval = 0;
        stepper->_speed = 0.0;
        stepper->_n = 0;
        return stepper->_stepInterval;
    }

    if (distanceTo > 0)
    {
        // We are anticlockwise from the target
        // Need to go clockwise from here, maybe decelerate now
        if (stepper->_n > 0)
        {
            // Currently accelerating, need to decel now? Or maybe going the wrong way?
            if ((stepsToStop >= distanceTo) || stepper->_direction == DIRECTION_CCW)
                stepper->_n = -stepsToStop; // Start deceleration
        }
        else if (stepper->_n < 0)
        {
            // Currently decelerating, need to accel again?
            if ((stepsToStop < distanceTo) && stepper->_direction == DIRECTION_CW)
                stepper->_n = -stepper->_n; // Start accceleration
        }
    }
    else if (distanceTo < 0)
    {
        // We are clockwise from the target
        // Need to go anticlockwise from here, maybe decelerate
        if (stepper->_n > 0)
        {
            // Currently accelerating, need to decel now? Or maybe going the wrong way?
            if ((stepsToStop >= -distanceTo) || stepper->_direction == DIRECTION_CW)
                stepper->_n = -stepsToStop; // Start deceleration
        }
        else if (stepper->_n < 0)
        {
            // Currently decelerating, need to accel again?
            if ((stepsToStop < -distanceTo) && stepper->_direction == DIRECTION_CCW)
                stepper->_n = -stepper->_n; // Start accceleration
        }
    }

    // Need to accelerate or decelerate
    if (stepper->_n == 0)
    {
        // First step from stopped
        stepper->_cn = stepper->_c0;
        stepper->_direction = (distanceTo > 0) ? DIRECTION_CW : DIRECTION_CCW;
    }
    else
    {
        // Subsequent step. Works for accel (n is +_ve) and decel (n is -ve).
        stepper->_cn = stepper->_cn - ((2.0 * stepper->_cn) / ((4.0 * stepper->_n) + 1)); // Equation 13
        stepper->_cn = max(stepper->_cn, stepper->_cmin);
    }
    stepper->_n++;
    stepper->_stepInterval = stepper->_cn;
    stepper->_speed = 1000000.0 / stepper->_cn;
    if (stepper->_direction == DIRECTION_CCW)
        stepper->_speed = -stepper->_speed;
    Serial.println(stepper->_stepInterval);
    return stepper->_stepInterval;
}

void kxnStep_move(kxnStep *stepper, long relative)
{
    kxnStep_moveTo(stepper, stepper->_currentPos + relative);
}

void kxnStep_moveTo(kxnStep *stepper, long absolute)
{
    if (stepper->_targetPos != absolute)
    {
        stepper->_targetPos = absolute;
        kxnStep_computeNewSpeed(stepper);
        // compute new n?
    }
}

void kxnStep_step1(kxnStep *stepper, long step)
{
    digitalWrite(stepper->_pin[1], stepper->_direction);
    digitalWrite(stepper->_pin[0], 1);
    // delay(1);
    digitalWrite(stepper->_pin[0], 0);
}

void kxnStep_step(kxnStep *stepper, long step)
{
    kxnStep_step1(stepper, step);
}

boolean kxnStep_runSpeed(kxnStep *stepper)
{
    // Dont do anything unless we actually have a step interval
    if (!stepper->_stepInterval)
        return false;

    unsigned long time = micros();
    if (time - stepper->_lastStepTime >= stepper->_stepInterval)
    {
        if (stepper->_direction == DIRECTION_CW)
        {
            // Clockwise
            stepper->_currentPos += 1;
        }
        else
        {
            // Anticlockwise
            stepper->_currentPos -= 1;
        }
        kxnStep_step(stepper, stepper->_currentPos);

        stepper->_lastStepTime = time; // Caution: does not account for costs in step()

        return true;
    }
    else
    {
        return false;
    }
}

void kxnStep_stop(kxnStep *stepper)
{
    if (stepper->_speed != 0.0)
    {
        long stepsToStop = (long)((stepper->_speed * stepper->_speed) / (2.0 * stepper->_acceleration)) + 1; // Equation 16 (+integer rounding)
        if (stepper->_speed > 0)
            kxnStep_move(stepper, stepsToStop);
        else
            kxnStep_move(stepper, -stepsToStop);
    }
}

bool kxnStep_isRunning(kxnStep *stepper)
{
    return !(stepper->_speed == 0.0 && stepper->_targetPos == stepper->_currentPos);
}

uint8_t kxnStep_run(kxnStep *stepper)
{
    if (kxnStep_runSpeed(stepper))
        kxnStep_computeNewSpeed(stepper);
    return stepper->_speed != 0.0 || kxnStep_distanceToGo(stepper) != 0;
}

float kxnStep_acceleration(kxnStep *stepper)
{
    return stepper->_acceleration;
}

float kxnStep_speed(kxnStep *stepper)
{
    return stepper->_speed;
}

float kxnStep_maxSpeed(kxnStep *stepper)
{
    return stepper->_maxSpeed;
}

long kxnStep_currentPosition(kxnStep *stepper)
{
    return stepper->_currentPos;
}

#endif

kxnStep testStep;

void Test_Stepper_C()
{

    kxnStep_init(&testStep, 0, STEP, DIR);
    kxnStep_setAcceleration(&testStep, 10000);
    kxnStep_setMaxSpeed(&testStep, 2200);

    while (1)
    {
        if (kxnStep_currentPosition(&testStep) <= START)
        {
            delay(100);
            kxnStep_moveTo(&testStep, 1500);
            Serial.println("AAAA");
            // kxnStep_setSpeed(stepSpeedV);
        }
        else if (kxnStep_currentPosition(&testStep) >= 1500)
        {
            delay(100);
            kxnStep_moveTo(&testStep, START);
            Serial.println("1500");
            // kxnStep_setSpeed(stepSpeedV);
        }

        // kxnStep_runSpeedToPosition();
        kxnStep_run(&testStep);
    }
}
