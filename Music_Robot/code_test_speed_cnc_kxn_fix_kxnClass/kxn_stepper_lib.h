#ifndef kxn_stepper_lib_h__
#define kxn_stepper_lib_h__
#include "Arduino.h"

typedef enum
{
    DIRECTION_CCW = 0, ///< Counter-Clockwise
    DIRECTION_CW = 1   ///< Clockwise
} Direction;

class kxnStep
{
public:
    float _speed;
    float _maxSpeed;
    float _cmin; // at max speed

    /// The step counter for speed calculations
    long _n;

    /// The acceleration to use to accelerate or decelerate the motor in steps
    /// per second per second. Must be > 0
    float _acceleration;

    /// The target position in steps. The AccelStepper library will move the
    /// motor from the _currentPos to the _targetPos, taking into account the
    /// max speed, acceleration and deceleration
    long _targetPos; // Steps

    /// The current absolution position in steps.
    long _currentPos; // Steps

    /// The current interval between steps in microseconds.
    /// 0 means the motor is currently stopped with _speed == 0
    unsigned long _stepInterval;

    /// Current direction motor is spinning in
    /// Protected because some peoples subclasses need it to be so
    boolean _direction; // 1 == CW

    /// Last step size in microseconds
    float _cn;

    /// Initial step size in microseconds
    float _c0;

    /// The last step time in microseconds
    unsigned long _lastStepTime;

    /// Arduino pin number assignments for the 2 pins required to interface to the
    /// stepper motor or driver
    uint8_t _pin[2]; // 0-Step, 1-dir

    /// The minimum allowed pulse width in microseconds
    unsigned int   _minPulseWidth;

    kxnStep(uint8_t pa1, uint8_t pa2_steppin, uint8_t pa3_dirpin)
    {
        _minPulseWidth = 1; // us
        _pin[0] = pa2_steppin;
        _pin[1] = pa3_dirpin;

        pinMode(pa2_steppin, OUTPUT);
        pinMode(pa3_dirpin, OUTPUT);
    }

    void setSpeed(float speed)
    {
        if (speed == _speed)
            return;
        speed = constrain(speed, -_maxSpeed, _maxSpeed);
        if (speed == 0.0)
            _stepInterval = 0;
        else
        {
            _stepInterval = fabs(1000000.0 / speed);
            _direction = (speed > 0.0) ? DIRECTION_CW : DIRECTION_CCW;
        }
        _speed = speed;
    }

    void setMaxSpeed(float speed)
    {
        if (speed < 0.0)
            speed = -speed;
        if (_maxSpeed != speed)
        {
            _maxSpeed = speed;
            _cmin = 1000000.0 / speed;
            // Recompute _n from current speed and adjust speed if accelerating or cruising
            if (_n > 0)
            {
                _n = (long)((_speed * _speed) / (2.0 * _acceleration)); // Equation 16
                computeNewSpeed();
            }
        }
    }

    void setAcceleration(float acceleration)
    {
        if (acceleration == 0.0)
            return;
        if (acceleration < 0.0)
            acceleration = -acceleration;
        if (_acceleration != acceleration)
        {
            // Recompute _n per Equation 17
            _n = _n * (_acceleration / acceleration);
            // New c0 per Equation 7, with correction per Equation 15
            _c0 = 0.676 * sqrt(2.0 / acceleration) * 1000000.0; // Equation 15
            _acceleration = acceleration;
            computeNewSpeed();
        }
    }

    void setCurrentPosition(long position)
    {
        _targetPos = _currentPos = position;
        _n = 0;
        _stepInterval = 0;
        _speed = 0.0;
    }

    long distanceToGo()
    {
        return _targetPos - _currentPos;
    }

    unsigned long computeNewSpeed()
    {
        long distanceTo = distanceToGo(); // +ve is clockwise from curent location

        long stepsToStop = (long)((_speed * _speed) / (2.0 * _acceleration)); // Equation 16

        if (distanceTo == 0 && stepsToStop <= 1)
        {
            // We are at the target and its time to stop
            _stepInterval = 0;
            _speed = 0.0;
            _n = 0;
            return _stepInterval;
        }

        if (distanceTo > 0)
        {
            // We are anticlockwise from the target
            // Need to go clockwise from here, maybe decelerate now
            if (_n > 0)
            {
                // Currently accelerating, need to decel now? Or maybe going the wrong way?
                if ((stepsToStop >= distanceTo) || _direction == DIRECTION_CCW)
                    _n = -stepsToStop; // Start deceleration
            }
            else if (_n < 0)
            {
                // Currently decelerating, need to accel again?
                if ((stepsToStop < distanceTo) && _direction == DIRECTION_CW)
                    _n = -_n; // Start accceleration
            }
        }
        else if (distanceTo < 0)
        {
            // We are clockwise from the target
            // Need to go anticlockwise from here, maybe decelerate
            if (_n > 0)
            {
                // Currently accelerating, need to decel now? Or maybe going the wrong way?
                if ((stepsToStop >= -distanceTo) || _direction == DIRECTION_CW)
                    _n = -stepsToStop; // Start deceleration
            }
            else if (_n < 0)
            {
                // Currently decelerating, need to accel again?
                if ((stepsToStop < -distanceTo) && _direction == DIRECTION_CCW)
                    _n = -_n; // Start accceleration
            }
        }

        // Need to accelerate or decelerate
        if (_n == 0)
        {
            // First step from stopped
            _cn = _c0;
            _direction = (distanceTo > 0) ? DIRECTION_CW : DIRECTION_CCW;
        }
        else
        {
            // Subsequent step. Works for accel (n is +_ve) and decel (n is -ve).
            _cn = _cn - ((2.0 * _cn) / ((4.0 * _n) + 1)); // Equation 13
            _cn = max(_cn, _cmin);
        }
        _n++;
        _stepInterval = _cn;
        _speed = 1000000.0 / _cn;
        if (_direction == DIRECTION_CCW)
            _speed = -_speed;

#if 0
    Serial.println(_speed);
    Serial.println(_acceleration);
    Serial.println(_cn);
    Serial.println(_c0);
    Serial.println(_n);
    Serial.println(_stepInterval);
    Serial.println(distanceTo);
    Serial.println(stepsToStop);
    Serial.println("-----");
#endif
        return _stepInterval;
    }

    void move(long relative)
    {
        moveTo(_currentPos + relative);
    }

    void moveTo(long absolute)
    {
        if (_targetPos != absolute)
        {
            _targetPos = absolute;
            computeNewSpeed();
            // compute new n?
        }
    }

    void step1(long step)
    {
        (void)(step); // Unused

        // _pin[0] is step, _pin[1] is direction
        // setOutputPins(_direction ? 0b10 : 0b00); // Set direction first else get rogue pulses
        // setOutputPins(_direction ? 0b11 : 0b01); // step HIGH
        digitalWrite(_pin[1], _direction);
        digitalWrite(_pin[0], 1);

        // Caution 200ns setup time
        // Delay the minimum allowed pulse width
        // delayMicroseconds(_minPulseWidth);
        // setOutputPins(_direction ? 0b10 : 0b00); // step LOW
        digitalWrite(_pin[0], 0);
        // Serial.println(distanceToGo());
    }

    void step(long step)
    {
        step1(step);
    }

    boolean runSpeed()
    {
        // Dont do anything unless we actually have a step interval
        if (!_stepInterval)
            return false;

        unsigned long time = micros();
        if (time - _lastStepTime >= _stepInterval)
        {
            if (_direction == DIRECTION_CW)
            {
                // Clockwise
                _currentPos += 1;
            }
            else
            {
                // Anticlockwise
                _currentPos -= 1;
            }
            step(_currentPos);

            _lastStepTime = time; // Caution: does not account for costs in step()

            return true;
        }
        else
        {
            return false;
        }
    }

    void stop()
    {
        if (_speed != 0.0)
        {
            long stepsToStop = (long)((_speed * _speed) / (2.0 * _acceleration)) + 1; // Equation 16 (+integer rounding)
            if (_speed > 0)
                move(stepsToStop);
            else
                move(-stepsToStop);
        }
    }

    bool isRunning()
    {
        return !(_speed == 0.0 && _targetPos == _currentPos);
    }

    uint8_t run()
    {
        if (runSpeed())
            computeNewSpeed();
        return _speed != 0.0 || distanceToGo() != 0;
    }

    float acceleration()
    {
        return _acceleration;
    }
    float speed()
    {
        return _speed;
    }
    float maxSpeed()
    {
        return _maxSpeed;
    }
    long currentPosition()
    {
        return _currentPos;
    }
};

#endif