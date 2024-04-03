#include "hbq_MusicRobot_Servo.h"

void MusicRobot_Servo::addServo(Servo *p_pa_Servo)
{
    MusicServo = p_pa_Servo;
}

void MusicRobot_Servo::setup(uint8_t pa_ServoPin)
{
    MusicServo->attach(pa_ServoPin);
    MusicServo->write(35);
}

void MusicRobot_Servo::start()
{
    setState(SERVO_STATE_PRESS);
}

void MusicRobot_Servo::loop()
{
    switch (getState())
    {
    case SERVO_STATE_PRESS:
        MusicServo->write(servoPosEnd);
        kDelay(servoDelay);
        setState(SERVO_STATE_RELEASE);
        break;
    case SERVO_STATE_RELEASE:
        MusicServo->write(servoPosIdle);
        kDelay(0);
        setState(SERVO_STATE_IDLE);
        break;
    case SERVO_STATE_IDLE:
        setStateIdle();
    default:
        break;
    }
}

void MusicRobot_Servo::set_Servo_Position(uint8_t pa_Servo_PositionStart, uint8_t pa_Servo_Position_End, uint8_t pa_Delay)
{

    servoPosIdle = pa_Servo_PositionStart;
    servoPosEnd = pa_Servo_Position_End;
    servoDelay = pa_Delay;
    setState(SERVO_STATE_PRESS);
}

bool MusicRobot_Servo::isServoIdle()
{
    if (getState() == SERVO_STATE_IDLE)
    {
        return true;
    }
    else
    {
        return false;
    }
}
