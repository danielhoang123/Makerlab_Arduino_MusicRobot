#include "hbq_MusicRobot_StepperMotor.h"
#include "hbq_MusicRobot_Servo.h"
#include "hbq_MusicRobot_StepperServo_Controller.h"
#include "kxn_QueueKeyValue.h"

AccelStepper mainStepper(MODE_DRIVER, STEP_PIN, DIR_PIN);

MusicRobot_StepperMotor myStepperController;

Servo mainServo;

MusicRobot_Servo myServoController;

StepperServo_Controller myController;

//**************************TEST QUEUE******************************//
DECLARE_QUEUE(Schedule, int, uint16_t, 39)

DEFINE_QUEUE(Schedule, int, uint16_t, 39)

STRUCT_QUEUE(Schedule) kv;

unsigned long queueStartMillis = 0;

int queueInterval = 500;

float oneNote = 750; 

void setup()
{

    Serial.begin(9600);

    myServoController.addServo(&mainServo);

    myServoController.setup(SERVO_PIN);

    myStepperController.addStepper(&mainStepper);

    myStepperController.setup(EN_PIN, Z_ENDSTOP_PIN);

    myStepperController.start();

    myController.begin(&myStepperController, &myServoController);

    //**********************TEST QUEUE****************************************************//

    testQueue();

    myServoController.servoPosEnd = 28;
    myServoController.servoPosIdle = 35;
    myServoController.servoDelay = 100;
}

void loop()
{
    unsigned long currentMillis = millis();

    CheckSerial();

    myStepperController.run(currentMillis);

    myServoController.run(currentMillis);

    myController.run(currentMillis);

    mainStepper.run();

    runQueue();
}

void CheckSerial()
{
    if (Serial.available())
    {
        String kk = Serial.readStringUntil('\n');
        Serial.println(kk);

        if (kk.indexOf("G1") > -1)
        {
            myStepperController.setPosition(G1_KEY);
            myStepperController.start();
        }

        if (kk.indexOf("A1") > -1)
        {
            myStepperController.setPosition(A1_KEY);
            myStepperController.start();
        }

        if (kk.indexOf("G2") > -1)
        {
            myStepperController.setPosition(G2_KEY);
            myStepperController.start();
        }
        if (kk.indexOf("A2") > -1)
        {
            myStepperController.setPosition(A2_KEY);
            myStepperController.start();
        }
        if (kk.indexOf("B2") > -1)
        {
            myStepperController.setPosition(B2_KEY);
            myStepperController.start();
        }
        if (kk.indexOf("C2") > -1)
        {
            myStepperController.setPosition(C2_KEY);
            myStepperController.start();
        }
        if (kk.indexOf("C1") > -1)
        {
            myStepperController.setPosition(G1_KEY);
            myStepperController.start();
        }
        if (kk.indexOf("G3") > -1)
        {
            myStepperController.setPosition(G3_KEY);
            myStepperController.start();
        }

        if (kk.indexOf("RC") > -1) // RC*90#40.
        {
            uint8_t start = myServoController.splitString(kk, "RC", " ", " ", 0).toInt();

            Serial.println(start);

            uint8_t end = myServoController.splitString(kk, "RC", " ", " ", 1).toInt();

            Serial.println(end);

            uint8_t delay1 = myServoController.splitString(kk, "RC", " ", " ", 2).toInt();

            Serial.println(delay1);

            myServoController.set_Servo_Position(start, end, delay1);
        }

        if (kk.indexOf("K") > -1)
        {
            // myController.musicStart(G1_KEY);
            runQueue();
        }

        if (kk.indexOf("L") > -1)
        {
            myController.musicStart(G2_KEY);
        }

        if (kk.indexOf("M") > -1)
        {
            myController.musicStart(G3_KEY);
        }
    }
}

void testQueue()
{
    ENQUEUE(Schedule, G2_KEY, oneNote);
    ENQUEUE(Schedule, A2_KEY, oneNote);
    ENQUEUE(Schedule, B2_KEY, oneNote);
    ENQUEUE(Schedule, G2_KEY, oneNote);
    ENQUEUE(Schedule, G2_KEY, oneNote);

    ENQUEUE(Schedule, A2_KEY, oneNote);
    ENQUEUE(Schedule, B2_KEY, oneNote);
    ENQUEUE(Schedule, G2_KEY, oneNote);

    ENQUEUE(Schedule, B2_KEY, oneNote);
    ENQUEUE(Schedule, C3_KEY, oneNote);
    ENQUEUE(Schedule, D3_KEY, oneNote*2);
    ENQUEUE(Schedule, B2_KEY, oneNote);
    ENQUEUE(Schedule, C3_KEY, oneNote);
    ENQUEUE(Schedule, D3_KEY, oneNote*2);

    ENQUEUE(Schedule, D3_KEY, oneNote);
    ENQUEUE(Schedule, E3_KEY, oneNote);
    ENQUEUE(Schedule, D3_KEY, oneNote);
    ENQUEUE(Schedule, C3_KEY, oneNote);
    ENQUEUE(Schedule, B2_KEY, oneNote*2);
    ENQUEUE(Schedule, G2_KEY, oneNote*2);

    ENQUEUE(Schedule, D3_KEY, oneNote);
    ENQUEUE(Schedule, E3_KEY, oneNote);
    ENQUEUE(Schedule, D3_KEY, oneNote);
    ENQUEUE(Schedule, C3_KEY, oneNote);
    ENQUEUE(Schedule, B2_KEY, oneNote*2);
    ENQUEUE(Schedule, G2_KEY, oneNote*2);

    ENQUEUE(Schedule, G2_KEY, oneNote);
    ENQUEUE(Schedule, D2_KEY, oneNote);
    ENQUEUE(Schedule, G2_KEY, oneNote*2);

    ENQUEUE(Schedule, G2_KEY, oneNote);
    ENQUEUE(Schedule, D2_KEY, oneNote);
    ENQUEUE(Schedule, G2_KEY, oneNote*2);

    // Serial.println("Queue size" + String(GETQUEUESIZE(Schedule)));
}

void runQueue()
{
    unsigned long queueMillis = millis();

    if (queueMillis - queueStartMillis >= queueInterval)
    {
        queueStartMillis = queueMillis;

        if (myController.isStateIdle())
        {
            if (GETQUEUESIZE(Schedule) > 0)
            {
                kv = DEQUEUE(Schedule);

                queueInterval = kv.value;

                // Serial.println("Hello");

                myController.musicStart(kv.key);

                // Serial.println(kv.key);

                // Serial.println(kv.value);
            }
        }
    }
}