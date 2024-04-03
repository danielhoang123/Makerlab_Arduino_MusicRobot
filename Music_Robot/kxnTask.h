#ifndef kxnTask_H__
#define kxnTask_H__

#include "Arduino.h"

typedef enum {
  KXN_TASK_STATE_START = 250,
  KXN_TASK_STATE_STOP,
  KXN_TASK_STATE_IDLE
} enumStatekxnTask;

class kxnTask {
private:

  // Variables will change:
  uint8_t taskState = KXN_TASK_STATE_IDLE;      // taskState used to set the LED
  uint8_t lastTaskState = KXN_TASK_STATE_IDLE;  // taskState used to set the LED

  // Generally, you should use "unsigned long" for variables that hold time
  // The value will quickly become too large for an int to store
  unsigned long previousMillis = 0;  // will store last time LED was updated

  unsigned long interval = 50;

public:

  // Set time delay when change state
  void kDelay(unsigned long paTime);

  uint8_t getState();

  bool isStateStop();

  bool isStateStart();

  bool isStateIdle();

  void setState(uint8_t pa_state);

  void setStateStop();

  void setStateIdle();

  virtual void stop() {
    setStateStop();
  }

  virtual void start() {
    setState(KXN_TASK_STATE_START);
  }

  virtual void setup() {}

  virtual void loop() {}

  void run(unsigned long pa_ulCurrentTime);
};

#define CREATE_TASK_STATE(nameClass) typedef enum enumState##nameClass {
#define CREATE_TASK(nameClass) \ 
  class nameClass : public kxnTask { \
  public:
#define END };

//-------------------------------------------------------------------------------------------------------------

class NodeKxnTask {
public:
    kxnTask* task;
    NodeKxnTask* next;

    NodeKxnTask(kxnTask* task) : task(task), next(NULL) {}
};

class kxnTaskManager {
private:
    NodeKxnTask* head;
public:
    kxnTaskManager() : head(NULL) {}

    void add(kxnTask* task) {
        NodeKxnTask* newNode = new NodeKxnTask(task);
        newNode->next = head;
        head = newNode;
    }

    void run(unsigned long CurrentTime) {
        NodeKxnTask* temp = head;
        while (temp != NULL) {
            temp->task->run(CurrentTime);
            temp = temp->next;
        }
    }
};

extern kxnTaskManager kxnTaskManager1;

#endif