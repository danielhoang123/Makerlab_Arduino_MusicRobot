#include "kxnTask.h"

void kxnTask::kDelay(unsigned long paTime){
    interval = paTime;
  }

  uint8_t kxnTask::getState(){
    return taskState;
  }

  bool kxnTask::isStateStop(){
    return taskState == KXN_TASK_STATE_STOP;
  }

  bool kxnTask::isStateStart(){
    return taskState == KXN_TASK_STATE_START;
  }

  bool kxnTask::isStateIdle(){
    return taskState == KXN_TASK_STATE_IDLE;
  }

  void kxnTask::setState(uint8_t pa_state){
    lastTaskState = taskState;
    taskState = pa_state;
  }

  void kxnTask::setStateStop(){
    setState(KXN_TASK_STATE_STOP);
  }

  void kxnTask::setStateIdle(){
    taskState = KXN_TASK_STATE_IDLE;
  }

  void kxnTask::run(unsigned long pa_ulCurrentTime) {
    unsigned long currentMillis = pa_ulCurrentTime;

    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      switch (taskState) {
      case KXN_TASK_STATE_IDLE:
        break;
      default:
        loop();
      }
      
    }
  }

  kxnTaskManager kxnTaskManager1;

  void yield(){
    kxnTaskManager1.run(millis());
  }