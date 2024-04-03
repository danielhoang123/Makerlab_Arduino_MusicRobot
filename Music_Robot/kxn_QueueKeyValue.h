/**
 * @file kxn_QueueKeyValue.h
 * @author your name (you@domain.com)
 * @brief 
 * @version kxn_QueueKeyValue_h_1.0.2
 * @date 2024-03-28
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef kxn_QueueKeyValue_h
#define kxn_QueueKeyValue_h

#include "Arduino.h"
// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>

#define STRUCT_QUEUE(QueueName) QueueName##Pair

#define DECLARE_QUEUE(QueueName, KeyType, ValueType, Size) \
typedef struct { \
    KeyType key; \
    ValueType value; \
} QueueName##Pair; \
extern QueueName##Pair QueueName##Array[];

#define DEFINE_QUEUE(QueueName, KeyType, ValueType, Size) \
QueueName##Pair QueueName##Array[Size]; \
int QueueName##Front = 0; \
int QueueName##Rear = -1; \
int QueueName##Count = 0;\
DEFINE_QUEUE_FUNCTIONS(QueueName, KeyType, ValueType, Size)

#define ENQUEUE(QueueName, key, value) enqueue_##QueueName(key, value)
#define DEQUEUE(QueueName) dequeue_##QueueName()
#define GETQUEUESIZE(QueueName) getQueueSize_##QueueName()

#define DEFINE_QUEUE_FUNCTIONS(QueueName, KeyType, ValueType, Size) \
void enqueue_##QueueName(KeyType key, ValueType value) { \
    if (QueueName##Count == Size) { \
        /*printf(#QueueName " array is full.\n"); */ \
    } else { \
        for (int i = 0; i < QueueName##Count; i++) { \
            /*if (QueueName##Array[(QueueName##Front + i) % Size].key == key) { \
                QueueName##Array[(QueueName##Front + i) % Size].value = value; \
                return; \
            } */ \
        } \
        QueueName##Rear = (QueueName##Rear + 1) % Size; \
        QueueName##Array[QueueName##Rear].key = key; \
        QueueName##Array[QueueName##Rear].value = value; \
        QueueName##Count++; \
    } \
} \
\
QueueName##Pair dequeue_##QueueName() { \
    if (QueueName##Count == 0) { \
        printf(#QueueName " array is empty.\n"); \
        QueueName##Pair kv = {-1, -1}; \
        return kv; \
    } else { \
        QueueName##Pair kv = QueueName##Array[QueueName##Front]; \
        QueueName##Front = (QueueName##Front + 1) % Size; \
        QueueName##Count--; \
        return kv; \
    } \
} \
\
int getQueueSize_##QueueName() { \
    return QueueName##Count; \
}



// DEFINE_QUEUE(Schedule, int, float, 3)

// // https://www.onlinegdb.com/online_c_compiler
// int QueueKeyValueTest() {
//     int key1 = 1, key2 = 2;
//     float value1 = 100.0, value2 = 200.0, value3 = 300.0;
//     ENQUEUE(Schedule, key1, value1);
//     ENQUEUE(Schedule, key2, value2);
//     ENQUEUE(Schedule, 1, 300.0);
//     ENQUEUE(Schedule, 4, 300.0);
//     ENQUEUE(Schedule, 5, 300.0);
//     ENQUEUE(Schedule, 6, 300.0);
//     ENQUEUE(Schedule, 7, 300.0);
//     ENQUEUE(Schedule, 8, 300.0);
//     ENQUEUE(Schedule, 9, 300.0);
//     ENQUEUE(Schedule, 10, 300.0);
//     ENQUEUE(Schedule, 11, 300.0);
    
//     printf("Queue size: %d\n", GETQUEUESIZE(Schedule));
//     SchedulePair kv = DEQUEUE(Schedule);
//     printf("Dequeued key: %d, value: %f\n", kv.key, kv.value);
//     printf("Queue size: %d\n", GETQUEUESIZE(Schedule));
    
//     ENQUEUE(Schedule, 12, 1200.0);
//     printf("Enqueued key: %d, value: %f\n", 12, 1200.00);
//     printf("Queue size: %d\n", GETQUEUESIZE(Schedule));
    
//     kv = DEQUEUE(Schedule);
//     printf("Dequeued key: %d, value: %f\n", kv.key, kv.value);
//     printf("Queue size: %d\n", GETQUEUESIZE(Schedule));
    
//     kv = DEQUEUE(Schedule);
//     printf("Dequeued key: %d, value: %f\n", kv.key, kv.value);
//     printf("Queue size: %d\n", GETQUEUESIZE(Schedule));
//     return 0;
// }

#endif