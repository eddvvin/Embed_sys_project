#ifndef RTOS_H
#define RTOS_H

#include <stdint.h>

#define RTOS_MAX_TASKS       8
#define RTOS_MAX_SEMAPHORES  4
#define RTOS_MAX_QUEUES      4
#define RTOS_QUEUE_SIZE      16

/* ============================================================
 * TASK STRUCTURE
 * ============================================================ */
typedef void (*TaskFunction_t)(void *param);

typedef enum {
    TASK_STATE_READY = 0,
    TASK_STATE_RUNNING,
    TASK_STATE_BLOCKED,
    TASK_STATE_SUSPENDED
} TaskState_t;

typedef struct {
    TaskFunction_t  taskFunc;
    void           *param;
    uint32_t        periodMs;
    uint32_t        elapsedMs;
    uint8_t         priority;
    TaskState_t     state;
    uint8_t         isUsed;
} RTOS_Task_t;

/* ============================================================
 * SEMAPHORE STRUCTURE
 * ============================================================ */
typedef struct {
    volatile uint8_t count;
    uint8_t maxCount;
    uint8_t isUsed;
} RTOS_Semaphore_t;

/* ============================================================
 * MESSAGE QUEUE STRUCTURE
 * ============================================================ */
typedef struct {
    uint32_t buffer[RTOS_QUEUE_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
    volatile uint8_t count;
    uint8_t isUsed;
} RTOS_Queue_t;

/* ============================================================
 * CORE RTOS FUNCTIONS
 * ============================================================ */

/**

 */
void RTOS_Init(uint32_t tickMs);

/**

 */
int RTOS_AddTask(TaskFunction_t func, void *param,
                 uint32_t periodMs, uint32_t startDelayMs,
                 uint8_t priority);

/**
 * @brief Run scheduler (call this in while(1) loop)
 */
void RTOS_RunScheduler(void);

/**
 * @brief Tick hook - called from SysTick_Handler
 */
void RTOS_Tick(void);

/**
 * @brief Suspend a task
 * @param taskId Task ID returned by RTOS_AddTask
 */
void RTOS_SuspendTask(uint8_t taskId);

/**
 * @brief Resume a suspended task
 * @param taskId Task ID returned by RTOS_AddTask
 */
void RTOS_ResumeTask(uint8_t taskId);

/**
 * @brief Get current system tick count
 * @return Current tick count in milliseconds
 */
uint32_t RTOS_GetTickCount(void);

/* ============================================================
 * SEMAPHORE FUNCTIONS
 * ============================================================ */

/**

 */
int RTOS_CreateSemaphore(uint8_t maxCount, uint8_t initialCount);

/**
 * @brief Take (decrement) a semaphore
 * @param semId Semaphore ID
 * @return 1 if successful, 0 if semaphore is zero
 */
uint8_t RTOS_SemaphoreTake(uint8_t semId);

/**
 * @brief Give (increment) a semaphore
 * @param semId Semaphore ID
 * @return 1 if successful, 0 if already at max
 */
uint8_t RTOS_SemaphoreGive(uint8_t semId);

/**
 * @brief Get current semaphore count
 * @param semId Semaphore ID
 * @return Current count value
 */
uint8_t RTOS_SemaphoreGetCount(uint8_t semId);

/* ============================================================
 * MESSAGE QUEUE FUNCTIONS
 * ============================================================ */

/**
 * @brief Create a message queue
 * @return Queue ID or -1 on failure
 */
int RTOS_CreateQueue(void);

/**
 * @brief Send a message to a queue
 * @param queueId Queue ID
 * @param data 32-bit data to send
 * @return 1 if successful, 0 if queue is full
 */
uint8_t RTOS_QueueSend(uint8_t queueId, uint32_t data);

/**
 * @brief Receive a message from a queue
 * @param queueId Queue ID
 * @param data Pointer to store received data
 * @return 1 if successful, 0 if queue is empty
 */
uint8_t RTOS_QueueReceive(uint8_t queueId, uint32_t *data);

/**
 * @brief Get number of items in queue
 * @param queueId Queue ID
 * @return Number of items in queue
 */
uint8_t RTOS_QueueCount(uint8_t queueId);

/**
 * @brief Check if queue is empty
 * @param queueId Queue ID
 * @return 1 if empty, 0 otherwise
 */
uint8_t RTOS_QueueIsEmpty(uint8_t queueId);



void RTOS_EnterCritical(void);

/**
 * @brief Exit critical section (enable interrupts)
 */
void RTOS_ExitCritical(void);

#endif // RTOS_H
