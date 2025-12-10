/**
 ******************************************************************************
 * @file    rtos.c
 * @brief   Custom RTOS implementation for STM32
 * @details Priority-based cooperative scheduler with semaphores and queues
 ******************************************************************************
 */

#include "rtos.h"
#include "stm32f4xx.h"


static RTOS_Task_t      tasks[RTOS_MAX_TASKS];
static RTOS_Semaphore_t semaphores[RTOS_MAX_SEMAPHORES];
static RTOS_Queue_t     queues[RTOS_MAX_QUEUES];

static uint32_t rtosTickMs = 1;
static volatile uint32_t systemTickCount = 0;
static uint8_t  numTasks = 0;
static uint8_t  currentTaskId = 0;

// rtos initialization
void RTOS_Init(uint32_t tickMs)
{
    rtosTickMs = tickMs;
    SystemCoreClockUpdate();

    // Configure SysTick to fire every tickMs
    uint32_t ticksPerSec = 1000U / tickMs;
    SysTick_Config(SystemCoreClock / ticksPerSec);

    // Set SysTick to lowest priority
    NVIC_SetPriority(SysTick_IRQn, 15);

    // Initialize all tasks
    for (uint8_t i = 0; i < RTOS_MAX_TASKS; i++) {
        tasks[i].isUsed    = 0;
        tasks[i].taskFunc  = NULL;
        tasks[i].param     = NULL;
        tasks[i].periodMs  = 0;
        tasks[i].elapsedMs = 0;
        tasks[i].priority  = 255;
        tasks[i].state     = TASK_STATE_READY;
    }

    // Initialize all semaphores
    for (uint8_t i = 0; i < RTOS_MAX_SEMAPHORES; i++) {
        semaphores[i].count    = 0;
        semaphores[i].maxCount = 0;
        semaphores[i].isUsed   = 0;
    }

    // Initialize all queues
    for (uint8_t i = 0; i < RTOS_MAX_QUEUES; i++) {
        queues[i].head    = 0;
        queues[i].tail    = 0;
        queues[i].count   = 0;
        queues[i].isUsed  = 0;
    }

    numTasks = 0;
    currentTaskId = 0;
    systemTickCount = 0;
}

// task managment
int RTOS_AddTask(TaskFunction_t func, void *param,
                 uint32_t periodMs, uint32_t startDelayMs,
                 uint8_t priority)
{
    if (numTasks >= RTOS_MAX_TASKS || func == NULL) {
        return -1;
    }

    tasks[numTasks].taskFunc  = func;
    tasks[numTasks].param     = param;
    tasks[numTasks].periodMs  = periodMs;
    tasks[numTasks].priority  = priority;
    tasks[numTasks].state     = TASK_STATE_READY;
    tasks[numTasks].isUsed    = 1;

    // Start so it first fires after startDelayMs
    if (startDelayMs < periodMs) {
        tasks[numTasks].elapsedMs = periodMs - startDelayMs;
    } else {
        tasks[numTasks].elapsedMs = 0;
    }

    numTasks++;
    return (int)(numTasks - 1);
}

void RTOS_SuspendTask(uint8_t taskId)
{
    if (taskId < RTOS_MAX_TASKS && tasks[taskId].isUsed) {
        tasks[taskId].state = TASK_STATE_SUSPENDED;
    }
}

void RTOS_ResumeTask(uint8_t taskId)
{
    if (taskId < RTOS_MAX_TASKS && tasks[taskId].isUsed) {
        tasks[taskId].state = TASK_STATE_READY;
    }
}

uint32_t RTOS_GetTickCount(void)
{
    return systemTickCount;
}

// scheduler
void RTOS_RunScheduler(void)
{
    // Find highest priority ready task that needs to run
    int8_t nextTask = -1;
    uint8_t highestPriority = 255;

    for (uint8_t i = 0; i < numTasks; i++) {
        RTOS_Task_t *t = &tasks[i];

        if (!t->isUsed || t->taskFunc == NULL) {
            continue;
        }

        if (t->state != TASK_STATE_READY) {
            continue;
        }

        // Check if task is ready to run based on period
        if (t->elapsedMs >= t->periodMs) {
            // Select highest priority task
            if (t->priority < highestPriority) {
                highestPriority = t->priority;
                nextTask = i;
            }
        }
    }

    // Execute the selected task
    if (nextTask >= 0) {
        RTOS_Task_t *t = &tasks[nextTask];
        currentTaskId = nextTask;
        t->state = TASK_STATE_RUNNING;

        // Execute task function
        t->taskFunc(t->param);

        // Reset elapsed time after execution
        t->elapsedMs = 0;
        t->state = TASK_STATE_READY;
    }
}

// tick handler
void RTOS_Tick(void)
{
    systemTickCount += rtosTickMs;

    // Update all task timers
    for (uint8_t i = 0; i < numTasks; i++) {
        RTOS_Task_t *t = &tasks[i];
        if (!t->isUsed) continue;

        // Only increment timer for ready tasks
        if (t->state == TASK_STATE_READY) {
            t->elapsedMs += rtosTickMs;
        }
    }
}

// semaphore functions
int RTOS_CreateSemaphore(uint8_t maxCount, uint8_t initialCount)
{
    for (uint8_t i = 0; i < RTOS_MAX_SEMAPHORES; i++) {
        if (!semaphores[i].isUsed) {
            semaphores[i].maxCount = maxCount;
            semaphores[i].count = (initialCount <= maxCount) ? initialCount : maxCount;
            semaphores[i].isUsed = 1;
            return i;
        }
    }
    return -1;
}

uint8_t RTOS_SemaphoreTake(uint8_t semId)
{
    if (semId >= RTOS_MAX_SEMAPHORES || !semaphores[semId].isUsed) {
        return 0;
    }

    RTOS_EnterCritical();

    if (semaphores[semId].count > 0) {
        semaphores[semId].count--;
        RTOS_ExitCritical();
        return 1;
    }

    RTOS_ExitCritical();
    return 0;
}

uint8_t RTOS_SemaphoreGive(uint8_t semId)
{
    if (semId >= RTOS_MAX_SEMAPHORES || !semaphores[semId].isUsed) {
        return 0;
    }

    RTOS_EnterCritical();

    if (semaphores[semId].count < semaphores[semId].maxCount) {
        semaphores[semId].count++;
        RTOS_ExitCritical();
        return 1;
    }

    RTOS_ExitCritical();
    return 0;
}

uint8_t RTOS_SemaphoreGetCount(uint8_t semId)
{
    if (semId >= RTOS_MAX_SEMAPHORES || !semaphores[semId].isUsed) {
        return 0;
    }
    return semaphores[semId].count;
}

// message queue functions
int RTOS_CreateQueue(void)
{
    for (uint8_t i = 0; i < RTOS_MAX_QUEUES; i++) {
        if (!queues[i].isUsed) {
            queues[i].head = 0;
            queues[i].tail = 0;
            queues[i].count = 0;
            queues[i].isUsed = 1;
            return i;
        }
    }
    return -1;
}

uint8_t RTOS_QueueSend(uint8_t queueId, uint32_t data)
{
    if (queueId >= RTOS_MAX_QUEUES || !queues[queueId].isUsed) {
        return 0;
    }

    RTOS_EnterCritical();

    RTOS_Queue_t *q = &queues[queueId];

    if (q->count >= RTOS_QUEUE_SIZE) {
        RTOS_ExitCritical();
        return 0; // Queue full
    }

    q->buffer[q->tail] = data;
    q->tail = (q->tail + 1) % RTOS_QUEUE_SIZE;
    q->count++;

    RTOS_ExitCritical();
    return 1;
}

uint8_t RTOS_QueueReceive(uint8_t queueId, uint32_t *data)
{
    if (queueId >= RTOS_MAX_QUEUES || !queues[queueId].isUsed || data == NULL) {
        return 0;
    }

    RTOS_EnterCritical();

    RTOS_Queue_t *q = &queues[queueId];

    if (q->count == 0) {
        RTOS_ExitCritical();
        return 0; // Queue empty
    }

    *data = q->buffer[q->head];
    q->head = (q->head + 1) % RTOS_QUEUE_SIZE;
    q->count--;

    RTOS_ExitCritical();
    return 1;
}

uint8_t RTOS_QueueCount(uint8_t queueId)
{
    if (queueId >= RTOS_MAX_QUEUES || !queues[queueId].isUsed) {
        return 0;
    }
    return queues[queueId].count;
}

uint8_t RTOS_QueueIsEmpty(uint8_t queueId)
{
    if (queueId >= RTOS_MAX_QUEUES || !queues[queueId].isUsed) {
        return 1;
    }
    return (queues[queueId].count == 0) ? 1 : 0;
}


void RTOS_EnterCritical(void)
{
    __disable_irq();
}

void RTOS_ExitCritical(void)
{
    __enable_irq();
}


