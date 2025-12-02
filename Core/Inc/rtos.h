#ifndef RTOS_H
#define RTOS_H

#include <stdint.h>

#define RTOS_MAX_TASKS   8

typedef void (*TaskFunction_t)(void *param);

typedef struct {
    TaskFunction_t  taskFunc;
    void           *param;
    uint32_t        periodMs;      // How often to run
    uint32_t        elapsedMs;     // Time since last run
    uint8_t         isUsed;
} RTOS_Task_t;

void RTOS_Init(uint32_t tickMs);
int  RTOS_AddTask(TaskFunction_t func, void *param,
                  uint32_t periodMs, uint32_t startDelayMs);
void RTOS_RunScheduler(void);

// SysTick handler will be implemented in rtos.c
void SysTick_Handler(void);

#endif // RTOS_H
