#include "rtos.h"
#include "stm32f4xx.h"   // CMSIS + device header

static RTOS_Task_t tasks[RTOS_MAX_TASKS];
static uint32_t    rtosTickMs = 1;
static uint8_t     numTasks   = 0;

// Initialize RTOS and SysTick
void RTOS_Init(uint32_t tickMs)
{
    rtosTickMs = tickMs;

    // Ensure core clock is up-to-date
    SystemCoreClockUpdate();

    // Configure SysTick to interrupt every tickMs
    // tickMs MUST divide 1000 for this simple formula.
    uint32_t ticksPerSec = 1000U / tickMs;
    SysTick_Config(SystemCoreClock / ticksPerSec);

    // Optional: set SysTick interrupt priority (low)
    NVIC_SetPriority(SysTick_IRQn, 15);

    // Clear task table
    for (uint8_t i = 0; i < RTOS_MAX_TASKS; i++) {
        tasks[i].isUsed    = 0;
        tasks[i].taskFunc  = 0;
        tasks[i].param     = 0;
        tasks[i].periodMs  = 0;
        tasks[i].elapsedMs = 0;
    }
    numTasks = 0;
}

// Add a task to the scheduler
int RTOS_AddTask(TaskFunction_t func, void *param,
                 uint32_t periodMs, uint32_t startDelayMs)
{
    if (numTasks >= RTOS_MAX_TASKS) {
        return -1; // no space
    }

    tasks[numTasks].taskFunc  = func;
    tasks[numTasks].param     = param;
    tasks[numTasks].periodMs  = periodMs;
    tasks[numTasks].elapsedMs = (periodMs - startDelayMs);
    tasks[numTasks].isUsed    = 1;

    numTasks++;
    return (int)(numTasks - 1);
}

// Scheduler: call from while(1) loop
void RTOS_RunScheduler(void)
{
    for (uint8_t i = 0; i < numTasks; i++) {
        RTOS_Task_t *t = &tasks[i];
        if (!t->isUsed || t->taskFunc == 0) {
            continue;
        }

        // If enough time elapsed, run task
        if (t->elapsedMs >= t->periodMs) {
            t->taskFunc(t->param);
            t->elapsedMs = 0;
        }
    }
}
