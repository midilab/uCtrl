#include <Arduino.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// forward declaration of uClockHandler
void uCtrlHandler();

namespace uctrl {

#define TIMER_ID	1
hw_timer_t * _uctrlTimer = NULL;
// mutex control for ISR
//portMUX_TYPE _uctrlTimerMux = portMUX_INITIALIZER_UNLOCKED;
//#define ATOMIC(X) portENTER_CRITICAL_ISR(&_uctrlTimerMux); X; portEXIT_CRITICAL_ISR(&_uctrlTimerMux);

// FreeRTOS main clock task size in bytes
//#define CTRL_STACK_SIZE     2048
#define CTRL_STACK_SIZE     5*1024
//#define CTRL_STACK_SIZE     configMINIMAL_STACK_SIZE
#define CTRL_TASK_PRIORITY  tskIDLE_PRIORITY + 2
TaskHandle_t _taskHandle;
// mutex to protect the shared resource
SemaphoreHandle_t _mutex;
// mutex control for task
#define ATOMIC(X) xSemaphoreTake(_mutex, portMAX_DELAY); X; xSemaphoreGive(_mutex);

// ISR handler
void ARDUINO_ISR_ATTR handlerISR(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // send the notification to ctrlTask
    vTaskNotifyGiveFromISR(_taskHandle, &xHigherPriorityTaskWoken);
    
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// task for user clock process
void ctrlTask(void *pvParameters)
{
    while (1) {
        // wait for a notification from ISR
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        uCtrlHandler();
    }
}

void initTimer(uint32_t init_clock)
{
    // initialize the mutex for shared resource access
    _mutex = xSemaphoreCreateMutex();

    // create the ctrlTask
    xTaskCreate(ctrlTask, "ctrlTask", CTRL_STACK_SIZE, NULL, CTRL_TASK_PRIORITY, &_taskHandle);

    _uctrlTimer = timerBegin(TIMER_ID, 60, true);

    // attach to generic uclock ISR
    timerAttachInterrupt(_uctrlTimer, &handlerISR, false);

    // init clock tick time
    timerAlarmWrite(_uctrlTimer, init_clock, true); 

    // activate it!
    timerAlarmEnable(_uctrlTimer);
}

} // end namespace uctrl
