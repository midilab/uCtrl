#include <Arduino.h>

// forward declaration of ISR
void ARDUINO_ISR_ATTR ucrtISR();

namespace uctrl {

#define TIMER_ID	1

hw_timer_t * _uclockTimer = NULL;
portMUX_TYPE _uclockTimerMux = portMUX_INITIALIZER_UNLOCKED;
#define ATOMIC(X) portENTER_CRITICAL_ISR(&_uclockTimerMux); X; portEXIT_CRITICAL_ISR(&_uclockTimerMux);

void initTimer(uint32_t init_clock)
{
    _uclockTimer = timerBegin(TIMER_ID, 60, true);

    // attach to generic uclock ISR
    timerAttachInterrupt(_uclockTimer, &ucrtISR, false);

    // init clock tick time
    timerAlarmWrite(_uclockTimer, init_clock, true); 

    // activate it!
    timerAlarmEnable(_uclockTimer);
}

}