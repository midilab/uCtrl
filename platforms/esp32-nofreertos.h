#include <Arduino.h>

// forward declaration of uClockHandler
void uCtrlHandler();

namespace uctrl {

hw_timer_t * _uctrlTimer = NULL;
portMUX_TYPE _uctrlTimerMux = portMUX_INITIALIZER_UNLOCKED;
#define ATOMIC(X) portENTER_CRITICAL_ISR(&_uctrlTimerMux); X; portEXIT_CRITICAL_ISR(&_uctrlTimerMux);

// ISR handler
void ARDUINO_ISR_ATTR handlerISR(void)
{
    uCtrlHandler();
}

void initTimer(uint32_t init_clock)
{
    _uctrlTimer = timerBegin(init_clock);

    // attach to generic uclock ISR
    timerAttachInterrupt(_uctrlTimer, &handlerISR);

    // init clock tick time
    timerAlarm(_uctrlTimer, init_clock, true, 0); 
}

}