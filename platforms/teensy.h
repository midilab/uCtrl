#include <Arduino.h>

// forward declaration of ISR
void uCtrlHandler();

namespace uctrl {

#define ATOMIC(X) noInterrupts(); X; interrupts();

IntervalTimer _uclockTimer;

void initTimer(uint32_t init_clock)
{
    _uclockTimer.begin(uCtrlHandler, init_clock); 

    // Set the interrupt priority level, controlling which other interrupts
    // this timer is allowed to interrupt. Lower numbers are higher priority, 
    // with 0 the highest and 255 the lowest. Most other interrupts default to 128. 
    // As a general guideline, interrupt routines that run longer should be given 
    // lower priority (higher numerical values).
    _uclockTimer.priority(255);
}

void setTimer(uint32_t us_interval)
{
    _uclockTimer.update(us_interval);
}

}