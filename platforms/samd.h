#include <Arduino.h>
// 16 bits timer
#include <TimerTC3.h>

// forward declaration of ISR
void uCtrlHandler();

namespace uctrl {

// uses TimerTc3
#define ATOMIC(X) noInterrupts(); X; interrupts();

IntervalTimer _uclockTimer;

void initTimer(uint32_t init_clock)
{
    TimerTc3.initialize(init_clock);

    // attach to generic uclock ISR
    TimerTc3.attachInterrupt(uCtrlHandler);
}

}