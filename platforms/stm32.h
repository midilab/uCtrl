#include <Arduino.h>

// forward declaration of ISR
void uCtrlHandler();

namespace uctrl {

#if !defined(STM32_CORE_VERSION) || (STM32_CORE_VERSION  < 0x01090000)
  #error "Due to API change, this library is compatible with STM32_CORE_VERSION >= 0x01090000. Please update/install stm32duino core."
#endif

#if defined(TIM1)
  TIM_TypeDef * TimerInstance = TIM1;
#else
  TIM_TypeDef * TimerInstance = TIM2;
#endif

// instantiate HardwareTimer object
HardwareTimer * _uclockTimer = new HardwareTimer(TimerInstance);

#define ATOMIC(X) noInterrupts(); X; interrupts();

void initTimer(uint32_t us_interval)
{
  _uclockTimer->setOverflow(us_interval, MICROSEC_FORMAT);
  _uclockTimer->attachInterrupt(uCtrlHandler);
  _uclockTimer->resume();
}

void setTimer(uint32_t us_interval)
{
  _uclockTimer->setOverflow(us_interval, MICROSEC_FORMAT);
  _uclockTimer->refresh();
}

}