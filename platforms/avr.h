#include <Arduino.h>

// forward declaration of uClockHandler
void uCtrlHandler();

namespace uctrl {

#define ATOMIC(X) noInterrupts(); X; interrupts();

// want a different avr clock support?
// TODO: we should do this using macro guards for avrs different clocks freqeuncy setup at compile time
#define AVR_CLOCK_FREQ	16000000

#if defined(__AVR_ATmega32U4__)	
ISR(TIMER3_COMPA_vect) 
#else
ISR(TIMER2_COMPA_vect) 
#endif
{
	uCtrlHandler();
}

void initTimer(uint32_t us_interval)
{
    // we always keep 250us default task time
	ATOMIC(
#if defined(__AVR_ATmega32U4__)	
		// avr general timer3 - 16bits
		TCCR3A = 0; // set entire TCCR1A register to 0
		TCCR3B = 0; // same for TCCR1B
		TCNT3  = 0; // initialize counter value to 0
		// set compare match register for 4000 Hz increments [250us]
		OCR3A = 3999; // = 16000000 / (1 * 4000) - 1 (must be <65536)
		// turn on CTC mode
		TCCR3B |= (1 << WGM32);
		// Set CS12, CS11 and CS10 bits for 1 prescaler
		TCCR3B |= (0 << CS32) | (0 << CS31) | (1 << CS30);
		// enable timer compare interrupt
		TIMSK3 |= (1 << OCIE3A);
#else // defined(__AVR_ATmega32U4__)	
		// avr general timer2 - 8bits
		TCCR2A = 0; // set entire TCCR2A register to 0
		TCCR2B = 0; // same for TCCR2B
		TCNT2  = 0; // initialize counter value to 0
		// set compare match register for 4000 Hz increments [250us]
		OCR2A = 124; // = 16000000 / (32 * 4000) - 1 (must be <256)
		// turn on CTC mode
		TCCR2A |= (1 << WGM21);
		// Set CS22, CS21 and CS20 bits for 32 prescaler
		TCCR2B |= (0 << CS22) | (1 << CS21) | (1 << CS20);
		// enable timer compare interrupt
		TIMSK2 |= (1 << OCIE2A);
		/* 
		we can make an option here for those who dont need a input clock sync 
		instead of running at 250us goes to 1ms will make the interface more responsive for 16mhz AVRs
		// 1000 Hz (16000000/((124+1)*128))
		OCR2A = 124;
		// turn on CTC mode
		TCCR2A |= (1 << WGM21);
		// Prescaler 128
		TCCR2B |= (1 << CS22) | (1 << CS20);
		*/
#endif // defined(__AVR_ATmega32U4__)	
	)
}

}