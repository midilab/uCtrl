/*!
 *  @file       uCtrl.cpp
 *  Project     Arduino Library API interface for uMODULAR projects
 *  @brief      ...
 *  @version    1.1.0
 *  @author     Romulo Silva
 *  @date       30/10/22
 *  @license    MIT - (c) 2022 - Romulo Silva - contact@midilab.co
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE. 
 */

#include "uCtrl.h"

// 
// Timer setup for work clock
//
// Teensyduino port
//
#if defined(TEENSYDUINO)
	IntervalTimer _uctrlTimer;
#endif
//
// Seedstudio XIAO M0 port
//
#if defined(SEEED_XIAO_M0)
	// 16 bits timer
	#include <TimerTC3.h>
	// uses TimerTc3
#endif
//
// ESP32 family
//
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
	hw_timer_t * _uctrlTimer = NULL;
	#define TIMER_ID	1
#endif

//
// multicore archs
//
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
	portMUX_TYPE _uctrlTimerMux = portMUX_INITIALIZER_UNLOCKED;
	#define ATOMIC(X) portENTER_CRITICAL_ISR(&_uctrlTimerMux); X; portEXIT_CRITICAL_ISR(&_uctrlTimerMux);
//
// singlecore archs
//
#else
	#define ATOMIC(X) noInterrupts(); X; interrupts();
#endif

//
// Generic AVR timer
#if defined(ARDUINO_ARCH_AVR)
void enableTimer()
{
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
#else
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
#endif
	)
}
// ARM timers
#else

	// forward declaration of ISR
	#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
		void ARDUINO_ISR_ATTR ucrtISR();
	#else
		void ucrtISR();
	#endif
	
void enableTimer()
{
	#if defined(TEENSYDUINO)
		_uctrlTimer.begin(ucrtISR, 250);
		// Set the interrupt priority level, controlling which other interrupts
		// this timer is allowed to interrupt. Lower numbers are higher priority, 
		// with 0 the highest and 255 the lowest. Most other interrupts default to 128. 
		// As a general guideline, interrupt routines that run longer should be given 
		// lower priority (higher numerical values).
		_uctrlTimer.priority(80);
	#endif

	#if defined(SEEED_XIAO_M0)
		TimerTc3.initialize(250);

		// attach to generic uclock ISR
		TimerTc3.attachInterrupt(ucrtISR);
	#endif

	#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
		_uctrlTimer = timerBegin(TIMER_ID, 80, true);

		// attach to generic uclock ISR
		timerAttachInterrupt(_uctrlTimer, &ucrtISR, true);

		// init clock tick time
		timerAlarmWrite(_uctrlTimer, 250, true); 

		// activate it!
		timerAlarmEnable(_uctrlTimer);
	#endif
}
#endif

namespace uctrl {

uCtrlClass::uCtrlClass()
{
	on250usCallback = nullptr;
	on1msCallback = nullptr;
}

// Not called on arduino
uCtrlClass::~uCtrlClass()
{

}

#ifdef USE_EXT_RAM
bool uCtrlClass::initRam(SPIClass * device, uint8_t chip_select)
{
	if ( ram == nullptr ) {
		ram = &ram_module;
	}
	
	if ( ram != nullptr ) {
		ram->init(device, chip_select);
		return true;
	} else {
		return false;
	}
}
#endif

#ifdef USE_SDCARD
bool uCtrlClass::initSdCard(SPIClass * spi_device, uint8_t chip_select)
{
	if ( sdcard == nullptr ) {
		sdcard = &sdcard_module;
	}
	
	if ( sdcard != nullptr ) {
		sdcard->init(spi_device, chip_select);
		return true;
	} else {
		return false;
	}	
}
#endif

#ifdef USE_DEVICE
bool uCtrlClass::initDevice(uint8_t device_number, uint16_t event_buffer_size, uint8_t sysex_buffer_size, uint16_t device_label_buffer_size)
{
	if ( device == nullptr ) {
		device = &device_module;
	}
	
	if ( device != nullptr ) {
		device->init(device_number, event_buffer_size, sysex_buffer_size, device_label_buffer_size);
		return true;
	} else {
		return false;
	}
}
#endif

#ifdef USE_PAGE
bool uCtrlClass::initPage()
{
	if ( page == nullptr ) {
		page = &page_module;
	}
	
	if ( page != nullptr ) {
		page->init();
		return true;
	} else {
		return false;
	}
}
#endif

#ifdef USE_OLED
#ifdef USE_OLED_U8G2
bool uCtrlClass::initOled(U8G2 * display)
#else
bool uCtrlClass::initOled(U8X8 * display)
#endif
{
	if ( oled == nullptr ) {
		oled = &oled_module;
	}
	
	if ( oled != nullptr ) {
		// plug display
		oled->plug(display);
		return true;
	} else {
		return false;
	}
}

#ifdef USE_EXT_RAM
#ifdef USE_DEVICE
void uCtrlClass::processDisplay()
{
	if ( device->showDataFeedback() == true ) {
		oled->setDisplayLockState(false);
		device->dataFeedbackCallback();
		oled->setDisplayLockState(true);
		if ( ((millis() - device->getDataFeedbackTimeout()) >= 1000) && device->getCtrlMode() != 2 ) {
			device->setDataFeedback(false);
			oled->setDisplayLockState(false);
			oled->clearDisplay(1,1,1);				
		}			
	}
}
#endif 
#endif 
#endif 

#ifdef USE_MIDI
bool uCtrlClass::initMidi()
{
	if ( midi == nullptr ) {
		midi = &midi_module;
	}
	
	if ( midi != nullptr ) {
		return true;
	} else {
		return false;
	}
}

void uCtrlClass::processMidi()
{
	uint8_t size = midi->sizeOf();
	for ( uint8_t port=1; port <= size; port++ ) {
		midi->read(port);
	}
}
#endif

#ifdef USE_DOUT
bool uCtrlClass::initDout(SPIClass * spi_device, uint8_t chip_select)
{
	if ( dout == nullptr ) {
		dout = &dout_module;
	}
	
	if ( dout != nullptr ) {
#if defined(USE_DOUT_SPI_DRIVER)		
		if (spi_device != nullptr) {
			dout->setSpi(spi_device, chip_select);
		}
#endif
		return true;
	} else {
		return false;
	}
}
#endif

#ifdef USE_DIN
bool uCtrlClass::initDin(SPIClass * spi_device, uint8_t chip_select)
{
	if ( din == nullptr ) {
		din = &din_module;
	}
	
	if ( din != nullptr ) {
#if defined(USE_DIN_SPI_DRIVER)		
		if (spi_device != nullptr) {
			din->setSpi(spi_device, chip_select);
		}
#endif
		return true;
	} else {
		return false;
	}
}
#endif

#ifdef USE_AIN
bool uCtrlClass::initAin(uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4)
{
	if ( ain == nullptr ) {
		ain = &ain_module;
	}
	
	if ( ain != nullptr ) {
// #ifdef USE_AIN_4067
#ifdef USE_AIN_4051
		ain->setMuxPins(pin1, pin2, pin3, pin4);
#endif
		return true;
	} else {
		return false;
	}
}

void uCtrlClass::processAin()
{
	uint8_t size_of_ports;
	int16_t value;
	uint8_t port;
	
	size_of_ports = ain->sizeOf();

	// 
	for ( port=0; port < size_of_ports; port++ ) {

#ifdef USE_DEVICE
		value = ain->getData(port, device->getCtrlAdcMin(port), device->getCtrlAdcMax(port));
#else
		value = ain->getData(port);
#endif
	
		if ( value > -1 ) {

#ifdef USE_DEVICE
			// make midi signal smooth as posible
			if ( device->handleAnalogEvent(port+1, value, 1) == true ) {
				//continue;
			}
#else
			// ain callback is processed inside a timmer interrupt, so always be short inside it!
			if ( ain->callback != nullptr ) {
				ain->callback(port+1, value, 1);
				continue;
			}   
#endif

			// add event to non interrupted queue in case no device control setup
			uint8_t tail = (_ain_event_queue.tail+1) >= _ain_event_queue.size ? 0 : (_ain_event_queue.tail+1);
			if ( _ain_event_queue.head != tail )
			{
				_ain_event_queue.event[_ain_event_queue.tail].port = port;
				_ain_event_queue.event[_ain_event_queue.tail].value = value;  
				_ain_event_queue.tail = tail; 
			}    

		}
		
	}
}
#endif

#ifdef USE_CAP_TOUCH
bool uCtrlClass::initCapTouch(uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4)
{
	if ( touch == nullptr ) {
		touch = &cap_touch_module;
	}
	
	if ( touch != nullptr ) {
		touch->setControlPins(pin1, pin2, pin3, pin4);
		return true;
	} else {
		return false;
	}
}
#endif

void uCtrlClass::init()
{
	// init of hardware configuration
	// if we have sdcard support, then retrive hardware config info from ucontrol.cfg on sdcard root filesystem
#ifdef USE_SDCARD
	// load default config
#else
	// load factory defaults
#endif

#ifdef USE_AIN
	ain->init();
	_ain_event_queue.head = 0;
	_ain_event_queue.tail = 0;
	_ain_event_queue.size = 8;		
#endif

#ifdef USE_CAP_TOUCH
	touch->init();
#endif

#ifdef USE_DIN
	din->init();
#endif
	
#ifdef USE_DOUT
	dout->init();
#endif
			
#ifdef USE_PAGE
	// default page and subpge
	if (page->getPageSize() > 0) {
		page->setPage(0);
		page->setSubPage(0);
	}
#endif
	
	// ...
	enableTimer();
}

void uCtrlClass::run()
{
	int16_t value;
	uint8_t port, head;
	uint8_t port_ref = 0;
	bool discard_ain_data = false;

	// timmer dependent UI visual effects
	uint32_t time = millis();
#ifdef USE_DOUT
	uCtrl.dout->setTimer(time);
#endif
#ifdef USE_OLED
	uCtrl.oled->setTimer(time);
#endif

#ifdef USE_OLED
#ifdef USE_OLED_U8G2
	oled->clearDisplay();
#endif
#endif

#ifdef USE_DIN 
	// din
	// read while empty
	while ( din->event_queue.head != din->event_queue.tail )
	{
		port = din->event_queue.event[din->event_queue.head].port;
		value = din->event_queue.event[din->event_queue.head].value; 
		head = (din->event_queue.head+1)%(din->event_queue.size);
		ATOMIC(
			din->event_queue.head = head
		)

		// +1 for user interface
		++port;

#ifdef USE_DEVICE
	   if ( device->handleDigitalEvent(port, value, 0) == true ) {
			continue;
	   }
#endif

#ifdef USE_PAGE
	#ifdef USE_AIN 
		// before each processEvent we need to: check if pot_ctrl is needed
		if(page->_use_nav_pot) {
			// if it is, check if it is inc or dec commands... 
			if (port == page->_nav_ctrl_port.incrementer || 
				port == page->_nav_ctrl_port.decrementer ||
				port == page->_nav_ctrl_port.incrementer_secondary || 
				port == page->_nav_ctrl_port.decrementer_secondary ||
				port == page->_nav_ctrl_port.up || 
				port == page->_nav_ctrl_port.down ||
				port == page->_nav_ctrl_port.left || 
				port == page->_nav_ctrl_port.right) {
				// if it is. lock ain pot control to avoid mess with inc/dec changes
				ain->lockControl(page->_nav_ctrl_port.pot-1);
				// we also need to remove from our event queue any data from nav pot
				discard_ain_data = true;
			}
		}
	#endif
		page->processEvent(port, value, uctrl::page::DIGITAL_EVENT);
#endif
	   if ( din->callback != nullptr ) {
			din->callback(port, value);
	   }
	}
	// set port_ref in case other digital modules were initialized
	port_ref = din->sizeOf();
#endif

#ifdef USE_CAP_TOUCH
	// touch
	// read while empty
	while ( touch->event_queue.head != touch->event_queue.tail )
	{
		// use port_ref in case din were initialized
		port = touch->event_queue.event[touch->event_queue.head].port+port_ref;
		value = touch->event_queue.event[touch->event_queue.head].value; 
		head = (touch->event_queue.head+1)%(touch->event_queue.size);
		ATOMIC(
			touch->event_queue.head = head
		)

		// +1 for user interface
		++port;

#ifdef USE_DEVICE
	   if ( device->handleDigitalEvent(port, value, 0) == true ) {
			continue;
	   }
#endif                 

#ifdef USE_PAGE
	#ifdef USE_AIN 
		// before each processEvent we need to: check if pot_ctrl is needed
		if(page->_use_nav_pot) {
			// if it is, check if it is inc or dec commands... 
			if (port == page->_nav_ctrl_port.incrementer || 
				port == page->_nav_ctrl_port.decrementer ||
				port == page->_nav_ctrl_port.incrementer_secondary || 
				port == page->_nav_ctrl_port.decrementer_secondary ||
				port == page->_nav_ctrl_port.up || 
				port == page->_nav_ctrl_port.down ||
				port == page->_nav_ctrl_port.left || 
				port == page->_nav_ctrl_port.right) {
				// if it is. lock ain pot control to avoid mess with inc/dec changes
				ain->lockControl(page->_nav_ctrl_port.pot-1);
				// we also need to remove from our event queue any data from nav pot
				discard_ain_data = true;
			}
		}
	#endif
		page->processEvent(port, (uint16_t)value, uctrl::page::DIGITAL_EVENT);
#endif
		if ( touch->callback != nullptr ) {
			touch->callback(port, value);
		}
	}
#endif

#ifdef USE_AIN 
   // ain:
   // read while empty
   while ( _ain_event_queue.head != _ain_event_queue.tail )
   {
		port = _ain_event_queue.event[_ain_event_queue.head].port;
		value = _ain_event_queue.event[_ain_event_queue.head].value;  
		head = (_ain_event_queue.head+1) >= _ain_event_queue.size ? 0 : (_ain_event_queue.head+1);
		ATOMIC(                      
			_ain_event_queue.head = head
		)

		// +1 for user interface
		++port;

		if (discard_ain_data) {
			if (port == page->_nav_ctrl_port.pot) {
				if (ain->isLocked(page->_nav_ctrl_port.pot-1) == false) {
					discard_ain_data = false;
				} else {
					continue;
				}
			}
		}
#ifdef USE_DEVICE
			// device process are done inside interrupt to keep smooth for realtime controllers events
			// EDIT MODE HANDLER
			if ( device->getCtrlMode() == 2 ) {
				device->setupCtrl(port, value);
			}

			if ( device->handleAnalogEvent(port, value, 0) == true ) {
				continue;
			}
#endif

#ifdef USE_PAGE
		page->processEvent(port, value, uctrl::page::ANALOG_EVENT);
#endif
		if ( ain->callback != nullptr ) {
			ain->callback(port, value, 0);
		}
	}

#endif         
     
#ifdef USE_PAGE                
	if ( page != nullptr ) {
#ifdef USE_PAGE_COMPONENT
		page->clearComponentMap();
#endif
		page->processView();
	}
#endif         
#ifdef USE_OLED
#ifdef USE_EXT_RAM
#ifdef USE_DEVICE
    processDisplay();
#endif 
#endif 
#ifdef USE_OLED_U8G2
	oled->refreshDisplay();
#endif
#endif 

#ifdef USE_DOUT
	uCtrl.dout->flushBuffer();
#endif                 	
}

uint8_t uCtrlClass::getAnalogPorts()
{

#ifdef USE_AIN
	return ain->sizeOf();
#endif

	return 0;
}

uint8_t uCtrlClass::getDigitalPorts()
{

#ifdef USE_DIN
	return din->sizeOf();
#endif

	return 0;
}

uint8_t uCtrlClass::getOutputPorts()
{

#ifdef USE_MIDI
	return midi->sizeOf();
#endif

	return 0;
	
/*
#ifdef UMODULAR_DMX		

#endif
	
#ifdef UMODULAR_CV	

#endif
*/
		
}

}

uctrl::uCtrlClass uCtrl;

//
// 250 microseconds base interrupt for
// realtime processment tasks(midi input, scan input/output modules...)
//
// priority on avr are handle by using nested
// interrupts(ISR_NOBLOCK) along with uClock.
// if you are not using timming critical interruption
// you can delete the ISR_NOBLOCK
//
// priority on teensy is at 80, while uClock
// keeps it at maximun priority 0(max prio).
// if you are not using timming critical interruption
// you can set the priority to 0(max prio)
//
uint8_t _timerCounter1ms = 0;
uint8_t _timerCounterAin = 0;
uint8_t _timerCapTouch = 0;
uint8_t _timerCounterDin = 0;
uint8_t _timerCounterDout = 0;

#if defined(ARDUINO_ARCH_AVR)
	#if defined(__AVR_ATmega32U4__)	
ISR(TIMER3_COMPA_vect, ISR_NOBLOCK) 
	#else
ISR(TIMER2_COMPA_vect, ISR_NOBLOCK) 
	#endif
#elif defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
void ARDUINO_ISR_ATTR ucrtISR()
#else
void ucrtISR() 
#endif
{
	if (uCtrl.on250usCallback) {
		uCtrl.on250usCallback();
	}

	if (uCtrl.on1msCallback) {
		// 1ms call
		if(++_timerCounter1ms == 4) {
			_timerCounter1ms = 0;
			uCtrl.on1msCallback();
			return;
		}
	}

#ifdef USE_DIN
	// 2ms call
	if (++_timerCounterDin == 8) {
		_timerCounterDin = 0;
		uCtrl.din->read(1);
		return;
	}
#endif

#ifdef USE_CAP_TOUCH	
	// 3ms call
	if (++_timerCapTouch == 12) 
	{
		_timerCapTouch = 0;
		uCtrl.touch->read();
		return;
	}
#endif

#ifdef USE_AIN	
	// 10ms call
	if (++_timerCounterAin == 40) 
	{
		_timerCounterAin = 0;
		uCtrl.processAin();
		return;
	}
#endif

#ifdef USE_DOUT
	// 30ms call
	if (++_timerCounterDout == 120) {
		_timerCounterDout = 0;
		uCtrl.dout->flush();
		return;
	}
#endif

}
