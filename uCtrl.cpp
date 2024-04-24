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
// General Arduino AVRs port
//
#if defined(ARDUINO_ARCH_AVR)
    #include "platforms/avr.h"
#endif
//
// Teensyduino ARMs port
//
#if defined(TEENSYDUINO)
    #include "platforms/teensy.h"
#endif
//
// Seedstudio XIAO M0 port
//
#if defined(SEEED_XIAO_M0)
    #include "platforms/samd.h"
#endif
//
// ESP32 family
//
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
    #include "platforms/esp32.h"
#endif
//
// STM32XX family
//
#if defined(ARDUINO_ARCH_STM32)
    #include "platforms/stm32.h"
#endif

namespace uctrl {

//
// Platform specific timer setup/control
//
// initTimer(uint32_t us_interval) and setTimer(uint32_t us_interval)
// are called from architecture specific module included at the
// header of this file
void enableTimer()
{
    // begin at 250us task
    initTimer(250);
}

uCtrlClass::uCtrlClass()
{
	on250usCallback = nullptr;
	on1msCallback = nullptr;
}

// Not called on arduino
uCtrlClass::~uCtrlClass()
{

}

//#if defined(USE_EXT_RAM)
bool uCtrlClass::initRam(SPIClass * device, uint8_t chip_select, bool is_shared)
{
	if ( ram == nullptr ) {
		ram = &ram_module;
	}
	
	if ( ram != nullptr ) {
		ram->init(device, chip_select, is_shared);
		return true;
	} else {
		return false;
	}
}
//#endif // defined(USE_EXT_RAM)

//#if defined(USE_STORAGE)
bool uCtrlClass::initStorage(SPIClass * spi_device, bool is_shared)
{
	if ( storage == nullptr ) {
		storage = &storage_module;
	}
	
	if ( storage != nullptr ) {
		storage->init(spi_device, is_shared);
		return true;
	} else {
		storage->init();
		return true;
	}	
	return false;
}
//#endif // defined(USE_STORAGE)

//#if defined(USE_SDCARD)
bool uCtrlClass::initSdCard(SPIClass * spi_device, uint8_t chip_select, bool is_shared)
{
	if ( sdcard == nullptr ) {
		sdcard = &sdcard_module;
	}
	
	if ( sdcard != nullptr ) {
		sdcard->init(spi_device, chip_select, is_shared);
		return true;
	} else {
		return false;
	}	
}
//#endif // defined(USE_SDCARD)

//#if defined(USE_DEVICE)
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
//#endif // defined(USE_DEVICE)

bool uCtrlClass::initPage(uint8_t pages_size)
{
	if ( page == nullptr ) {
		page = &page_module;
	}
	
	if ( page != nullptr ) {
		page->init(pages_size);
		return true;
	} else {
		return false;
	}
}

#if defined(USE_OLED_U8G2)
bool uCtrlClass::initOled(U8G2 * display)
#else // defined(USE_OLED_U8G2)
bool uCtrlClass::initOled(U8X8 * display)
#endif // defined(USE_OLED_U8G2)
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

//#if defined(USE_EXT_RAM)
//#if defined(USE_DEVICE)
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
//#endif // defined(USE_DEVICE)
//#endif // defined(USE_EXT_RAM)

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

bool uCtrlClass::initDout(SPIClass * spi_device, uint8_t latch_pin, bool is_shared)
{
	if ( dout == nullptr ) {
		dout = &dout_module;
	}
	
	if ( dout != nullptr ) {	
		if (spi_device != nullptr) {
			dout->setSpi(spi_device, latch_pin, is_shared);
		}
		return true;
	} else {
		return false;
	}
}

bool uCtrlClass::initDin(SPIClass * spi_device, uint8_t latch_pin, bool is_shared)
{
	if ( din == nullptr ) {
		din = &din_module;
	}
	
	if ( din != nullptr ) {
		if (spi_device != nullptr) {
			din->setSpi(spi_device, latch_pin, is_shared);
		}
		return true;
	} else {
		return false;
	}
}

bool uCtrlClass::initAin(int8_t pin1, int8_t pin2, int8_t pin3, int8_t pin4)
{
	if ( ain == nullptr ) {
		ain = &ain_module;
	}
	
	if ( ain != nullptr ) {
		// pin1 as argument means mux request to register
		if (pin1 >= 0) {
			ain->setMuxPins(pin1, pin2, pin3, pin4);
		}
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

		if ( device != nullptr ) {
			value = ain->getData(port, device->getCtrlAdcMin(port), device->getCtrlAdcMax(port));
		} else {
			value = ain->getData(port);
		}
	
		if ( value > -1 ) {

			if ( device != nullptr ) {
				// make midi signal smooth as posible
				if ( device->handleAnalogEvent(port, value, 1) == true ) {
					//continue;
				}
			} else {
				// ain callback is processed inside a timmer interrupt, so always be short inside it!
				if ( ain->rtCallback != nullptr ) {
					ain->rtCallback(port, value);
					continue;
				}
			}  

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

bool uCtrlClass::initCapTouch(int8_t pin1, int8_t pin2, int8_t pin3, int8_t pin4)
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

void uCtrlClass::init()
{
	// init of hardware configuration
	// if we have sdcard support, then retrive hardware config info from ucontrol.cfg on sdcard root filesystem
	if ( sdcard != nullptr ) {
		// load default config
	} else {
		// load factory defaults
	}

	if (ain != nullptr) {
		ain->init();
		_ain_event_queue.head = 0;
		_ain_event_queue.tail = 0;
		_ain_event_queue.size = 8;
	}

	if (touch != nullptr) {
		touch->init();
	}

	if (din != nullptr) {
		din->init();
	}
	
	if (dout != nullptr) {
		dout->init();
	}
			
	if (page != nullptr) {
		// default page and subpge
		if (page->getPageSize() > 0) {
			page->setPage(0);
			page->setSubPage(0);
		}
	}
	
	// ...
	enableTimer();
}

void uCtrlClass::run()
{
	int16_t value;
	uint8_t port, head;
	uint8_t port_ref = 0;
	bool discard_ain_data = false;

	// 250us call
	if (on250usCallback) {
		if (_doTrigger250us) {
			ATOMIC(_doTrigger250us = false)
			uCtrl.on250usCallback();
		}
	}
	
	// ~1ms call
	if (on1msCallback) {
		if (_doTrigger1ms) {
			ATOMIC(_doTrigger1ms = false)
			uCtrl.on1msCallback();
			//return;
		}
	}
	
	// ~2ms call
	if (din != nullptr) {
		if (_doTriggerDin) {
			ATOMIC(_doTriggerDin = false)

			// read while empty
			while ( din->event_queue.head != din->event_queue.tail )
			{
				port = din->event_queue.event[din->event_queue.head].port;
				value = din->event_queue.event[din->event_queue.head].value; 
				head = (din->event_queue.head+1)%(din->event_queue.size);
				ATOMIC(
					din->event_queue.head = head;
				)

				if ( device != nullptr ) {
					if ( device->handleDigitalEvent(port, value, 0) == true ) {
						continue;
					}
				}

				if (page != nullptr) {
					if (ain != nullptr) {
#if defined(USE_PAGE_COMPONENT)
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
								port == page->_nav_ctrl_port.right) 
							{
								// if it is. lock ain pot control to avoid mess with inc/dec changes
								ain->lockControl(page->_nav_ctrl_port.pot);
								// we also need to remove from our event queue any data from nav pot
								discard_ain_data = true;
							}
						}
#endif // defined(USE_PAGE_COMPONENT)
					}
					page->processEvent(port, value, uctrl::module::DIGITAL_EVENT);
				}

				if ( din->callback != nullptr )
					din->callback(port, value);
			}
			//return;
		}
		// set port_ref in case other digital modules were initialized
		port_ref = din->sizeOf();
	}
	
	// ~3ms call
	if (touch != nullptr) {
		if (_doTriggerTouch) {
			ATOMIC(_doTriggerTouch = false)

			// read while empty
			while ( touch->event_queue.head != touch->event_queue.tail )
			{
				// use port_ref in case din were initialized
				port = touch->event_queue.event[touch->event_queue.head].port+port_ref;
				value = touch->event_queue.event[touch->event_queue.head].value; 
				head = (touch->event_queue.head+1)%(touch->event_queue.size);
				ATOMIC(
					touch->event_queue.head = head;
				)

				if ( device != nullptr ) {
					if ( device->handleDigitalEvent(port, value, 0) == true )
						continue;
				}

				if (page != nullptr) {
					if (ain != nullptr) {
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
								port == page->_nav_ctrl_port.right) 
							{
								// if it is. lock ain pot control to avoid mess with inc/dec changes
								ain->lockControl(page->_nav_ctrl_port.pot);
								// we also need to remove from our event queue any data from nav pot
								discard_ain_data = true;
							}
						}
					}
					page->processEvent(port, (uint16_t)value, uctrl::module::DIGITAL_EVENT);
				}

				if ( touch->callback != nullptr )
					touch->callback(port, value);
			}
			//return;
		}
	}

	// ~10ms call
	if (uCtrl.ain != nullptr) {
		if (_doTriggerAin) {
			ATOMIC(_doTriggerAin = false)

			// read while empty
			while ( _ain_event_queue.head != _ain_event_queue.tail )
			{
				port = _ain_event_queue.event[_ain_event_queue.head].port;
				value = _ain_event_queue.event[_ain_event_queue.head].value;  
				head = (_ain_event_queue.head+1) >= _ain_event_queue.size ? 0 : (_ain_event_queue.head+1);
				ATOMIC(                      
					_ain_event_queue.head = head;
				)

		#if defined(USE_PAGE_COMPONENT)
				if (discard_ain_data) {
					if (port == page->_nav_ctrl_port.pot) {
						if (ain->isLocked(page->_nav_ctrl_port.pot) == false) {
							discard_ain_data = false;
						} else {
							continue;
						}
					}
				}
		#endif // defined(USE_PAGE_COMPONENT)

				if ( device != nullptr ) {
					// device process are done inside interrupt to keep smooth for realtime controllers events
					// EDIT MODE HANDLER
					if ( device->getCtrlMode() == 2 ) {
						device->setupCtrl(port, value);
					}

					if ( device->handleAnalogEvent(port, value, 0) == true ) {
						continue;
					}
				}

				if (page != nullptr) {
					page->processEvent(port, value, uctrl::module::ANALOG_EVENT);
				}

				if ( ain->callback != nullptr ) {
					ain->callback(port, value);
				}
			}
			//return;
		}
	}

	// ~30ms call
	if (uCtrl.dout != nullptr) {
		if (_doTriggerDout) {
			ATOMIC(_doTriggerDout = false)
			uCtrl.dout->flush(0);
			//return;
		}
	}

	// 500ms call
	if (_doTriggerUi) {
		ATOMIC(_doTriggerUi = false)
		// process ui stuffs
		uCtrl.ui();
	}
}

void uCtrlClass::ui()
{
	int16_t value;
	uint8_t port, head;
	uint8_t port_ref = 0;
	bool discard_ain_data = false;

	// timmer dependent UI visual effects
	uint32_t time = millis();
	if (dout != nullptr) {
		uCtrl.dout->setTimer(time);
	}
	if ( oled != nullptr ) {
		uCtrl.oled->setTimer(time);
#if defined(USE_OLED_U8G2)
		uCtrl.oled->clearDisplay();
#endif // defined(USE_OLED_U8G2)
	}

	// din
	if (din != nullptr) {
		
	}

	if (touch != nullptr) {
		// touch
		
	}

	if (ain != nullptr) {
		// ain:
		
	} 
              
	if ( page != nullptr ) {
#if defined(USE_PAGE_COMPONENT)
		page->clearComponentMap();
#endif // defined(USE_PAGE_COMPONENT)
		page->processView();
	}

	if ( oled != nullptr ) {
		if ( device != nullptr && ram != nullptr ) {
    		processDisplay();
		}
#if defined(USE_OLED_U8G2)
		oled->refreshDisplay();
#endif // defined(USE_OLED_U8G2)
	}

	if (dout != nullptr) {
		uCtrl.dout->flushBuffer();
	}
}

uint8_t uCtrlClass::getAnalogPorts()
{

	if (ain != nullptr) {
		return ain->sizeOf();
	}

	return 0;
}

uint8_t uCtrlClass::getDigitalPorts()
{

	if (din != nullptr) {
		return din->sizeOf();
	}

	return 0;
}

uint8_t uCtrlClass::getOutputPorts()
{

	return midi->sizeOf();
	
/*
#if defined(UMODULAR_DMX)	

#endif // defined(UMODULAR_DMX)
	
#if defined(UMODULAR_CV)	

#endif // defined(UMODULAR_CV)
*/
		
}

} // end namespace uctrl

uctrl::uCtrlClass uCtrl;

//
// 250 microseconds base interrupt for
// realtime processment tasks(midi input, scan input/output modules...)
//
uint8_t _timerCounter1ms = 0;
uint8_t _timerCounterAin = 0;
uint8_t _timerCapTouch = 0;
uint8_t _timerCounterDin = 0;
uint8_t _timerCounterDout = 0;
uint16_t _timerCounterUi = 0;

#if defined(ARDUINO_ARCH_AVR)
	#if defined(__AVR_ATmega32U4__)	
ISR(TIMER3_COMPA_vect) 
	#else
ISR(TIMER2_COMPA_vect) 
	#endif
#else
void uCtrlHandler() 
#endif
{
	// 250us call
	if (uCtrl.on250usCallback) {
		uCtrl._doTrigger250us = true;
	}
	
	if (uCtrl.on1msCallback) {
		// ~1ms call
		if(++_timerCounter1ms == 4) {
			_timerCounter1ms = 0;
			uCtrl._doTrigger1ms = true;
		}
	}
	
	if (uCtrl.din != nullptr) {
		// ~2ms call
		if (++_timerCounterDin == 8) {
			_timerCounterDin = 0;
			uCtrl._doTriggerDin = true;
			uCtrl.din->read(1);
		}
	}
	
	if (uCtrl.touch != nullptr) {
		// ~3ms call
		if (++_timerCapTouch == 12) {
			_timerCapTouch = 0;
			uCtrl.touch->read();
			uCtrl._doTriggerTouch = true;
		}
	}

	if (uCtrl.ain != nullptr) {
		// ~10ms call
		if (++_timerCounterAin == 40) {
			_timerCounterAin = 0;
			uCtrl.processAin();
			uCtrl._doTriggerAin = true;
		}
	}

	if (uCtrl.dout != nullptr) {
		// ~30ms call
		if (++_timerCounterDout == 120) {
			_timerCounterDout = 0;
			uCtrl._doTriggerDout = true;
		}
	}

	// UI handling
	// 500ms call
	if (++_timerCounterUi == 500) {
		_timerCounterUi = 0;
		uCtrl._doTriggerUi = true;
	}
}
