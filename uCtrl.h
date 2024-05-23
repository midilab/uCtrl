/*!
 *  @file       uCtrl.h
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

#ifndef __U_CTRL_H__
#define __U_CTRL_H__

#include <Arduino.h> 
#include <SPI.h>

// modules classes includes
#include "module/oled/oled.hpp"

#include "module/midi/midi.hpp"

// dout 595 or single output for arduino only
#include "module/dout/dout.hpp"

#include "module/din/din.hpp"

#include "module/ain/ain.hpp"

#include "module/touch/touch.hpp"

#include "module/ram/ram.hpp"

#include "module/storage/storage.hpp"

#include "module/sdcard/sdcard.hpp"

#include "module/page/page.hpp"

#include "module/device/device.hpp"

// tools
#define BLINK_TIME 250	

namespace uctrl {

typedef struct 
{
	uint16_t port;
	uint16_t value;
} EVENT_QUEUE_DATA;

typedef struct
{
	volatile EVENT_QUEUE_DATA event[8];
	volatile uint8_t head;
	volatile uint8_t tail;
	uint8_t size; //of the buffer
} EVENT_QUEUE;

class uCtrlClass 
{
	
  public:
  
	uCtrlClass();
	~uCtrlClass();
	
	//
	// Grab config data and create all memory data layout
	//
	void init();
	
	//
	// modules access
	//
	// external ram module
	bool initRam(SPIClass * device, uint8_t chip_select = 2, bool is_shared = false);
	uctrl::module::Ram * ram = nullptr;		

	bool initStorage(SPIClass * spi_device = nullptr, bool is_shared = false);
	uctrl::module::Storage * storage = nullptr;	

	// oled module
#if defined(USE_OLED_U8G2)
	bool initOled(U8G2 * display);
#else // defined(USE_OLED_U8G2)
	bool initOled(U8X8 * display);
#endif // defined(USE_OLED_U8G2)

//#if defined(USE_EXT_RAM)
//#if defined(USE_DEVICE)
	void processDisplay();
//#endif // defined(USE_DEVICE)
//#endif // defined(USE_EXT_RAM)
	uctrl::module::Oled * oled = nullptr;

	// midi module
	bool initMidi();
	uctrl::module::Midi * midi = nullptr;
	
	// dout module
	bool initDout(SPIClass * spi_device = nullptr, uint8_t latch_pin = 2, bool is_shared = false);
	uctrl::module::Dout * dout = nullptr;
	
	// din module
	bool initDin(SPIClass * spi_device = nullptr, uint8_t latch_pin = 2, bool is_shared = false);
	uctrl::module::Din * din = nullptr;
	
	// ain module
	bool initAin(int8_t pin1 = -1, int8_t pin2 = -1, int8_t pin3 = -1, int8_t pin4 = -1);
	void processAin();
	uctrl::module::Ain * ain = nullptr;
    volatile EVENT_QUEUE _ain_event_queue;
	
	// capacitive touch module
	bool initCapTouch(int8_t pin1 = -1, int8_t pin2 = -1, int8_t pin3 = -1, int8_t pin4 = -1);
	uctrl::module::CapTouch * touch = nullptr;
	
	// sdcard module
	bool initSdCard(SPIClass * spi_device = nullptr, uint8_t chip_select = 2, bool is_shared = false);	
	uctrl::module::SdCard * sdcard;
	
	// page module
	bool initPage(uint8_t pages_size);
	void processPage();
	uctrl::module::Page * page = nullptr;
	
	// device module
	bool initDevice(uint8_t device_number, uint16_t event_buffer_size, uint8_t sysex_buffer_size = 0, uint16_t device_label_buffer_size = 0);
	uctrl::module::Device * device = nullptr;
        
	//
	// Registred Ports query
	//
	uint8_t getOutputPorts();
	uint8_t getAnalogPorts();
	uint8_t getDigitalPorts();
	
	// runtime handler
	void run();

	void setLoopCallback(void (*callback)()) {
		loopCallback = callback;
	}

	void setOn250usCallback(void (*callback)()) {
		on250usCallback = callback;
	}

	void setOn1msCallback(void (*callback)()) {
		on1msCallback = callback;
	}

	void (*loopCallback)();
	void (*on250usCallback)();
	void (*on1msCallback)();
};

}

extern uctrl::uCtrlClass uCtrl;

#endif // __U_CTRL_H__

