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

#include "../../modules.h"

// modules classes includes
#if defined(USE_OLED)
#include "module/oled/oled.hpp"
#endif // defined(USE_OLED)

#if defined(USE_MIDI)
#include "module/midi/midi.hpp"
#endif // defined(USE_MIDI)

// dout 595 or single output for arduino only
#if defined(USE_DOUT)
#include "module/dout/dout.hpp"
#endif // defined(USE_DOUT)

#if defined(USE_DIN)
#include "module/din/din.hpp"
#endif // defined(USE_DIN)

#if defined(USE_AIN)
#include "module/ain/ain.hpp"
#endif // defined(USE_AIN)

#if defined(USE_CAP_TOUCH)
#include "module/touch/touch.hpp"
#endif // defined(USE_CAP_TOUCH)

#if defined(USE_EXT_RAM)
#include "module/ram/ram.hpp"
#endif // defined(USE_EXT_RAM)

#if defined(USE_STORAGE)
#include "module/storage/storage.hpp"
#endif // defined(USE_STORAGE)

#if defined(USE_SDCARD)
#include "module/sdcard/sdcard.hpp"
#endif // defined(USE_SDCARD)

#if defined(USE_PAGE)
#include "module/page/page.hpp"
#endif // defined(USE_PAGE)

#if defined(USE_DEVICE)
#include "module/device/device.hpp"
#endif // defined(USE_DEVICE)

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
	EVENT_QUEUE_DATA event[8];
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
#if defined(USE_EXT_RAM)
	// external ram module
	bool initRam(SPIClass * device, uint8_t chip_select);
	uctrl::module::Ram * ram = nullptr;		
#endif // defined(USE_EXT_RAM)

#if defined(USE_STORAGE)
	bool initStorage(SPIClass * spi_device = nullptr, uint8_t chip_select = 1);
	uctrl::module::Storage * storage = nullptr;		
#endif // defined(USE_STORAGE)

#if defined(USE_OLED)	
	// oled module
#if defined(USE_OLED_U8G2)
	bool initOled(U8G2 * display);
#else // defined(USE_OLED_U8G2)
	bool initOled(U8X8 * display);
#endif // defined(USE_OLED_U8G2)

#if defined(USE_EXT_RAM)
#if defined(USE_DEVICE)
	void processDisplay();
#endif // defined(USE_DEVICE)
#endif // defined(USE_EXT_RAM)
	uctrl::module::Oled * oled = nullptr;
#endif	// defined(USE_OLED)

#if defined(USE_MIDI)
	// midi module
	bool initMidi();
	void processMidi();
	uctrl::module::Midi * midi = nullptr;
#endif // defined(USE_MIDI)
	
#if defined(USE_DOUT)	
	// dout module
	bool initDout(SPIClass * spi_device = nullptr);
	uctrl::module::Dout * dout = nullptr;
#endif // defined(USE_DOUT)
	
#if defined(USE_DIN)
	// din module
	bool initDin(SPIClass * spi_device = nullptr);
	uctrl::module::Din * din = nullptr;
#endif // defined(USE_DIN)
	
#if defined(USE_AIN)
	// ain module
	bool initAin(uint8_t pin1 = 0, uint8_t pin2 = 0, uint8_t pin3 = 0, uint8_t pin4 = 0);
	void processAin();
	uctrl::module::Ain * ain = nullptr;
    volatile EVENT_QUEUE _ain_event_queue;	
#endif // defined(USE_AIN)
	
#if defined(USE_CAP_TOUCH)
	// capacitive touch module
	bool initCapTouch(uint8_t pin1 = 0, uint8_t pin2 = 0, uint8_t pin3 = 0, uint8_t pin4 = 0);
	uctrl::module::CapTouch * touch = nullptr;
#endif // defined(USE_CAP_TOUCH)
	
#if defined(USE_SDCARD)
	// sdcard module
	bool initSdCard(SPIClass * spi_device = nullptr, uint8_t chip_select = 0);	
	uctrl::module::SdCard * sdcard;
#endif // defined(USE_SDCARD)
	
#if defined(USE_PAGE)
	// page module
	bool initPage();
	void processPage();
	uctrl::module::Page * page = nullptr;
#endif // defined(USE_PAGE)
	
#if defined(USE_DEVICE)	
	// device module
	bool initDevice(uint8_t device_number, uint16_t event_buffer_size, uint8_t sysex_buffer_size = 0, uint16_t device_label_buffer_size = 0);
	uctrl::module::Device * device = nullptr;
#endif // defined(USE_DEVICE)
        
	//
	// Registred Ports query
	//
	uint8_t getOutputPorts();
	uint8_t getAnalogPorts();
	uint8_t getDigitalPorts();
	
	void run();

	void setOn250usCallback(void (*callback)()) {
		on250usCallback = callback;
	}

	void setOn1msCallback(void (*callback)()) {
		on1msCallback = callback;
	}

	void (*on250usCallback)();
	void (*on1msCallback)();
};

}

extern uctrl::uCtrlClass uCtrl;

#endif // __U_CTRL_H__

