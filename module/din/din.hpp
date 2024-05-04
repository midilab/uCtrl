/*!
 *  @file       din.hpp
 *  Project     Arduino Library API interface for uMODULAR projects
 *  @brief      Digital input driver module (165 shiftregister)
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

#ifndef __U_CTRL_DIN_HPP__
#define __U_CTRL_DIN_HPP__

#include <Arduino.h>
#include <SPI.h>

namespace uctrl { namespace module { 

//#if !defined(USE_DIN_MAX_PORTS)
//#define USE_DIN_MAX_PORTS 18
//#endif

#define DIN_EVENT_QUEUE_SIZE	32

typedef struct 
{
	uint8_t port;
	uint8_t value;
} DIN_EVENT_QUEUE_DATA;

typedef struct
{
	volatile DIN_EVENT_QUEUE_DATA event[DIN_EVENT_QUEUE_SIZE];
	volatile uint8_t head;
	volatile uint8_t tail;
	uint8_t size; //of the buffer
} DIN_EVENT_QUEUE;

#define SPI_SPEED_DIN         4000000
//#define SPI_SPEED_DIN         2000000
#define SPI_MODE_DIN          SPI_MODE0

// helper
#define BIT_VALUE(a,n) ((a >> n)  & 0x01)		

// encoders support
typedef struct 
{
	uint8_t clk;
	uint8_t data;
} ENCODERS;	

class Din
{
    public:
    
        Din();
        ~Din();
			
		void init();
		
		void read(uint8_t interrupted = 0);
		void processQueue();
		int8_t getDataRaw(uint8_t port);
		int8_t getData(uint8_t port);				
		uint8_t sizeOf();	
		void plug(uint8_t setup);
		void plugSR(uint8_t setup);
		void encoder(uint8_t channel_a, uint8_t channel_b);

		// default callback
		void (*callback)(uint8_t port, uint16_t value) = nullptr;
		void setCallback(void (*action_callback)(uint8_t port, uint16_t value)) {
			callback = action_callback;
		}

		// for filtering Digital data changes
		// a matrix of 8 bits each for shift registers memory buffer
		uint8_t * _digital_input_state = nullptr;
		uint8_t * _digital_input_last_state = nullptr;
		uint8_t * _digital_detent_pin = nullptr;

		bool use_encoder = false;

		uint8_t _remote_digital_port = 0; 
		uint8_t _chain_size = 0;
		uint8_t _chain_size_pin = 0;
		uint8_t _chain_size_sr = 0;
		uint8_t _chain_pin_gap = 0;

    	volatile DIN_EVENT_QUEUE event_queue;	

		// used for direct microcontroller digital input pins
		uint8_t * _din_pin_map = nullptr;
		//uint8_t _din_pin_map[USE_DIN_MAX_PORTS] = {0};

		SPIClass * _spi_device = nullptr;
		void setSpi(SPIClass * spi_device = nullptr, uint8_t latch_pin = 2, bool is_shared = false);
		uint8_t _latch_pin;
		bool _is_shared = false;
};

} }

#endif
