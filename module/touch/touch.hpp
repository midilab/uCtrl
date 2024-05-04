/*!
 *  @file       touch.cpp
 *  Project     Arduino Library API interface for uMODULAR projects
 *  @brief      Capacitive Touch driver/helper module
 *  @version    1.0.0
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

#ifndef __U_CTRL_CAP_TOUCH_HPP__
#define __U_CTRL_CAP_TOUCH_HPP__

#include <Arduino.h>
#include "FastTouch.h"

namespace uctrl { namespace module {

#define TOUCH_EVENT_QUEUE_SIZE	4

#define READ_BUFFER_SIZE	1

typedef struct 
{
	uint8_t port;
	uint8_t value;
} TOUCH_EVENT_QUEUE_DATA;

typedef struct
{
	TOUCH_EVENT_QUEUE_DATA event[TOUCH_EVENT_QUEUE_SIZE];
	volatile uint8_t head;
	volatile uint8_t tail;
	uint8_t size; //of the buffer
} TOUCH_EVENT_QUEUE;

// helper
#define BIT_VALUE(a,n) ((a >> n)  & 0x01)	

#define MAX_TOUCH_MUX	4

class CapTouch
{
    public:
    
        CapTouch();
        ~CapTouch();
			
		void init();
		void selectMuxPort(uint8_t port);
		void read();
		void setControlPins(int8_t pin1 = -1, int8_t pin2 = -1, int8_t pin3 = -1, int8_t pin4 = -1);
		void plug(uint8_t analog_port);	
		void setThreshold(uint16_t threshold);		
		uint8_t sizeOf();

		// default callback
		void (*callback)(uint8_t port, uint16_t value) = nullptr;
		void setCallback(void (*action_callback)(uint8_t port, uint16_t value)) {
			callback = action_callback;
		}
		
		//int8_t _port[MAX_TOUCH_MUX] = {-1};
		int8_t * _port = nullptr;
		int8_t _control_pin_1 = -1;
		int8_t _control_pin_2 = -1;
		int8_t _control_pin_3 = -1;
		int8_t _control_pin_4 = -1;

		// for filtering Digital data changes
		// a matrix of 16 bits each for shift registers memory buffer
		uint16_t * _digital_input_state = nullptr;
		uint16_t * _digital_input_last_state = nullptr;

		uint8_t _host_analog_port = 0;
		uint8_t _remote_touch_port = 0; 
		uint8_t _current_touch_port = 0; 
		uint8_t _next_touch_port = 0; 

		uint16_t _capacitance_threshold = 40;

    	volatile TOUCH_EVENT_QUEUE event_queue;	
			
};		
			
} }
		
#endif
