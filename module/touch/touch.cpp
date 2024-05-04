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
#include "touch.hpp"

namespace uctrl { namespace module {

CapTouch::CapTouch()
{
	callback = nullptr;
}

CapTouch::~CapTouch()
{
	delete[] _digital_input_state;
	delete[] _digital_input_last_state;
	free(_port);
}

uint8_t CapTouch::sizeOf()
{
	return _remote_touch_port;
}

void CapTouch::setControlPins(int8_t pin1, int8_t pin2, int8_t pin3, int8_t pin4)
{
	if ( _control_pin_1 == -1 ) { // MUX_4067
		// setup pin 1
		_control_pin_1 = pin1;
		pinMode(_control_pin_1, OUTPUT);
		digitalWrite(_control_pin_1, LOW);

		// setup pin 2
		_control_pin_2 = pin2;
		pinMode(_control_pin_2, OUTPUT);
		digitalWrite(_control_pin_2, LOW);

		// setup pin 3
		_control_pin_3 = pin3;
		pinMode(_control_pin_3, OUTPUT);
		digitalWrite(_control_pin_3, LOW);

		// setup pin 4
		_control_pin_4 = pin4;
		pinMode(_control_pin_4, OUTPUT);
		digitalWrite(_control_pin_4, LOW);
	}
}

void CapTouch::plug(uint8_t analog_port)
{
	// alloc once and forever policy!
	if (_port == nullptr) {
		_port = (int8_t*) malloc( sizeof(int8_t) );
	} else {
		_port = (int8_t*) realloc( _port, sizeof(int8_t) * (_host_analog_port+1) );
	}

	if ( _control_pin_1 != -1 ) {
		_port[_host_analog_port] = analog_port;

		_host_analog_port++;
		_remote_touch_port += 16;
	}
}

void CapTouch::init() 
{
	// Allocate memory for BUFFER and his PORT_FILER structure _digital_input_state and _digital_input_last_state
	// For each 16 buttons alloc 2 byte memory area state data and other 2 byte for last state data.
	// Each bit represents the value state readed by digital inputs	
	if ( _remote_touch_port > 0 ) {
		
		uint8_t array_size = ceil(_remote_touch_port/16);
		// alloc rules: alloc once and forever! no memory free call at runtime
		_digital_input_state = new uint16_t[array_size];
		_digital_input_last_state = new uint16_t[array_size];

		for (uint8_t i=0; i < array_size; i++) {
			_digital_input_state[i] = 0;
			_digital_input_last_state[i] = 0;
		}
		_next_touch_port = 0;
		// since we use the post process of CapTouch as a wait time for mux change port signal to propagate lets select first one
		selectMuxPort(_next_touch_port);	
	}

	event_queue.head = 0;
	event_queue.tail = 0;
	event_queue.size = TOUCH_EVENT_QUEUE_SIZE;
}

void CapTouch::setThreshold(uint16_t threshold)
{
	_capacitance_threshold = threshold;
}

void CapTouch::selectMuxPort(uint8_t port)
{
	port = port%16;
			
	// select the mux port to be readed for 4067
	digitalWrite(_control_pin_1, bitRead(port, 0));
	digitalWrite(_control_pin_2, bitRead(port, 1));
	digitalWrite(_control_pin_3, bitRead(port, 2));	
	digitalWrite(_control_pin_4, bitRead(port, 3));	
}

// runs inside interruption
void CapTouch::read()
{
	uint16_t value;

	// find our indexes
	uint8_t group_analog_port = floor(_next_touch_port/16);	
	uint8_t group_touch_port = _next_touch_port%16;

	// Before refresh data, set the last state data
	// but resets only per group
	if (_next_touch_port % 16 == 0) {
		_digital_input_last_state[group_analog_port] = _digital_input_state[group_analog_port];	
		_digital_input_state[group_analog_port] = 0;
	}

	if (READ_BUFFER_SIZE == 1) {
		value = fastTouchRead(_port[group_analog_port]) > _capacitance_threshold ? 1 : 0;
	} else {
		value = 0;
		for(uint8_t k=0; k < READ_BUFFER_SIZE; k++) {
			value += fastTouchRead(_port[group_analog_port]);
		}
		value = (value / READ_BUFFER_SIZE) > _capacitance_threshold ? 1 : 0;
	}

	// select next mux port while processing this one to avoid crosstalk issues
	_next_touch_port = ++_next_touch_port % _remote_touch_port;
	selectMuxPort(_next_touch_port);

	_digital_input_state[group_analog_port] |= value << group_touch_port;

	if ( value != BIT_VALUE(_digital_input_last_state[group_analog_port], group_touch_port) ) {
		// we got a change
		uint8_t port = (group_analog_port*16)+group_touch_port;
		
		uint8_t tail = (event_queue.tail+1) >= event_queue.size ? 0 : (event_queue.tail+1);

		if ( event_queue.head == tail )
		{
			// dropping event, full queue
			return;
		}
		event_queue.event[event_queue.tail].port = port;
		event_queue.event[event_queue.tail].value = value;  
		event_queue.tail = tail; 
	}
	
}

} }
