/*!
 *  @file       ain.cpp
 *  Project     Arduino Library API interface for uMODULAR projects
 *  @brief      Analog input driver module (4051 multiplexer)
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

#include "../../../../modules.h"

#ifdef USE_AIN

#include "ain.hpp"

namespace uctrl { namespace module {

Ain::Ain()
{
	callback = nullptr;
}

Ain::~Ain()
{
	
}

uint8_t Ain::sizeOf()
{
	return _remote_analog_port;
}

void Ain::setMaxAdcValue(uint16_t max_adc_value)
{
	_user_adc_max_resolution = max_adc_value;
}

#ifdef USE_AIN_4051
void Ain::setMuxPins(uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4)
{
	// setup pin 1
	_mux_control_pin_1 = pin1;
	pinMode(_mux_control_pin_1, OUTPUT);
	digitalWrite(_mux_control_pin_1, LOW);

	// setup pin 2
	_mux_control_pin_2 = pin2;
	pinMode(_mux_control_pin_2, OUTPUT);
	digitalWrite(_mux_control_pin_2, LOW);

	// setup pin 3
	_mux_control_pin_3 = pin3;
	pinMode(_mux_control_pin_3, OUTPUT);
	digitalWrite(_mux_control_pin_3, LOW);
}
#endif

void Ain::plug(uint8_t analog_pin)
{
	_port[_host_analog_port] = analog_pin;

	// 
	pinMode(analog_pin, INPUT);

#ifdef USE_AIN_4051
	_host_analog_port++;
	_remote_analog_port += 8;
#else
	_remote_analog_port++;
#endif	
}

void Ain::init() 
{
	uint8_t host_analog_port;
	uint16_t input_data;

	// Allocate memory
	// alloc rules: alloc once and forever! no memory free call at runtime
	if ( _remote_analog_port > 0 ) {
		_analog_input_last_state = (uint16_t*) malloc( sizeof(uint16_t) * _remote_analog_port );
#ifdef ANALOG_AVG_READS
		_analog_input_state = (AVG_READS*) malloc( sizeof(AVG_READS) * _remote_analog_port );	
#else
		_analog_input_state = (uint16_t*) malloc( sizeof(uint16_t*) * _remote_analog_port );	
#endif
		_analog_input_lock_control = (int8_t*) malloc( sizeof(int8_t) * _remote_analog_port );	
#ifdef AUTOLOCK
		_analog_input_check_state = (uint16_t*) malloc( sizeof(uint16_t) * _remote_analog_port );
#endif		
	}
	
	// first mux scan 	
	for (uint8_t i=0; i < _remote_analog_port; i++) {
#ifdef USE_AIN_4051
		host_analog_port = (uint8_t)i/8;
		selectMuxPort(i);
#else
		host_analog_port = i;
#endif
#ifdef ANALOG_AVG_READS
		_analog_input_state[i].sum_value = 0;
		_analog_input_state[i].avg_count = 0;
#else
		_analog_input_state[i] = 0;
#endif
		input_data = analogRead(_port[host_analog_port]);
		_analog_input_last_state[i] = input_data;
		_analog_input_lock_control[i] = -1; // locked
#ifdef AUTOLOCK		
		_analog_input_check_state[i] = 0;
#endif		
	}
#ifdef USE_AIN_4051
	// since we use the post process of ain as a wait time for mux change port signal to propagate lets select first one
	selectMuxPort(0);			
#endif
}

void Ain::lockControl(uint8_t remote_port)
{
	_analog_input_lock_control[remote_port]	= -1;
}

bool Ain::isLocked(uint8_t remote_port)
{
	if (_analog_input_lock_control[remote_port] == -1) {
		return true;
	} else {
		return false;
	}
}

void Ain::lockAllControls()
{
	uint8_t i = _remote_analog_port;
	
	do {
		--i;
		_analog_input_lock_control[i] = -1;
	} while (i);
}

uint16_t Ain::rangeMe(uint16_t value, uint16_t min, uint16_t max, uint8_t adc_calc)
{
	if ( adc_calc == 1 ) {
		return (value / (_adc_max_resolution / ((max - min) + 1))) + min;
	} else {
		return (value / (_user_adc_max_resolution / ((max - min) + 1))) + min;
	}
}

#ifdef USE_AIN_4051
void Ain::selectMuxPort(uint8_t port)
{
	port = port%8;
			
	// select the mux port to be readed for 4051
	digitalWrite(_mux_control_pin_1, bitRead(port, 0));
	digitalWrite(_mux_control_pin_2, bitRead(port, 1));
	digitalWrite(_mux_control_pin_3, bitRead(port, 2));	
}
#endif

void Ain::invertRead(bool state)
{
	_invert_read = state;
}

#ifdef ANALOG_AVG_READS
int16_t Ain::readPortAvg(uint8_t remote_port)
{
	if (_analog_input_state[remote_port].avg_count >= ANALOG_AVG_READS) {
		int16_t avg_value = _analog_input_state[remote_port].sum_value / _analog_input_state[remote_port].avg_count;
		_analog_input_state[remote_port].sum_value = 0;
		_analog_input_state[remote_port].avg_count = 0;
		return avg_value;
	} else {

#ifdef USE_AIN_4051
		_analog_input_state[remote_port].sum_value += analogRead(_port[(uint8_t)(remote_port/8)]);
#else 
		_analog_input_state[remote_port].sum_value += analogRead(_port[remote_port]);
#endif
		_analog_input_state[remote_port].avg_count++;
		return -1;
	}
}
#endif

int16_t Ain::getData(uint8_t remote_port, uint16_t min, uint16_t max)
{
	uint8_t host_analog_port;
	int16_t value, last_value;
	int16_t input_data;
	
#ifdef USE_AIN_4051

	// find our indexes
	host_analog_port = (uint8_t)(remote_port/8);	

	// get selected value on last getdata operation(only serial reading incremental+ works)
	#ifdef ANALOG_AVG_READS
	input_data = readPortAvg(remote_port);
	#else
	input_data = analogRead(_port[host_analog_port]);
	#endif

	// select next mux port while processing this one
	selectMuxPort(remote_port+1);

#else

	#ifdef ANALOG_AVG_READS
	input_data = readPortAvg(remote_port);
	#else
	input_data = analogRead(_port[remote_port]);
	#endif
	host_analog_port = remote_port;

#endif
	
	if (input_data < 0) {
		return -1;
	}

	// do we need to invert reads?(pot with gnd and vcc swapped)
	if (_invert_read) {
		input_data = _user_adc_max_resolution - input_data;
	}

	if ( min == 0 && max == 0 ) {
		max = _user_adc_max_resolution - 1;
		value = input_data;
		last_value = _analog_input_last_state[remote_port];
	} else {
		value = rangeMe(input_data, min, max, 1);
		last_value = rangeMe(_analog_input_last_state[remote_port], min, max, 1);		
	}

	// Process only the registered host_ports
	if (_port[host_analog_port] != -1) {
		// check locker
		if ( _analog_input_lock_control[remote_port] == -1 ) {
			if ( abs(value - last_value) > ceil((max+1)/_adc_unlock_divider) ) {	
				// unlock
				_analog_input_lock_control[remote_port] = 1;	
#ifdef AUTOLOCK			
				_analog_input_check_state[remote_port] = _analog_input_last_state[remote_port];
#endif				
				_analog_input_last_state[remote_port] = input_data;
				return value;
			}
			return -1;
		}
		
#ifdef AUTOLOCK	
		if ( input_data == _analog_input_check_state[remote_port] ) {
			_analog_input_lock_control[remote_port]--;
		}
#endif		
		
		if ( abs(value - last_value) >= 1 ) {
#ifdef AUTOLOCK			
			_analog_input_check_state[remote_port] = _analog_input_last_state[remote_port];
#endif		 
			_analog_input_last_state[remote_port] = input_data;      	
			return value;
		}
	}
	
	return -1;
	
}

} }

uctrl::module::Ain ain_module;
#endif
