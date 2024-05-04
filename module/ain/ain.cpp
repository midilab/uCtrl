/*!
 *  @file       ain.cpp
 *  Project     Arduino Library API interface for uMODULAR projects
 *  @brief      Analog input driver module, supports uc analog ports and 4051/4067 multiplexers
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
#include "ain.hpp"

namespace uctrl { namespace module {

Ain::Ain()
{
	callback = nullptr;
	rtCallback = nullptr;
}

// we never reach here because its a lifetime object
Ain::~Ain()
{
	delete[] _analog_input_last_state;
	delete[] _analog_input_state;
	delete[] _analog_input_lock_control;
#ifdef AUTOLOCK	
	delete[] _analog_input_check_state;	
#endif		 
	free(_port);
}

uint8_t Ain::sizeOf()
{
	return _remote_analog_port;
}

void Ain::setMaxAdcValue(uint16_t max_adc_value)
{
	_user_adc_max_resolution = max_adc_value;
}

void Ain::setMuxPins(int8_t pin1, int8_t pin2, int8_t pin3, int8_t pin4)
{
	// start saying this is a 4051 mux pin register
	_use_mux_driver = MUX_DRIVER_4051;

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

	// setup pin 4
	// its a 4067 driver if user request to setup pin4
	if (pin4 != -1) {
		// setup pin 4
		_mux_control_pin_4 = pin4;
		pinMode(_mux_control_pin_4, OUTPUT);
		digitalWrite(_mux_control_pin_4, LOW);
		// in case its a 4067, change it
		_use_mux_driver = MUX_DRIVER_4067;
	}

	_mux_size = _use_mux_driver == MUX_DRIVER_4051 ? AIN_4051_MUX_SIZE : AIN_4067_MUX_SIZE;
}

// call first all plug() for pin register, then plugMux()
void Ain::plug(uint8_t setup)
{
	//if (_host_analog_port >= USE_AIN_MAX_PORTS) 
	//	return;

	// alloc once and forever policy!
	if (_port == nullptr) {
		_port = (int8_t*) malloc( sizeof(int8_t) );
	} else {
		_port = (int8_t*) realloc( _port, sizeof(int8_t) * (_host_analog_port+1) );
	}

	_port[_host_analog_port] = setup;
	pinMode(setup, INPUT);

	++_host_analog_port;
	++_remote_analog_port;
	// we should keep the size of registered direct pins to use
	++_direct_pin_size;
}

void Ain::plugMux(uint8_t setup)
{
	//if (_host_analog_port >= USE_AIN_MAX_PORTS) 
	//	return;

	// alloc once and forever policy!
	if (_port == nullptr) {
		_port = (int8_t*) malloc( sizeof(int8_t) );
	} else {
		_port = (int8_t*) realloc( _port, sizeof(int8_t) * (_host_analog_port+1) );
	}

	_port[_host_analog_port] = setup;
	pinMode(setup, INPUT);

	++_host_analog_port;
	_remote_analog_port += _mux_size;
}

void Ain::init() 
{
	uint8_t host_analog_port;
	uint16_t input_data;

	// Allocate memory
	// alloc rules: alloc once and forever! no memory free call at runtime
	if ( _remote_analog_port > 0 ) {
		//_analog_input_last_state = (uint16_t*) malloc( sizeof(uint16_t) * _remote_analog_port );
		_analog_input_last_state = new uint16_t[_remote_analog_port];
#ifdef ANALOG_AVG_READS
		//_analog_input_state = (AVG_READS*) malloc( sizeof(AVG_READS) * _remote_analog_port ); 
		_analog_input_state = new AVG_READS[_remote_analog_port];
#else
		//_analog_input_state = (uint16_t*) malloc( sizeof(uint16_t*) * _remote_analog_port ); 
		_analog_input_state = new uint16_t[_remote_analog_port];	
#endif
		//_analog_input_lock_control = (int8_t*) malloc( sizeof(int8_t) * _remote_analog_port );  
		_analog_input_lock_control = new int8_t[_remote_analog_port];
#ifdef AUTOLOCK
		//_analog_input_check_state = (uint16_t*) malloc( sizeof(uint16_t) * _remote_analog_port );
		_analog_input_check_state = new uint16_t[_remote_analog_port];
#endif   
	}

	// initing memory and first mux scan 	
	for (uint8_t remote_port=0; remote_port < _remote_analog_port; remote_port++) {

		// is this a direct pin registered to read?
		if (remote_port < _direct_pin_size) {
			host_analog_port = remote_port;
		// otherwise its a mux read request
		} else {
			uint8_t mux_host_port = remote_port - _direct_pin_size;
			// find our indexes
			host_analog_port = (uint8_t)(mux_host_port/_mux_size) + _direct_pin_size;
			selectMuxPort(mux_host_port);
		}

#ifdef ANALOG_AVG_READS
		_analog_input_state[remote_port].sum_value = 0;
		_analog_input_state[remote_port].avg_count = 0;
#else
		_analog_input_state[remote_port] = 0;
#endif

		input_data = analogRead(_port[host_analog_port]);
		_analog_input_last_state[remote_port] = input_data;
		_analog_input_lock_control[remote_port] = -1; // locked

#ifdef AUTOLOCK		
		_analog_input_check_state[remote_port] = 0;
#endif		

	}
	
	if (_use_mux_driver == MUX_DRIVER_4051 || _use_mux_driver == MUX_DRIVER_4067) {
		// since we use the post process of ain as a wait time for mux change port signal to propagate lets select first one
		selectMuxPort(0);		
	}	
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

inline uint16_t Ain::rangeMe(uint16_t value, uint16_t min, uint16_t max)
{
	return (value / (_adc_max_resolution / ((max - min) + 1))) + min;
}

void Ain::selectMuxPort(uint8_t port)
{
	port = port%_mux_size;
	
	// select the mux port to be readed for 4051 or 4067
	digitalWrite(_mux_control_pin_1, bitRead(port, 0));
	digitalWrite(_mux_control_pin_2, bitRead(port, 1));
	digitalWrite(_mux_control_pin_3, bitRead(port, 2));	
	if (_use_mux_driver == MUX_DRIVER_4067) {
		digitalWrite(_mux_control_pin_4, bitRead(port, 3));	
	}
}

void Ain::invertRead(bool state)
{
	_invert_read = state;
}

int16_t Ain::readPort(uint8_t remote_port, uint8_t host_analog_port)
{
#ifdef ANALOG_AVG_READS
	if (_analog_input_state[remote_port].avg_count >= _analog_avg_reads) {
		int16_t avg_value = _analog_input_state[remote_port].sum_value / _analog_input_state[remote_port].avg_count;
		_analog_input_state[remote_port].sum_value = 0;
		_analog_input_state[remote_port].avg_count = 0;
		return avg_value;
	} else {
		_analog_input_state[remote_port].sum_value += analogRead(_port[host_analog_port]);
		_analog_input_state[remote_port].avg_count++;
		return -1;
	}
#else // ANALOG_AVG_READS
	return analogRead(_port[host_analog_port]);
#endif
}

#ifdef ANALOG_AVG_READS
void Ain::setAvgReads(uint8_t average)
{
	_analog_avg_reads = average;
}
#endif

int16_t Ain::getData(uint8_t remote_port, uint16_t min, uint16_t max)
{
	uint8_t host_analog_port = 0;
	int16_t value = 0, last_value = 0, input_data = 0;
	
	// is this a direct pin registered to read?
	if (remote_port < _direct_pin_size) {
		host_analog_port = remote_port;
		// get data
		input_data = readPort(remote_port, host_analog_port);
	// otherwise its a mux read request
	} else {
//#if defined(USE_AIN_4051_DRIVER) || defined(USE_AIN_4067_DRIVER)
		//if (_use_mux_driver == MUX_DRIVER_4051 || _use_mux_driver == MUX_DRIVER_4067) {
			uint8_t mux_host_port = remote_port - _direct_pin_size;
			// find our indexes
			host_analog_port = (uint8_t)(mux_host_port/_mux_size) + _direct_pin_size;
			// get data
			input_data = readPort(remote_port, host_analog_port);
			// select next mux port while processing this one
			selectMuxPort(mux_host_port+1);
		//}
//#endif
	}
	
	if (input_data < 0) {
		return -1;
	}

	// do we need to invert reads?(pot with gnd and vcc swapped)
	if (_invert_read) {
		input_data = _adc_max_resolution - input_data;
	}

	// value remap?
	if (max == 0)
		max = _user_adc_max_resolution != _adc_max_resolution ? _user_adc_max_resolution-1 : _adc_max_resolution;

	if ( min == 0 && max == _adc_max_resolution ) {
		value = input_data;
		last_value = _analog_input_last_state[remote_port];
	} else {
		value = rangeMe(input_data, min, max);
		last_value = rangeMe(_analog_input_last_state[remote_port], min, max);		
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