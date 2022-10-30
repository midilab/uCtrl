/*!
 *  @file       ain.hpp
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

#ifndef __U_CTRL_AIN_HPP__
#define __U_CTRL_AIN_HPP__

#include <Arduino.h>

namespace uctrl { namespace module { 

#define AUTOLOCK
#define ANALOG_AVG_READS 8

#ifdef ANALOG_AVG_READS
typedef struct
{
	uint16_t sum_value;
	uint8_t avg_count;
} AVG_READS; 
#endif

class Ain
{
    public:
    
        Ain();
        ~Ain();
			
		void init();
		int16_t getData(uint8_t remote_port, uint16_t min = 0, uint16_t max = 0);
#ifdef ANALOG_AVG_READS
		int16_t readPortAvg(uint8_t remote_port);
#endif
		uint16_t rangeMe(uint16_t value, uint16_t min, uint16_t max, uint8_t adc_calc = 0);
		void lockControl(uint8_t remote_port);
		void lockAllControls();
		bool isLocked(uint8_t remote_port);
		void plug(uint8_t analog_pin);		
		void invertRead(bool state);		
		uint8_t sizeOf();

		// default callback
		void (*callback)(uint8_t port, uint16_t value, uint8_t interrupted);
		void setCallback(void (*action_callback)(uint8_t port, uint16_t value, uint8_t interrupted)) {
			callback = action_callback;
		}
		
		void setMaxAdcValue(uint16_t max_adc_value);
	
#ifdef USE_AIN_4051
		void setMuxPins(uint8_t pin1 = 0, uint8_t pin2 = 0, uint8_t pin3 = 0, uint8_t pin4 = 0);
		void selectMuxPort(uint8_t port);
		int8_t _mux_control_pin_1 = -1;
		int8_t _mux_control_pin_2 = -1;
		int8_t _mux_control_pin_3 = -1;
#endif

		// max of 128 analog ports with 16x 4051
		int8_t _port[16] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};

		uint8_t _host_analog_port = 0; // uint8_t hard_port[16];
		uint8_t _remote_analog_port = 0; // uint8_t soft_port[16];

		// For arduino ADC max resolution is 10bits(1024)
		uint16_t _adc_max_resolution = 1024;
		//uint8_t _adc_unlock_divider = 8;
		//uint8_t _adc_unlock_divider = 16;
		//uint8_t _adc_unlock_divider = 32;
		uint8_t _adc_unlock_divider = 64;
		uint16_t _user_adc_max_resolution = 1024;
			
		// for filtering ADC data changes
#ifdef ANALOG_AVG_READS
		AVG_READS _analog_input_state[USE_AIN_MAX_PORTS];
#else
		uint16_t _analog_input_state[USE_AIN_MAX_PORTS];
#endif
		uint16_t _analog_input_last_state[USE_AIN_MAX_PORTS];	
		int8_t _analog_input_lock_control[USE_AIN_MAX_PORTS];	
#ifdef AUTOLOCK	
		uint16_t _analog_input_check_state[USE_AIN_MAX_PORTS];	
#endif		

		bool _invert_read = false;
			
};		
			
} }

extern uctrl::module::Ain ain_module;
		
#endif
