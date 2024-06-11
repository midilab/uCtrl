/*!
 *  @file       ain.hpp
 *  Project     Arduino Library API interface for uMODULAR projects
 *  @brief      Analog input driver module (4051/4067 multiplexer)
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

#ifndef __U_CTRL_AIN_HPP__
#define __U_CTRL_AIN_HPP__

#include <Arduino.h>

namespace uctrl { namespace module { 

#define AUTOLOCK
#define ANALOG_AVG_READS

//#if !defined(USE_AIN_MAX_PORTS)
//#define USE_AIN_MAX_PORTS 8
//#endif

#define AIN_4051_MUX_SIZE	8
#define AIN_4067_MUX_SIZE	16

#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
#define ADC_RESOLUTION 4096
#else
#define ADC_RESOLUTION 1024
#endif

typedef enum {
	MUX_DRIVER_4051,
	MUX_DRIVER_4067
} MUX_DRIVERS;

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
		int16_t readPort(uint8_t remote_port, uint8_t host_analog_port);
#ifdef ANALOG_AVG_READS
		void setAvgReads(uint8_t average);
#endif
		inline uint16_t rangeMe(uint16_t value, uint16_t min, uint16_t max);
		void lockControl(uint8_t remote_port);
		void lockAllControls();
		bool isLocked(uint8_t remote_port);
		void plug(uint8_t setup);
		void plugMux(uint8_t setup);
		void invertRead(bool state);		
		uint8_t sizeOf();

		// callback called by uCtrl using realtime buffer outside timer interrupt
		void (*callback)(uint8_t port, uint16_t value);
		void setCallback(void (*action_callback)(uint8_t port, uint16_t value)) {
			callback = action_callback;
		}
		// callback called by uCtrl at procesing time inside timer interrupt(use this with care and responsability! ATOMIC() shared resources and volatile then)
		void (*rtCallback)(uint8_t port, uint16_t value);
		void setRTCallback(void (*action_callback)(uint8_t port, uint16_t value)) {
			rtCallback = action_callback;
		}
		
		void setMaxAdcValue(uint16_t max_adc_value);
	
		void setMuxPins(int8_t pin1 = -1, int8_t pin2 = -1, int8_t pin3 = -1, int8_t pin4 = -1);
		void selectMuxPort(uint8_t port);
		int8_t _mux_control_pin_1 = -1;
		int8_t _mux_control_pin_2 = -1;
		int8_t _mux_control_pin_3 = -1;
		int8_t _mux_control_pin_4 = -1;
		int8_t _use_mux_driver = -1; // 0=4051, 1=4067
		uint8_t _mux_size = 0;

		//int8_t _port[USE_AIN_MAX_PORTS] = {-1};
		int8_t * _port = nullptr;

		uint8_t _host_analog_port = 0; // uint8_t hard_port[16];
		uint8_t _remote_analog_port = 0; // uint8_t soft_port[16];
		uint8_t _direct_pin_size = 0;

		// For arduino ADC max resolution is 10bits(ADC_RESOLUTION)
		uint16_t _adc_max_resolution = ADC_RESOLUTION;
		//uint8_t _adc_unlock_divider = 8;
		//uint8_t _adc_unlock_divider = 16;
		//uint8_t _adc_unlock_divider = 32;
		uint8_t _adc_unlock_divider = 64;
		uint16_t _user_adc_max_resolution = ADC_RESOLUTION;
			
		// for filtering ADC data changes
#ifdef ANALOG_AVG_READS
		AVG_READS * _analog_input_state;
		uint8_t _analog_avg_reads = 1;
#else
		uint16_t * _analog_input_state;
#endif
		uint16_t * _analog_input_last_state;	
		int8_t * _analog_input_lock_control;	
#ifdef AUTOLOCK	
		uint16_t * _analog_input_check_state;	
#endif	

		bool _invert_read = false;
			
};		
			
} }
		
#endif
