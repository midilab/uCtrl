/*!
 *  @file       dout.hpp
 *  Project     Arduino Library API interface for uMODULAR projects
 *  @brief      Digital output driver module (595 shiftregister)
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

#ifndef __U_CTRL_DOUT_HPP__
#define __U_CTRL_DOUT_HPP__

#include <Arduino.h>
#include <SPI.h>

namespace uctrl { namespace module { 

/* #if !defined(USE_DOUT_MAX_PORTS)
#define USE_DOUT_MAX_PORTS 12
#endif */

#define SPI_SPEED_DOUT         4000000
//#define SPI_SPEED_DOUT         2000000
#define SPI_MODE_DOUT          SPI_MODE0	

// helper
#define BIT_GET_VALUE(a,n) ((a >> n)  & 0x01)

#define BLINK_TIME 250

class Dout
{
    public:
    
        Dout();
        ~Dout();
                    		
        void init();
        void flush(uint8_t interrupted);
        void flushBuffer();
        void write(uint8_t remote_port, uint8_t value, uint8_t interrupted = 0);
        void writeAll(uint8_t value, uint8_t interrupted = 0);		
        uint8_t sizeOf();	
        void plug(uint8_t setup);
        void plugSR(uint8_t setup);
        void setTimer(uint32_t time);
        bool blink();

        uint8_t * _digital_output_state = nullptr;
        uint8_t * _digital_output_buffer = nullptr;

        // used for direct microcontroller digital output pins
	//uint8_t _dout_pin_map[USE_DOUT_MAX_PORTS] = {0};
        uint8_t * _dout_pin_map = nullptr;

        uint8_t _remote_digital_output_port = 0; 

        uint8_t _chain_size = 0;
        uint8_t _chain_size_pin = 0;
        volatile bool _flush_dout = false;
        bool _change_flag = true;
        
	bool _blink = false;
	uint32_t _blink_timer = 0;

	SPIClass * _spi_device = nullptr;
	void setSpi(SPIClass * spi_device = nullptr, uint8_t latch_pin = 2, bool is_shared = false);
        int8_t _latch_pin = -1;
        bool _is_shared = false;

};

} }

#endif
