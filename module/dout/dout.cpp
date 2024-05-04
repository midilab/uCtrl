/*!
 *  @file       dout.cpp
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

#include "dout.hpp"

namespace uctrl { namespace module {

Dout::Dout()
{
	
}

Dout::~Dout()
{
	delete[] _digital_output_state;
    delete[] _digital_output_buffer;
	free(_dout_pin_map);
}

uint8_t Dout::sizeOf()
{
	return _remote_digital_output_port;
}	

void Dout::setSpi(SPIClass * spi_device, uint8_t latch_pin, bool is_shared)
{
	_spi_device = spi_device;
	_latch_pin = latch_pin;
	_is_shared = is_shared;
}

// call first all plug() for pin register, then plugSR if needed
void Dout::plug(uint8_t setup)
{
	//if (_chain_size_pin >= USE_DOUT_MAX_PORTS)
	//	return;

	// alloc once and forever policy!
	if (_dout_pin_map == nullptr) {
		_dout_pin_map = (uint8_t*) malloc( sizeof(uint8_t) );
	} else {
		_dout_pin_map = (uint8_t*) realloc( _dout_pin_map, sizeof(uint8_t) * (_chain_size_pin+1) );
	}

	_dout_pin_map[_chain_size_pin] = setup;
	++_chain_size_pin;
}

void Dout::plugSR(uint8_t setup)
{
	_chain_size = setup;
}

void Dout::init()
{
#if defined(USE_DOUT_BITBANG_DRIVER)
	pinMode(DOUT_LATCH_PIN, OUTPUT);
	pinMode(DOUT_DATA_PIN, OUTPUT); 
	pinMode(DOUT_CLOCK_PIN, OUTPUT);
	// Set SPI slave device 74H595 deactivated by default
	digitalWrite(DOUT_LATCH_PIN, HIGH);
	_change_flag = true;
#endif

	// total remote out port
	_remote_digital_output_port = (_chain_size * 8) + _chain_size_pin;

	// any plug() for direct pin register on DOUT?
	if (_chain_size_pin > 0) {
		// walk port reference structure and setup PINs as PULLUP/INPUT
		for (uint8_t i=0; i < _chain_size_pin; i++ ) {
			pinMode(_dout_pin_map[i], OUTPUT);
			digitalWrite(_dout_pin_map[i], LOW);
		}
		//_change_flag = false;
	}

//#if defined(USE_DOUT_SPI_DRIVER) || defined(USE_DOUT_BITBANG_DRIVER)
	if (_spi_device != nullptr) {
		// Chip select pin setup
		pinMode(_latch_pin, OUTPUT);
		// 74H595 latched out by default
		digitalWrite(_latch_pin, HIGH);	
		// Initing a common shared SPI device for all uMODULAR modules using SPI bus
		_spi_device->begin();
		_change_flag = true;
		// For each 8 buttons alloc 1 byte memory area state data and other 1 byte for last state data.
		// Each bit represents the value state readed by digital inputs
		if ( _chain_size > 0 ) {
			// alloc once and forever policy!
			//_digital_output_state = (uint8_t*) malloc( sizeof(uint8_t) * _chain_size );
			//_digital_output_buffer = (uint8_t*) malloc( sizeof(uint8_t) * _chain_size );
			_digital_output_state = new uint8_t[_chain_size];
			_digital_output_buffer = new uint8_t[_chain_size];
			for (uint8_t i=0; i < _chain_size; i++) {
				_digital_output_state[i] = 0;
				_digital_output_buffer[i] = 0;
			}			
		}
	}
//#endif
}

// should be called only inside non interrupted stack
// for interrupted the flush will be called on write()
void Dout::flushBuffer()
{
	//int8_t i;
	if (_change_flag == false) {
		return;
	}
//#if defined(USE_DOUT_SPI_DRIVER) || defined(USE_DOUT_BITBANG_DRIVER)
	if (_spi_device != nullptr) {
		noInterrupts();
		// TODO: instead of copying state over buffer, its a better approach to merge everything from state into buffer
		memcpy(_digital_output_buffer, _digital_output_state, sizeof(_digital_output_buffer)*_chain_size);
		//i=_chain_size-1;
		//while(i >= 0) {
		//	_digital_output_buffer[i] = _digital_output_state[i];
		//	i--;
		//}
		_flush_dout = true;
		interrupts();
	}
//#endif

	// should we flush using realtime resource shared?
	// better not!
	//flush(0);
}

void Dout::flush(uint8_t interrupted)
{
	int8_t i = 0;

	if (_flush_dout == false) {
		return;
	}
	
#if defined(USE_DOUT_BITBANG_DRIVER)
	// active device
	digitalWrite(DOUT_LATCH_PIN, LOW);
	// Transfer byte a byte, in inverse order
	i=_chain_size-1;
	while(i >= 0) {
		shiftOut(DOUT_DATA_PIN, DOUT_CLOCK_PIN, MSBFIRST, _digital_output_buffer[i]);
		i--;
	}
	// deactive device
	digitalWrite(DOUT_LATCH_PIN, HIGH);
#endif 
	if (_spi_device != nullptr) {
		// we are always running inside IRS, if is shared make sure no one will try to handle while we do it
		if ( _is_shared ) { 
			noInterrupts();
		} 
		_spi_device->beginTransaction(SPISettings(SPI_SPEED_DOUT, MSBFIRST, SPI_MODE_DOUT));
		// active device
		digitalWrite(_latch_pin, LOW);
		// Transfer byte a byte, in inverse order
		//_spi_device->transfer(_digital_output_buffer, _chain_size); // this way we dont transfer inversed
		i=_chain_size-1;
		while(i >= 0) {
			_spi_device->transfer(_digital_output_buffer[i]);
			i--;
		}
		// deactive device
		digitalWrite(_latch_pin, HIGH);
		_spi_device->endTransaction(); 
		if ( _is_shared ) { 
			interrupts();
		}
	}
	// wait for the next change request
	_flush_dout = false;
}

void Dout::write(uint8_t remote_port, uint8_t value, uint8_t interrupted)
{
	uint8_t chain_group, chain_group_index;
	uint8_t * buffer = nullptr;

	if ( _remote_digital_output_port == 0 ) 
		return;	
	
	if (remote_port < _chain_size_pin) {
		digitalWrite(_dout_pin_map[remote_port], value);
		return;
	}
	// fix reference for bitmap read later
	remote_port -= _chain_size_pin;

//#if defined(USE_DOUT_SPI_DRIVER) || defined(USE_DOUT_BITBANG_DRIVER)
	if (_spi_device != nullptr) {
		if (interrupted == 0) {
			buffer = _digital_output_state;
		} else {
			buffer = _digital_output_buffer;
		}

		// The bitmap access key and intramap counter
		chain_group = floor(remote_port / 8);
		// Walk in range of bits inside bitmap
		chain_group_index = remote_port % 8;
		
		// state changed?
		if ( BIT_GET_VALUE(buffer[chain_group], chain_group_index) != value ) {		
			if ( value == 0) {
				buffer[chain_group] &= ~(1 << chain_group_index); 
			} else if (value == 1) {
				buffer[chain_group] |= (1 << chain_group_index);
			}
			
			if (interrupted == 0) {
				_change_flag = true;
			} else {
				_flush_dout = true;
				// let the main uCtrl ISR take care of flushing to avoid spi usage conflict
				//flush(interrupted);
			}
		}
	}
//#endif
}

void Dout::writeAll(uint8_t value, uint8_t interrupted)
{
	uint8_t * buffer = nullptr;

	if ( _remote_digital_output_port == 0 ) {
		return;	
	}

	for (uint8_t i=0; i < _chain_size_pin; i++) {
		digitalWrite(_dout_pin_map[i], value);
	}

//#if defined(USE_DOUT_SPI_DRIVER) || defined(USE_DOUT_BITBANG_DRIVER)
	if (_spi_device != nullptr) {
		if ( value == 0 ) {
			value = 0x00;
		} else if ( value == 1 ) {
			value = 0xFF;
		}

		if (interrupted == 0) {
			buffer = _digital_output_state;
		} else {
			buffer = _digital_output_buffer;
		}

		for (uint8_t i=0; i < _chain_size; i++) {
			if (buffer[i] != value) {
				buffer[i] = value;
			}
		}
		
		if (interrupted == 0) {
			_change_flag = true;
		} else {
			_flush_dout = true;
			// let the main uCtrl ISR take care of flushing to avoid spi usage conflict
			//flush(interrupted);
		}
	}
//#endif	
}

void Dout::setTimer(uint32_t time) {
	// timmer dependent UI visual effects
	if ( (time - _blink_timer) > BLINK_TIME ) {
		_blink = !_blink;
		_blink_timer = time;
	}
}

bool Dout::blink()
{
	return _blink;
}

} }

/*
void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val)
{
        uint8_t i;

        for (i = 0; i < 8; i++)  {
                if (bitOrder == LSBFIRST)
                        digitalWriteFast(dataPin, !!(val & (1 << i)));
                else
                        digitalWriteFast(dataPin, !!(val & (1 << (7 - i))));

                digitalWriteFast(clockPin, HIGH);
                digitalWriteFast(clockPin, LOW);
        }
}
*/
