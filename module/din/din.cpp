/*!
 *  @file       din.cpp
 *  Project     Arduino Library API interface for uMODULAR projects
 *  @brief      Digital input driver module (165 shiftregister)
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

#ifdef USE_DIN

#include "din.hpp"

namespace uctrl { namespace module { namespace din {
    
Din::Din()
{

}

Din::~Din()
{
	
}

uint8_t Din::sizeOf()
{
	return _remote_digital_port;
}			

void Din::plug(uint8_t setup)
{
#if defined(DIN_BITBANG_DRIVER) || defined(DIN_SPI_DRIVER)
	_chain_size = setup;
	_remote_digital_port = _chain_size * 8;
#else
	if (_remote_digital_port < USE_DIN_MAX_PORTS) {
		_button_pin[_remote_digital_port] = setup;
		_remote_digital_port++;
		_chain_size = floor(_remote_digital_port/8)+1;
	}
#endif
}

void Din::encoder(uint8_t channel_a_id, uint8_t channel_b_id)
{
	uint8_t state_group;

	if (use_encoder == false) {
		use_encoder = true;
		// we need to init din drivers here to malloc heap data structures
		init();
	}

	// TODO: take off this decrement shit, do it in programmer way for everything
	channel_a_id--;
	channel_b_id--;

	// find our indexes
	state_group = floor(channel_a_id/8);

	if (channel_a_id % 2 == 1) {
		// no channel A id odd here...
		return;
	}

	if (channel_b_id - channel_a_id == 1) {
		// a pair register call
		_digital_detent_pin[state_group] |= 1 << (channel_a_id % 8);
		_digital_detent_pin[state_group] |= 1 << (channel_b_id % 8);
	} else {
		// a range of pairs register call(or invalid call)

		// check for parameters concistence for range of pairs
		//...
	}

}

void Din::init()
{
	// are we init our selfs before because of encoders call?
	if (initiate) {
		return;
	}

#if defined(DIN_BITBANG_DRIVER)
	pinModeFast(DIN_LATCH_PIN, OUTPUT);
	pinModeFast(DIN_DATA_PIN, INPUT); 
	pinModeFast(DIN_CLOCK_PIN, OUTPUT);	
	digitalWriteFast(DIN_LATCH_PIN, HIGH);	
#elif defined(DIN_SPI_DRIVER)
	pinMode(_chip_select, OUTPUT);
	digitalWrite(_chip_select, HIGH);	
	// Initing a common shared SPI device for all uMODULAR modules using SPI bus
	_spi_device->begin();
#else
	// walk port reference structure and setup PINs as PULLUP/INPUT
	for (uint8_t i=0; i < _remote_digital_port; i++ ) {
		pinMode(_button_pin[i], INPUT_PULLUP);
	}
#endif

	// For each 8 buttons alloc 1 byte memory area state data and other 1 byte for last state data.
	// Each bit represents the value state readed by digital inputs	
	if ( _remote_digital_port > 0 ) {
		for (uint8_t i=0; i < _chain_size; i++) {
			_digital_input_state[i] = 0;
			_digital_input_last_state[i] = 0;
			if (use_encoder) {
				_digital_detent_pin[i] = 0;
			}
		}
		// init registers with first scan
		read(0);
	}

	event_queue.head = 0;
	event_queue.tail = 0;
	event_queue.size = EVENT_QUEUE_SIZE;

	initiate = true;
}

#if defined(DIN_SPI_DRIVER)
void Din::setSpi(SPIClass * spi_device, uint8_t chip_select)
{
	// HARDWARE NOTES
	// For those using a SPI device for other devices than 165:
	// since the 165 are not spi compilant we need a 2.2k resistor on MISO line to not screw up other spi devices on same MISO pin
	_spi_device = spi_device;
	// Chip select pin setup
	_chip_select = chip_select;
}
#endif

// make it bool...
// if (read()) means something change...
// so we can discard non encoders 
// using skip control var
// check if we have encoders for fast read...
// if we dont, check skip counter and increment 
// it(or zero it and reaches the value state)
//
// Read all the stream DIN bytes
void Din::read(uint8_t interrupted)
{
	static bool state_change;
	state_change = false;
#if defined(DIN_BITBANG_DRIVER)
	//if ( interrupted == 0 ) {
	//	_tmpSREG = SREG;
	//	cli();
	//}
	// pulsing the chip select pin to start capturing data
	// force reset for clock edge on rising
	digitalWriteFast(DIN_CLOCK_PIN, HIGH);
	// latch and load!
	digitalWriteFast(DIN_LATCH_PIN, LOW);
	digitalWriteFast(DIN_LATCH_PIN, HIGH);
	
	// Read byte per byte
	for (uint8_t i=0; i < _chain_size; i++) {
		// Before refresh data, set the last state data
		_digital_input_last_state[i] = _digital_input_state[i];		
		// shift in that byte
		//_digital_input_state[i] = (uint8_t) shiftIn(DIN_DATA_PIN, DIN_CLOCK_PIN, MSBFIRST); 				
		_digital_input_state[i] = 0;
		for (uint8_t j=0; j < 8; ++j) {
			digitalWriteFast(DIN_CLOCK_PIN, HIGH);
			_digital_input_state[i] |= digitalRead(DIN_DATA_PIN) << (7 - j);
			// buggy digitalReadFast() or too fast read just after clock the device?
			//_digital_input_state[i] |= digitalReadFast(DIN_DATA_PIN) << (7 - j);
			digitalWriteFast(DIN_CLOCK_PIN, LOW);
		}				
		if (state_change == false && _digital_input_last_state[i] != _digital_input_state[i]) {
			state_change = true;
		}
	}
	//if ( interrupted == 0 ) {
	//	SREG = _tmpSREG;	
	//}
#elif defined(DIN_SPI_DRIVER)
	//if ( interrupted == 0 ) {	
	//	_spi_device->notUsingInterrupt(255);
	//} else {	
	//	_spi_device->usingInterrupt(255);
	//}
	_spi_device->beginTransaction(SPISettings(SPI_SPEED_DIN, MSBFIRST, SPI_MODE_DIN));
	// pulsing the chip select pin to start capturing data
	digitalWrite(_chip_select, LOW);
	digitalWrite(_chip_select, HIGH);
	// Read byte per byte
	for (uint8_t i=0; i < _chain_size; i++) {
		// Before refresh data, set the last state data
		_digital_input_last_state[i] = _digital_input_state[i];		
		_digital_input_state[i] = (uint8_t) _spi_device->transfer(0x00); 
		if (state_change == false && _digital_input_last_state[i] != _digital_input_state[i]) {
			state_change = true;
		}
	}
	_spi_device->endTransaction();
#else
	// Read port per port
	uint8_t remote_port = 0;
	for (uint8_t i=0; i < _chain_size; i++) {
		// Before refresh data, set the last state data
		_digital_input_last_state[i] = _digital_input_state[i];				
		_digital_input_state[i] = 0;
		for (uint8_t j=0; j < 8; j++) {
			remote_port = (i*8)+j;
			if (remote_port >= _remote_digital_port) {
				break;
			}
			_digital_input_state[i] |= digitalRead(_button_pin[remote_port]) << j;
		}				
		if (state_change == false && _digital_input_last_state[i] != _digital_input_state[i]) {
			state_change = true;
		}
	}
#endif	
	if (state_change) {
		processQueue();
	}
}

// runs inside interruption
void Din::processQueue()
{
	int16_t value;
	uint8_t port;

	// runs on each byte of array and make check
	for (uint8_t i=0; i < _chain_size; i++) {
		if (_digital_input_last_state[i] != _digital_input_state[i]) {
			for (uint8_t j=0; j < 8; j++) {
				if ( BIT_VALUE(_digital_input_state[i], j) != BIT_VALUE(_digital_input_last_state[i], j) ) {
					// we got a change
					port = (i*8)+j;
					value = !BIT_VALUE(_digital_input_state[i], j);

					// identify if we have a detend encoder pin case here
					// use a #define case here for detent driver usage
					if (_digital_detent_pin != nullptr) {
						if (_digital_detent_pin[i] != 0) {
							//  detent pin?
							if (BIT_VALUE(_digital_detent_pin[i], j) == 1) {
								// A or B channels?
								// we register then always in pairs(no matter what)
								// check against last state for a more stable read(thats polling!)
								if (j % 2 == 0) {
									// even: channel_a/decrement
									value = value > !BIT_VALUE(_digital_input_last_state[i], j+1) ? 1 : 0;
								} else {
									// odd: channel_b/increment
									value = value > !BIT_VALUE(_digital_input_last_state[i], j-1) ? 1 : 0;
								}
							}
						}
					}

					uint8_t tail = (event_queue.tail+1) >= event_queue.size ? 0 : (event_queue.tail+1);

					if ( event_queue.head == tail )
					{
						// dropping event, full queue
						continue;
					}
					event_queue.event[event_queue.tail].port = port;
					event_queue.event[event_queue.tail].value = value;  
					event_queue.tail = tail; 
				}
			}
		}
	}
}

int8_t Din::getData(uint8_t port)
{
	uint8_t state_group, state_port;

	if ( _remote_digital_port == 0 ) 
		return -1;

	// find our indexes
	state_group = floor(port/8);	
	state_port = port%8;

	// Check for digital state changes
	if ( BIT_VALUE(_digital_input_state[state_group], state_port) != BIT_VALUE(_digital_input_last_state[state_group], state_port) ) {
		return !BIT_VALUE(_digital_input_state[state_group], state_port);
	}
	
	return -1;
}

int8_t Din::getDataRaw(uint8_t port)
{
	if ( _remote_digital_port == 0 ) 
		return -1;	

	return !BIT_VALUE( _digital_input_state[(uint8_t)(floor(port / 8))], (port % 8) );
}

} } }

uctrl::module::din::Din din_module;
#endif