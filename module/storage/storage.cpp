/*!
 *  @file       storage.cpp
 *  Project     Arduino Library API interface for uMODULAR projects
 *  @brief      Generic storage module for EPPROM and SDCARD
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

#include "storage.hpp"

namespace uctrl { namespace module {

Storage::Storage()
{
}

void Storage::init(SPIClass * spi_device, bool is_shared)
{
	if (spi_device != nullptr) {
		// we need to activate sdcard support
		_is_shared = is_shared;
	}
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
	EEPROM.begin(INTERNAL_MEMORY_SIZE);
#endif
}

bool Storage::save(void *data, size_t n, int address)
{
    uint8_t* src = (uint8_t*)data;

	if (address != -1)
		_current_address = address;

    for (uint16_t i = 0; i < n; i++)
		EEPROM.put(_current_address++, *src++);

#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
	// esp32 has no epprom, it emulates using flash area, commit to it!
	EEPROM.commit();
#endif
	
	return true;
}

bool Storage::load(void *data, size_t n, int address)
{
    uint8_t* dst = (uint8_t*)data;

	if (address != -1)
		_current_address = address;

    for (uint16_t i = 0; i < n; i++)
		EEPROM.get(_current_address++, *dst++);

	return true;
}

bool Storage::copy(int address_from, int address_to, size_t n)
{
	for (uint16_t i = 0; i < n; i++)
		EEPROM.write(address_to++, EEPROM.read(address_from++));

#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
	// esp32 has no epprom, it emulates using flash area, commit to it!
	EEPROM.commit();
#endif

	return true;
}

bool Storage::save(void *data, size_t n, const char * path)
{

}

bool Storage::load(void *data, size_t n, const char * path)
{

}

} }

//uctrl::module::Storage storage_module;