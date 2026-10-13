/*!
 *  @file       storage.hpp
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

#ifndef __U_CTRL_STORAGE_HPP__
#define __U_CTRL_STORAGE_HPP__

#include <Arduino.h>
#include <SPI.h>
#include <EEPROM.h>

#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
// ESP32 family: maximum EEPROM size is 4096 bytes (4 KB)
#define INTERNAL_MEMORY_SIZE	4095
#endif

namespace uctrl { namespace module { 

class Storage
{
    public:

        Storage();

		void init(SPIClass * spi_device = nullptr, bool is_shared = false);

		// transparent byte address interface: backed by epprom or by the active sdcard session image
		bool save(void *data, size_t n, uint32_t address = (uint32_t)-1);
		bool load(void *data, size_t n, uint32_t address = (uint32_t)-1);
		bool copy(uint32_t address_from, uint32_t address_to, size_t n);

		// raw file access (sdcard only)
		bool load(void *data, size_t n, const char * path);
		bool save(void *data, size_t n, const char * path);

		// sdcard session management
		bool isSdActive();
		uint32_t size();
		uint16_t getActiveSession();
		uint16_t getSessionCount();
		bool useSession(uint16_t index, bool create = false);
		bool restoreSession();
		bool importFromEpprom(uint32_t n);
		bool hasEppromData(uint16_t valid_marker);
		// per session sidecar file: session stuff not part of the epprom compatible image
		bool saveSessionConfig(void *data, size_t n);
		bool loadSessionConfig(void *data, size_t n);

	private:

		uint32_t _current_address = 0;
		bool _is_shared = false;
		uint16_t _active_session = 0;

		bool sdReady();
		void sdSessionPath(char * buffer, uint16_t index, bool config);
		bool sdWriteAt(void *data, size_t n, uint32_t address);
		bool sdReadAt(void *data, size_t n, uint32_t address);
};
			
} }

//extern uctrl::module::Storage storage_module;
		
#endif
