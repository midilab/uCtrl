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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE. 
 */

#include "../../uCtrl.h"

#if defined(USE_STORAGE_MODULE)

#include "storage.hpp"

#define STORAGE_CHUNK 64
#define SESSION_PATH_ROOT "/ACIDUINO/"
#define SESSION_PATH_LAST SESSION_PATH_ROOT "LAST.BIN"
#define SESSION_MAX 1000

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

//
// sdcard backend helpers
//
bool Storage::sdReady()
{
#if defined(USE_SDCARD_MODULE)
	return uCtrl.sdcard != nullptr && uCtrl.sdcard->isReady();
#else
	return false;
#endif
}

void Storage::sdSessionPath(char * buffer, uint16_t index, bool config)
{
	snprintf(buffer, 24, SESSION_PATH_ROOT "S%03u.%s", index, config ? "CFG" : "BIN");
}

bool Storage::isSdActive()
{
	return sdReady();
}

uint32_t Storage::size()
{
	if (sdReady()) {
		char path[24];
		sdSessionPath(path, _active_session, false);
		if (!uCtrl.sdcard->exists(path)) {
			return 0;
		}
		uint32_t file_size = 0;
		if (uCtrl.sdcard->openFile(path, 0)) {
			file_size = uCtrl.sdcard->size();
			uCtrl.sdcard->closeFile();
		}
		return file_size;
	}
	return EEPROM.length();
}

uint16_t Storage::getActiveSession()
{
	return _active_session;
}

uint16_t Storage::getSessionCount()
{
	if (!sdReady()) {
		return 0;
	}
	uint16_t count = 0;
	char path[24];
	for (uint16_t i = 0; i < SESSION_MAX; i++) {
		sdSessionPath(path, i, false);
		if (!uCtrl.sdcard->exists(path)) {
			break;
		}
		count++;
	}
	return count;
}

bool Storage::useSession(uint16_t index, bool create)
{
	if (!sdReady() || index >= SESSION_MAX) {
		return false;
	}

	char path[24];
	sdSessionPath(path, index, false);

	if (!uCtrl.sdcard->exists(path)) {
		if (!create) {
			return false;
		}
		if (!uCtrl.sdcard->exists(SESSION_PATH_ROOT)) {
			uCtrl.sdcard->mkdir(SESSION_PATH_ROOT);
		}
		// create empty image file
		if (!uCtrl.sdcard->openFile(path, 2)) {
			return false;
		}
		uCtrl.sdcard->closeFile();
	}

	_active_session = index;
	// remember the active session across reboots
	return save((void*)&_active_session, sizeof(_active_session), SESSION_PATH_LAST);
}

bool Storage::restoreSession()
{
	if (!sdReady()) {
		return false;
	}
	uint16_t index = 0;
	if (!load((void*)&index, sizeof(index), SESSION_PATH_LAST) || index >= SESSION_MAX) {
		index = 0;
	}
	return useSession(index, true);
}

bool Storage::hasEppromData(uint16_t valid_marker)
{
	uint16_t marker = 0;
	// direct epprom read: backend agnostic check
	EEPROM.get(0, marker);
	return marker == valid_marker;
}

bool Storage::importFromEpprom(uint32_t n)
{
	if (!sdReady()) {
		return false;
	}

	char path[24];
	sdSessionPath(path, _active_session, false);

	// fresh image file from epprom content(epprom layout is the sdcard image layout)
	if (!uCtrl.sdcard->openFile(path, 1)) {
		return false;
	}
	// reset write buffer state
	uCtrl.sdcard->setData(NULL, 0);

	uint8_t buffer[STORAGE_CHUNK];
	for (uint32_t i = 0; i < n; i += STORAGE_CHUNK) {
		size_t chunk = n - i > STORAGE_CHUNK ? STORAGE_CHUNK : n - i;
		for (size_t j = 0; j < chunk; j++) {
			buffer[j] = EEPROM.read((int)(i + j));
		}
		if (!uCtrl.sdcard->setData(buffer, chunk)) {
			uCtrl.sdcard->closeFile();
			return false;
		}
	}

	uCtrl.sdcard->closeFile();
	return true;
}

//
// address mapped access
//
bool Storage::sdWriteAt(void *data, size_t n, uint32_t address)
{
	char path[24];
	sdSessionPath(path, _active_session, false);

	if (!uCtrl.sdcard->openFile(path, 2)) {
		return false;
	}
	// reset write buffer state
	uCtrl.sdcard->setData(NULL, 0);

	uint32_t file_size = uCtrl.sdcard->size();

	// zero fill the gap when writing beyond current end(new pending pattern slots)
	if (address > file_size) {
		uint8_t zeros[STORAGE_CHUNK] = {0};
		if (!uCtrl.sdcard->seek(file_size)) {
			uCtrl.sdcard->closeFile();
			return false;
		}
		while (file_size < address) {
			size_t chunk = address - file_size > STORAGE_CHUNK ? STORAGE_CHUNK : address - file_size;
			if (!uCtrl.sdcard->setData(zeros, chunk)) {
				uCtrl.sdcard->closeFile();
				return false;
			}
			file_size += chunk;
		}
	}

	if (!uCtrl.sdcard->seek(address)) {
		uCtrl.sdcard->closeFile();
		return false;
	}

	uint8_t *src = (uint8_t*)data;
	while (n) {
		size_t chunk = n > STORAGE_CHUNK ? STORAGE_CHUNK : n;
		if (!uCtrl.sdcard->setData(src, chunk)) {
			uCtrl.sdcard->closeFile();
			return false;
		}
		src += chunk;
		n -= chunk;
	}

	uCtrl.sdcard->closeFile();
	return true;
}

bool Storage::sdReadAt(void *data, size_t n, uint32_t address)
{
	char path[24];
	sdSessionPath(path, _active_session, false);

	if (!uCtrl.sdcard->openFile(path, 0)) {
		return false;
	}
	if (!uCtrl.sdcard->seek(address)) {
		uCtrl.sdcard->closeFile();
		return false;
	}

	uint8_t *dst = (uint8_t*)data;
	uint8_t *cache;
	while (n) {
		size_t chunk = n > STORAGE_CHUNK ? STORAGE_CHUNK : n;
		cache = uCtrl.sdcard->getData(chunk);
		if (cache == NULL) {
			uCtrl.sdcard->closeFile();
			return false;
		}
		memcpy(dst, cache, chunk);
		dst += chunk;
		n -= chunk;
	}

	uCtrl.sdcard->closeFile();
	return true;
}

bool Storage::save(void *data, size_t n, uint32_t address)
{
	if (address != (uint32_t)-1)
		_current_address = address;

	if (sdReady()) {
		return sdWriteAt(data, n, _current_address);
	}

	uint8_t* src = (uint8_t*)data;
	for (size_t i = 0; i < n; i++) {
		EEPROM.put((int)_current_address++, *src++);
	}

#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
	// esp32 has no epprom, it emulates using flash area, commit to it!
	EEPROM.commit();
#endif

	return true;
}

bool Storage::load(void *data, size_t n, uint32_t address)
{
	if (address != (uint32_t)-1)
		_current_address = address;

	if (sdReady()) {
		return sdReadAt(data, n, _current_address);
	}

	uint8_t* dst = (uint8_t*)data;
	for (size_t i = 0; i < n; i++) {
		EEPROM.get((int)_current_address++, *dst++);
	}

	return true;
}

bool Storage::copy(uint32_t address_from, uint32_t address_to, size_t n)
{
	if (sdReady()) {
		uint8_t buffer[STORAGE_CHUNK];
		while (n) {
			size_t chunk = n > STORAGE_CHUNK ? STORAGE_CHUNK : n;
			if (!sdReadAt(buffer, chunk, address_from) || !sdWriteAt(buffer, chunk, address_to)) {
				return false;
			}
			address_from += chunk;
			address_to += chunk;
			n -= chunk;
		}
		return true;
	}

	for (size_t i = 0; i < n; i++) {
		EEPROM.write((int)(address_to++), EEPROM.read((int)(address_from++)));
	}

#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
	// esp32 has no epprom, it emulates using flash area, commit to it!
	EEPROM.commit();
#endif

	return true;
}

//
// raw file path access
//
bool Storage::save(void *data, size_t n, const char * path)
{
	if (!sdReady()) {
		return false;
	}

	if (!uCtrl.sdcard->openFile(path, 1)) {
		return false;
	}
	// reset write buffer state
	uCtrl.sdcard->setData(NULL, 0);

	uint8_t *src = (uint8_t*)data;
	size_t left = n;
	while (left) {
		size_t chunk = left > STORAGE_CHUNK ? STORAGE_CHUNK : left;
		if (!uCtrl.sdcard->setData(src, chunk)) {
			uCtrl.sdcard->closeFile();
			return false;
		}
		src += chunk;
		left -= chunk;
	}

	uCtrl.sdcard->closeFile();
	return true;
}

bool Storage::load(void *data, size_t n, const char * path)
{
	if (!sdReady()) {
		return false;
	}

	if (!uCtrl.sdcard->openFile(path, 0)) {
		return false;
	}

	uint8_t *dst = (uint8_t*)data;
	uint8_t *cache;
	size_t left = n;
	while (left) {
		size_t chunk = left > STORAGE_CHUNK ? STORAGE_CHUNK : left;
		cache = uCtrl.sdcard->getData(chunk);
		if (cache == NULL) {
			uCtrl.sdcard->closeFile();
			return false;
		}
		memcpy(dst, cache, chunk);
		dst += chunk;
		left -= chunk;
	}

	uCtrl.sdcard->closeFile();
	return true;
}

bool Storage::saveSessionConfig(void *data, size_t n)
{
	char path[24];
	sdSessionPath(path, _active_session, true);
	return save(data, n, path);
}

bool Storage::loadSessionConfig(void *data, size_t n)
{
	char path[24];
	sdSessionPath(path, _active_session, true);
	return load(data, n, path);
}

} }

//uctrl::module::Storage storage_module;

#endif
