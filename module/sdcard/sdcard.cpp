/*
 we need 2 ways of handling files
  + binary mode for fast retrieve of data on memory ram format(to use with stepsequencer for exemple)
  + human readble mode: for easy access to device configuration users need to be able to
    open the files like session, controller maps and others and edit on common text editor(to be used
    on controller maps and session config for example).
 

*/ 
#include "sdcard.hpp"

#if !defined(SDCARD_BITBANG_DRIVER)
#include "sdspi.hpp"
#endif


//
// multicore archs
//
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
	portMUX_TYPE _uctrlTimerMux = portMUX_INITIALIZER_UNLOCKED;
	#define ATOMIC(X) portENTER_CRITICAL_ISR(&_uctrlTimerMux); X; portEXIT_CRITICAL_ISR(&_uctrlTimerMux);
	#define ATOMIC_START portENTER_CRITICAL_ISR(&_uctrlTimerMux);
	#define ATOMIC_END portEXIT_CRITICAL_ISR(&_uctrlTimerMux);
//
// singlecore archs
//
#else // defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
	#define ATOMIC(X) noInterrupts(); X; interrupts();
	#define ATOMIC_START noInterrupts();
	#define ATOMIC_END interrupts();
#endif // defined(ARDUINO_ARCH_ESP32) || defined(ESP32)

namespace uctrl { namespace module {

SdCard::SdCard()
{

}

SdCard::~SdCard()
{

}
				
void SdCard::plug()
{
  // ???
}				

void SdCard::init(SPIClass * spi_device, uint8_t chip_select, bool is_shared)
{  
  _buffer_count = 0;
  _is_shared = is_shared;

#if defined(SDCARD_BITBANG_DRIVER)
    // init sdcard
    if (!_sd_fat.begin(SdSpiConfig(chip_select, DEDICATED_SPI, SD_SCK_MHZ(0), &_soft_spi))) {
      //_sd_fat.initErrorHalt();
      _sd_fat.initErrorPrint();
    }
#else
    // init sdcard
    sdSpi.setDevice(spi_device);
    // SHARED_SPI, DEDICATED_SPI
    if (!_sd_fat.begin(SdSpiConfig(chip_select, SHARED_SPI, SD_SCK_MHZ(4), &sdSpi))) {
    //if (!_sd_fat.begin(ram_module._chip_select)) {
      //_sd_fat.initErrorHalt();
      _sd_fat.initErrorPrint();
    }
#endif  

  // get sdfat cache data address (512 bytes cache)
  //_cache = (uint8_t*)_sd_fat.vol()->cacheAddress()->data;
}

bool SdCard::openFile(const char * path, uint8_t oflags, uint8_t interrupted)
{
  if ( oflags == 1 ) {
    oflags = O_WRITE | O_CREAT;
    if ( interrupted == 0 && _is_shared ) ATOMIC_START
    _sd_fat.remove(path);
    if ( interrupted == 0 && _is_shared ) ATOMIC_END
  } else if ( oflags == 0 ) {
    oflags = O_READ;
  }

  if ( interrupted == 0 && _is_shared ) ATOMIC_START
  _file = _sd_fat.open(path, oflags);
  if ( interrupted == 0 && _is_shared ) ATOMIC_END

  if (_file) {
    return true;
  }
  return false;
}

bool SdCard::closeFile(uint8_t interrupted)
{
  if (_file) {

    if ( interrupted == 0 && _is_shared ) ATOMIC_START
    _file.close();
    if ( interrupted == 0 && _is_shared ) ATOMIC_END

    return true;
  }
  return false;
}

uint8_t * SdCard::getData(size_t buffer_size, uint8_t interrupted)
{
  uint8_t ret = 0;
  if ( interrupted == 0 && _is_shared ) ATOMIC_START
  ret = _file.read((char*)_cache, buffer_size);
  if ( interrupted == 0 && _is_shared ) ATOMIC_END
  if ( ret < buffer_size ) {
    return NULL;
  } else {
    return _cache;
  }
}

bool SdCard::setData(uint8_t * buffer, size_t buffer_size, uint8_t interrupted)
{
  uint8_t ret = 0;

  // buffer == NULL is the end of file setData
  if ( buffer == NULL ) {
    // zeroing our buffer count
    _buffer_count = 0;
    return false;
  }
  
  // if we reach the 512 bytes boundary flush our data 
  if ( (_buffer_count + buffer_size) > 512 ) {

    if ( interrupted == 0 && _is_shared ) ATOMIC_START
    _file.flush();
    if ( interrupted == 0 && _is_shared ) ATOMIC_END
    
    _buffer_count = 0;
  }
    
  // write data to sdcard
  if ( interrupted == 0 && _is_shared ) ATOMIC_START
  ret = _file.write(buffer, buffer_size);
  if ( interrupted == 0 && _is_shared ) ATOMIC_END
  
  if ( ret < buffer_size ) {
    return NULL;
  }
  
  // increment buffer counter
  _buffer_count += buffer_size;

  return true;
}

bool SdCard::readTextLine(char * line, char * str, size_t size, uint8_t field_num, char field_delim) 
{
  uint8_t index = 0;
  
  if ( field_num > 0 ) {
    // find the after field_delim index to read from line[]
    for ( index=0; index < BUFFER_SIZE; index++ ) {
      if ( line[index] == field_delim ) {
        field_num--;
        if ( field_num == 0 ) {
          index++;
          break;
        }
      } else if ( line[index] == '\n') {
        return false;
      }
    }
  }

  for ( uint8_t i=0; i < size; i++ ) {
    if ( line[index+i] == '\n' || line[index+i] == field_delim || i == (size-1) ) {
      str[i] = '\0';
      break;
    }
    str[i] = line[index+i];
  }

  return true;
}

bool SdCard::getConfig(char * string, uint8_t size, uint8_t field_num, uint8_t interrupted)
{
  uint8_t ret = 0;

  if ( interrupted == 0 && _is_shared ) ATOMIC_START
  ret = _file.available();
  if ( interrupted == 0 && _is_shared ) ATOMIC_END

  if ( (!ret && field_num == 0) || size > BUFFER_SIZE) {
      return false;
  }
  
  // do only process next file line into _cache if field_num == 0
  if ( field_num == 0 ) {

    if ( interrupted == 0 && _is_shared ) ATOMIC_START
    ret = _file.fgets((char*)_cache, BUFFER_SIZE);
    if ( interrupted == 0 && _is_shared ) ATOMIC_END

    if ( ret <= 0 ) { // EOF=0 error=-1
      return false;
    }
  }
  
  return readTextLine((char*)_cache, string, size, field_num, ',');
}

bool SdCard::getConfig(int16_t * number, uint8_t field_num, uint8_t interrupted)
{
  char buf[BUFFER_SIZE];
  char * ptr;
  uint8_t ret = 0;

  if ( interrupted == 0 && _is_shared ) ATOMIC_START
  ret = _file.available();
  if ( interrupted == 0 && _is_shared ) ATOMIC_END

  if ( !ret && field_num == 0 ) {
      return false;
  } 
  
  // do only process next file line into _cache if field_num == 0
  if ( field_num == 0 ) {   

    if ( interrupted == 0 && _is_shared ) ATOMIC_START
    ret = _file.fgets((char*)_cache, BUFFER_SIZE);
    if ( interrupted == 0 && _is_shared ) ATOMIC_END

    if ( ret <= 0 ) { // EOF=0 error=-1
      return false;
    }
  }
  
  //is the base for the number represented in the string. A "base" of zero indicates that the base should be determined from the leading digits of "s". The default is decimal, a leading '0' indicates octal, and a leading '0x' or '0X' indicates hexadecimal. 
  
  if ( readTextLine((char*)_cache, buf, sizeof(buf), field_num, ',') ) {
    *number = (int16_t)strtol(buf, &ptr, 0); 
    return true;
  }
  
  return false;
}

bool SdCard::getConfig(float * number, uint8_t field_num, uint8_t interrupted)
{
  char buf[BUFFER_SIZE];
  char * ptr;
  uint8_t ret = 0;
  
  if ( interrupted == 0 && _is_shared ) ATOMIC_START
  ret = _file.available();
  if ( interrupted == 0 && _is_shared ) ATOMIC_END

  if ( !ret && field_num == 0 ) {
      return false;
  } 
  
  // do only process next file line into _cache if field_num == 0
  if ( field_num == 0 ) {
    
    if ( interrupted == 0 && _is_shared ) ATOMIC_START
    ret = _file.fgets((char*)_cache, BUFFER_SIZE);
    if ( interrupted == 0 && _is_shared ) ATOMIC_END

    if ( ret <= 0 ) { // EOF=0 error=-1
      return false;
    }
  }
  
  if ( readTextLine((char*)_cache, buf, sizeof(buf), field_num, ',') ) {
    *number = (float)strtod(buf, &ptr);
    return true;
  }
  
  return false;  
}

bool SdCard::setConfig(char * string, uint8_t size, uint8_t field_num, uint8_t interrupted)
{
  if ( interrupted == 0 && _is_shared ) ATOMIC_START 
  // write data to sdcard
  //bool result = _file.println(string);
  _file.println(string);
	if ( interrupted == 0 && _is_shared ) ATOMIC_END
 
  //return result;
  return true;
}

bool SdCard::remove(char * path, uint8_t interrupted)
{
  if ( interrupted == 0 && _is_shared ) ATOMIC_START 
  bool result = _sd_fat.remove(path);
  if ( interrupted == 0 && _is_shared ) ATOMIC_END

  return result;
  return true;
}

bool SdCard::chdir(char * path, uint8_t interrupted)
{
  if ( interrupted == 0 && _is_shared ) ATOMIC_START 
  bool result = _sd_fat.chdir(path);
	if ( interrupted == 0 && _is_shared ) ATOMIC_END

  return result;
  return true;
}

bool SdCard::exists(char * path, uint8_t interrupted)
{
  if ( interrupted == 0 && _is_shared ) ATOMIC_START 
  bool result = _sd_fat.exists(path);
	if ( interrupted == 0 && _is_shared ) ATOMIC_END

  return result;
  return true;
}	


} }

//uctrl::module::SdCard sdcard_module;