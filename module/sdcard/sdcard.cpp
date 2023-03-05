#include "../../../../modules.h"

#ifdef USE_SDCARD
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

void SdCard::init(SPIClass * spi_device, uint8_t chip_select)
{  
  _buffer_count = 0;

#if defined(SDCARD_BITBANG_DRIVER)
    // init sdcard
    if (!_sd_fat.begin(SdSpiConfig(SDCARD_SOFT_CHIP_SELECT, DEDICATED_SPI, SD_SCK_MHZ(0), &_soft_spi))) {
      //_sd_fat.initErrorHalt();
      _sd_fat.initErrorPrint();
    }
#else
    // init sdcard
    sdSpi.setDevice(spi_device);
    // SHARED_SPI, DEDICATED_SPI
    if (!_sd_fat.begin(SdSpiConfig(chip_select, SHARED_SPI, SD_SCK_MHZ(4), &sdSpi))) {
    //if (!_sd_fat.begin(chip_select)) {
      //_sd_fat.initErrorHalt();
      _sd_fat.initErrorPrint();
    }
#endif  

  // get sdfat cache data address (512 bytes cache)
  //_cache = (uint8_t*)_sd_fat.vol()->cacheAddress()->data;
}

bool SdCard::openFile(const char * path, uint8_t oflags)
{
  if ( oflags == 1 ) {
    oflags = O_WRITE | O_CREAT;
    _sd_fat.remove(path);
  } else if ( oflags == 0 ) {
    oflags = O_READ;
  }
  if ( _file = _sd_fat.open(path, oflags) ) {
    return true;
  }
  return false;
}

bool SdCard::closeFile()
{
  if (_file) {
    _file.close();
    return true;
  }
  return false;
}

uint8_t * SdCard::getData(size_t buffer_size)
{
  if ( _file.read(_cache, buffer_size) < buffer_size ) {
    return NULL;
  } else {
    return _cache;
  }
}

bool SdCard::setData(uint8_t * buffer, size_t buffer_size)
{
  // buffer == NULL is the end of file setData
  if ( buffer == NULL ) {
    // zeroing our buffer count
    _buffer_count = 0;
    return false;
  }
  
  // if we reach the 512 bytes boundary flush our data 
  if ( (_buffer_count + buffer_size) > 512 ) {
    _file.flush();
    _buffer_count = 0;
  }
    
  // write data to sdcard
  if ( _file.write(buffer, buffer_size) < buffer_size ) {
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

bool SdCard::getConfig(char * string, uint8_t size, uint8_t field_num)
{
  if ( (!_file.available() && field_num == 0) || size > BUFFER_SIZE) {
      return false;
  }
  
  // do only process next file line into _cache if field_num == 0
  if ( field_num == 0 ) {
    if ( _file.fgets(_cache, BUFFER_SIZE) <= 0 ) { // EOF=0 error=-1
      return false;
    }
  }
  
  return readTextLine(_cache, string, size, field_num, ',');
}

bool SdCard::getConfig(int16_t * number, uint8_t field_num)
{
  char buf[BUFFER_SIZE];
  char * ptr;
  
  if ( !_file.available() && field_num == 0 ) {
      return false;
  } 
  
  // do only process next file line into _cache if field_num == 0
  if ( field_num == 0 ) {
    if ( _file.fgets(_cache, BUFFER_SIZE) <= 0 ) { // EOF=0 error=-1
      return false;
    }
  }
  
  //is the base for the number represented in the string. A "base" of zero indicates that the base should be determined from the leading digits of "s". The default is decimal, a leading '0' indicates octal, and a leading '0x' or '0X' indicates hexadecimal. 
  
  if ( readTextLine(_cache, buf, sizeof(buf), field_num, ',') ) {
    *number = (int16_t)strtol(buf, &ptr, 0); 
    return true;
  }
  
  return false;
}

bool SdCard::getConfig(float * number, uint8_t field_num)
{
  char buf[BUFFER_SIZE];
  char * ptr;
  
  if ( !_file.available() && field_num == 0 ) {
      return false;
  } 
  
  // do only process next file line into _cache if field_num == 0
  if ( field_num == 0 ) {
    if ( _file.fgets(_cache, BUFFER_SIZE) <= 0 ) { // EOF=0 error=-1
      return false;
    }
  }
  
  if ( readTextLine(_cache, buf, sizeof(buf), field_num, ',') ) {
    *number = (float)strtod(buf, &ptr);
    return true;
  }
  
  return false;  
}

bool SdCard::setConfig(char * string, uint8_t size, uint8_t field_num)
{
  // write data to sdcard
  _file.println(string);

  return true;
}

bool SdCard::remove(char * path)
{
  return _sd_fat.remove(path);
}

bool SdCard::chdir(char * path)
{
  return _sd_fat.chdir(path);
}

bool SdCard::exists(char * path)
{
  return _sd_fat.exists(path);
}	


} }

uctrl::module::SdCard sdcard_module;
#endif
