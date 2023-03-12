#include "../../../../modules.h"

#ifdef USE_EXT_RAM

#include "ram.hpp"
	
namespace uctrl { namespace module {

Ram::Ram()
{
  
}

Ram::~Ram()
{

}

void Ram::init(SPIClass * device)
{
  //EXT_RAM_CHIP_SELECT
  _spi_device = device;   
  pinMode (EXT_RAM_CHIP_SELECT, OUTPUT);
  digitalWrite(EXT_RAM_CHIP_SELECT, HIGH);  
  
  _buffer_address_id_pointer = 0;
  _buffer_address_pointer = 0;
  //_runtime_buffer_address[0] = 0;
  //_runtime_buffer_address[1] = 0;
  
	// Initing a common shared SPI device for all uMODULAR modules using SPI bus
	_spi_device->begin();
	//_spi_device->setDataMode(SPI_MODE0);
	//_spi_device->setClockDivider(SPI_CLOCK_DIV128);

  //setMode(BYTE_MODE);
  //setMode(PAGE_MODE);
  //setMode(STREAM_MODE);  
}

static int8_t Ram::registerBuffer( uint16_t buffer_size, uint16_t buffer_length )
{
  uint8_t buffer_address_id = ram_module._buffer_address_id_pointer;

  // above 1Mbit boundaries?
  if ( (ram_module._buffer_address_pointer + ((uint32_t)buffer_size * (uint32_t)buffer_length)) > 131072 ) {
    return -1;
  }
  
  ram_module._buffer_layout[buffer_address_id].buffer_length = buffer_length;
  ram_module._buffer_layout[buffer_address_id].buffer_size = buffer_size;
  ram_module._buffer_layout[buffer_address_id].buffer_address = ram_module._buffer_address_pointer;
  ram_module._buffer_address_pointer += (uint32_t)buffer_size * (uint32_t)buffer_length;
  ram_module._buffer_address_id_pointer++;

  return buffer_address_id;
}

uint32_t Ram::getFreeRam()
{
  return ((uint32_t)131072) - ram_module._buffer_address_pointer;
}

static void Ram::read(uint8_t * buffer, uint16_t buffer_address, uint8_t buffer_id, uint8_t interrupted, uint8_t size_to_read, uint8_t start_at)
{
  uint32_t memory_address = ram_module._buffer_layout[buffer_id].buffer_address + ((uint32_t)buffer_address * (uint32_t)ram_module._buffer_layout[buffer_id].buffer_size) + (uint32_t)start_at;
  
  //_runtime_buffer_address[interrupted] = _buffer_layout[buffer_id].buffer_address + ((uint32_t)buffer_address * (uint32_t)_buffer_layout[buffer_id].buffer_size) + (uint32_t)start_at;
  //_set_address[interrupted].byte1 = READ;
  //_set_address[interrupted].byte2 = (uint8_t)(_runtime_buffer_address[interrupted] >> 16);
  //_set_address[interrupted].byte3 = (uint8_t)(_runtime_buffer_address[interrupted] >> 8);
  //_set_address[interrupted].byte4 = (uint8_t)_runtime_buffer_address[interrupted];
 
  if ( interrupted == 0 ) { 
    //ram_module._tmpSREG = SREG;
    //cli();
    ram_module._spi_device->usingInterrupt(255);
  } 

  ram_module._spi_device->beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE));
  digitalWrite(EXT_RAM_CHIP_SELECT, LOW);
  
  // send address request
  //_spi_device->transfer(&_set_address[interrupted], 4);
  ram_module._spi_device->transfer(READ);
  ram_module._spi_device->transfer((uint8_t)(memory_address >> 16));
  ram_module._spi_device->transfer((uint8_t)(memory_address >> 8));
  ram_module._spi_device->transfer((uint8_t)memory_address);
   
  if ( size_to_read == 0 ) {
    size_to_read = ram_module._buffer_layout[buffer_id].buffer_size;
  }
  
  // get data
  //for (_counter = 0; _counter < size_to_read; _counter++) {
  //  buffer[_counter+start_at] = _spi_device->transfer(0x00);
  //}
    
  ram_module._spi_device->transfer(&buffer[start_at], size_to_read);
  
  digitalWrite(EXT_RAM_CHIP_SELECT, HIGH);     
  ram_module._spi_device->endTransaction();
  
  if ( interrupted == 0 ) { 
    //SREG = ram_module._tmpSREG;
    ram_module._spi_device->notUsingInterrupt(255);
  }  
}

static void Ram::write(uint8_t * buffer, uint16_t buffer_address, uint8_t buffer_id, uint8_t interrupted, uint8_t size_to_write, uint8_t start_at)
{
  uint32_t memory_address = ram_module._buffer_layout[buffer_id].buffer_address + ((uint32_t)buffer_address * (uint32_t)ram_module._buffer_layout[buffer_id].buffer_size) + (uint32_t)start_at;

  if ( interrupted == 0 ) { 
    ram_module._spi_device->usingInterrupt(255);
  } 

  ram_module._spi_device->beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE)); 
  digitalWrite(EXT_RAM_CHIP_SELECT, LOW);  

  // send address request
  //_spi_device->transfer(&_set_address[interrupted], 4);
  ram_module._spi_device->transfer(WRITE);
  ram_module._spi_device->transfer((uint8_t)(memory_address >> 16));
  ram_module._spi_device->transfer((uint8_t)(memory_address >> 8));
  ram_module._spi_device->transfer((uint8_t)memory_address);
  
  if ( size_to_write == 0 ) {
    size_to_write = ram_module._buffer_layout[buffer_id].buffer_size;
  }
  
  // send data to be writed
  for (ram_module._counter = 0; ram_module._counter < size_to_write; ram_module._counter++) {
    ram_module._spi_device->transfer(buffer[ram_module._counter+start_at]);
  }
  
  //_spi_device->transfer(&buffer[start_at], size_to_write);
  
  digitalWrite(EXT_RAM_CHIP_SELECT, HIGH);  
  ram_module._spi_device->endTransaction();
  
  if ( interrupted == 0 ) { 
    ram_module._spi_device->notUsingInterrupt(255);
  }  
}

static void Ram::fill(uint8_t fill, uint16_t buffer_address, uint8_t buffer_id, uint8_t interrupted, uint16_t size_to_write, uint16_t start_at)
{
  uint32_t memory_address = ram_module._buffer_layout[buffer_id].buffer_address + ((uint32_t)buffer_address * (uint32_t)ram_module._buffer_layout[buffer_id].buffer_size) + (uint32_t)start_at;
  
  if ( interrupted == 0 ) { 
    ram_module._spi_device->usingInterrupt(255);
  } 

  ram_module._spi_device->beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE)); 
  digitalWrite(EXT_RAM_CHIP_SELECT, LOW);  

  // send address request
  ram_module._spi_device->transfer(WRITE);
  ram_module._spi_device->transfer((uint8_t)(memory_address >> 16));
  ram_module._spi_device->transfer((uint8_t)(memory_address >> 8));
  ram_module._spi_device->transfer((uint8_t)memory_address);

  if ( size_to_write == 0 ) {
    size_to_write = ram_module._buffer_layout[buffer_id].buffer_size;
  }
  
  // send data to be writed
  for ( uint16_t counter = 0; counter < size_to_write; counter++) {
    ram_module._spi_device->transfer(fill);
  }
  
  //_spi_device->transfer(&buffer[start_at], size_to_write);
  
  digitalWrite(EXT_RAM_CHIP_SELECT, HIGH);  
  ram_module._spi_device->endTransaction();
  
  if ( interrupted == 0 ) { 
    ram_module._spi_device->notUsingInterrupt(255);
  }  
}
  
void Ram::setMode(uint8_t mode)
{
  _spi_device->usingInterrupt(255);

  _spi_device->beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE));
  digitalWrite(EXT_RAM_CHIP_SELECT, LOW);
  
  _spi_device->transfer(RDMR);
  _spi_device->transfer(mode);
  
  digitalWrite(EXT_RAM_CHIP_SELECT, HIGH);     
  _spi_device->endTransaction();
  
  _spi_device->notUsingInterrupt(255);
} 
      
} }

uctrl::module::Ram ram_module;

#endif