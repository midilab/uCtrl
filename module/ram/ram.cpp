#include "ram.hpp"
	
namespace uctrl { namespace module {

Ram::Ram()
{
  
}

Ram::~Ram()
{

}

void Ram::init(SPIClass * device, uint8_t chip_select, bool is_shared)
{
  ram_module._spi_device = device;   
  ram_module._chip_select = chip_select;
  ram_module._is_shared = is_shared;
  pinMode(ram_module._chip_select, OUTPUT);
  digitalWrite(ram_module._chip_select, HIGH);  
  
  ram_module._buffer_address_id_pointer = 0;
  ram_module._buffer_address_pointer = 0;
  
	// Initing a common shared SPI device for all uMODULAR modules using SPI bus
	ram_module._spi_device->begin();

  //setMode(BYTE_MODE);
  //setMode(PAGE_MODE);
  //setMode(STREAM_MODE);  
}

int8_t Ram::registerBuffer( uint16_t buffer_size, uint16_t buffer_length )
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

void Ram::read(uint8_t * buffer, uint16_t buffer_address, uint8_t buffer_id, uint8_t interrupted, uint8_t size_to_read, uint8_t start_at)
{
  uint32_t memory_address = ram_module._buffer_layout[buffer_id].buffer_address + ((uint32_t)buffer_address * (uint32_t)ram_module._buffer_layout[buffer_id].buffer_size) + (uint32_t)start_at;
  if ( interrupted == 0 && ram_module._is_shared) { 
    noInterrupts();
    //ram_module._spi_device->usingInterrupt(255);
  } 

  ram_module._spi_device->beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE));
  digitalWrite(ram_module._chip_select, LOW);
  
  // send address request
  ram_module._spi_device->transfer(READ);
  ram_module._spi_device->transfer((uint8_t)(memory_address >> 16));
  ram_module._spi_device->transfer((uint8_t)(memory_address >> 8));
  ram_module._spi_device->transfer((uint8_t)memory_address);
   
  if ( size_to_read == 0 ) {
    size_to_read = ram_module._buffer_layout[buffer_id].buffer_size;
  }
  
  // read data
  //for (uint8_t i=start_at; i < size_to_read; i++)
	//  buffer[i] = (uint8_t) ram_module._spi_device->transfer(0x00); 

  ram_module._spi_device->transfer(buffer, size_to_read);
  
  digitalWrite(ram_module._chip_select, HIGH);     
  ram_module._spi_device->endTransaction();
  
  if ( interrupted == 0 && ram_module._is_shared ) { 
    interrupts();
    //ram_module._spi_device->notUsingInterrupt(255);
  }  
}

void Ram::write(uint8_t * buffer, uint16_t buffer_address, uint8_t buffer_id, uint8_t interrupted, uint8_t size_to_write, uint8_t start_at)
{
  uint32_t memory_address = ram_module._buffer_layout[buffer_id].buffer_address + ((uint32_t)buffer_address * (uint32_t)ram_module._buffer_layout[buffer_id].buffer_size) + (uint32_t)start_at;

  if ( interrupted == 0 && ram_module._is_shared ) { 
    noInterrupts();
    //ram_module._spi_device->usingInterrupt(255);
  } 

  ram_module._spi_device->beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE)); 
  digitalWrite(ram_module._chip_select, LOW);  

  // send address request
  ram_module._spi_device->transfer(WRITE);
  ram_module._spi_device->transfer((uint8_t)(memory_address >> 16));
  ram_module._spi_device->transfer((uint8_t)(memory_address >> 8));
  ram_module._spi_device->transfer((uint8_t)memory_address);
  
  if ( size_to_write == 0 ) {
    size_to_write = ram_module._buffer_layout[buffer_id].buffer_size;
  }
  
  // send data to be writed
  for (uint8_t i=0; i < size_to_write; i++) {
    ram_module._spi_device->transfer(buffer[i+start_at]);
  }
  //uint8_t tmp[2];
  //memcpy(tmp, buffer, size_to_write);
  //ram_module._spi_device->transfer(tmp, size_to_write);
  
  digitalWrite(ram_module._chip_select, HIGH);  
  ram_module._spi_device->endTransaction();
  
  if ( interrupted == 0 && ram_module._is_shared ) { 
    interrupts();
    //ram_module._spi_device->notUsingInterrupt(255);
  }  
}

void Ram::fill(uint8_t fill, uint16_t buffer_address, uint8_t buffer_id, uint8_t interrupted, uint16_t size_to_write, uint16_t start_at)
{
  uint32_t memory_address = ram_module._buffer_layout[buffer_id].buffer_address + ((uint32_t)buffer_address * (uint32_t)ram_module._buffer_layout[buffer_id].buffer_size) + (uint32_t)start_at;
  
  if ( interrupted == 0 && ram_module._is_shared ) { 
    noInterrupts();
    //ram_module._spi_device->usingInterrupt(255);
  } 

  ram_module._spi_device->beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE)); 
  digitalWrite(ram_module._chip_select, LOW);  

  // send address request
  ram_module._spi_device->transfer(WRITE);
  ram_module._spi_device->transfer((uint8_t)(memory_address >> 16));
  ram_module._spi_device->transfer((uint8_t)(memory_address >> 8));
  ram_module._spi_device->transfer((uint8_t)memory_address);

  if ( size_to_write == 0 ) {
    size_to_write = ram_module._buffer_layout[buffer_id].buffer_size;
  }
  
  // send data to be writed
  for ( uint16_t i=0; i < size_to_write; i++) {
    ram_module._spi_device->transfer(fill);
  }
  
  digitalWrite(ram_module._chip_select, HIGH);  
  ram_module._spi_device->endTransaction();
  
  if ( interrupted == 0 && ram_module._is_shared ) { 
    interrupts();
    //ram_module._spi_device->notUsingInterrupt(255);
  }  
}
  
void Ram::setMode(uint8_t mode)
{
  noInterrupts();
  //_spi_device->usingInterrupt(255);

  ram_module._spi_device->beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE));
  digitalWrite(ram_module._chip_select, LOW);
  
  ram_module._spi_device->transfer(RDMR);
  ram_module._spi_device->transfer(mode);
  
  digitalWrite(ram_module._chip_select, HIGH);     
  ram_module._spi_device->endTransaction();
  
  interrupts();
  //_spi_device->notUsingInterrupt(255);
} 
      
} }

uctrl::module::Ram ram_module;