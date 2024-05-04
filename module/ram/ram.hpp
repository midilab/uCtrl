#ifndef __U_CTRL_RAM_HPP__
#define __U_CTRL_RAM_HPP__

#include <Arduino.h>
#include <SPI.h>

namespace uctrl { namespace module { 

// SRAM opcodes commands
#define RDMR        5 // read mode register
#define WRMR        1 // write mode register
#define READ        3
#define WRITE       2

// SRAM Hold line disabled when HOLD == 1
#define HOLD 1

// SRAM modes
#define BYTE_MODE (0x00 | HOLD)
#define PAGE_MODE (0x80 | HOLD)
#define STREAM_MODE (0x40 | HOLD)

// this chip operates at 20MHZ max speed
// SPI_CLOCK_DIV8	on a 16Mhz avr = 2 Mhz max
// on teensy lc runs fine at 16mhz
#define SPI_SPEED       16000000
//#define SPI_SPEED       2000000
#define SPI_MODE        SPI_MODE0

typedef struct
{
	uint8_t byte1;
	uint8_t byte2;
	uint8_t byte3;
	uint8_t byte4;  
} SET_ADDRESS_BUFFER;

typedef struct
{
	uint16_t buffer_length;
	uint16_t buffer_size;
	uint32_t buffer_address;
} BUFFER_LAYOUT_RAM;
		
class Ram
{
    public:
        Ram();
        ~Ram(); 
			
		void init(SPIClass * device, uint8_t chip_select = 0, bool is_shared = false);
		static int8_t registerBuffer( uint16_t buffer_size, uint16_t buffer_length );	
		void setMode(uint8_t mode);
		static void read(uint8_t * buffer, uint16_t buffer_address, uint8_t buffer_id, uint8_t interrupted = 0, uint8_t size_to_read = 0, uint8_t start_at = 0);
		static void write(uint8_t * buffer, uint16_t buffer_address, uint8_t buffer_id, uint8_t interrupted = 0, uint8_t size_to_write = 0, uint8_t start_at = 0);
		static void fill(uint8_t fill, uint16_t buffer_address, uint8_t buffer_id, uint8_t interrupted = 0, uint16_t size_to_write = 0, uint16_t start_at = 0);
		uint32_t getFreeRam();
			
    protected:
		SPIClass * _spi_device = nullptr;
		uint8_t _chip_select = 0;
		uint32_t _buffer_address_pointer = 0;
		uint8_t _buffer_address_id_pointer = 0;
		bool _is_shared = false;
		volatile BUFFER_LAYOUT_RAM _buffer_layout[4];
};

} }

extern uctrl::module::Ram ram_module;

#endif
