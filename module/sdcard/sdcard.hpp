#ifndef __U_CTRL_SDCARD_HPP__
#define __U_CTRL_SDCARD_HPP__

#include <Arduino.h>
#include <SPI.h>
#include "SdFat/SdFat.h"

#include "../../../../modules.h"

namespace uctrl { namespace module { 

#define BUFFER_SIZE 64	

class SdCard
{
    public:
        SdCard();
        ~SdCard();  
        
        void plug();					
        void init(SPIClass * spi_device = nullptr);	
        bool openFile(const char * path, uint8_t oflags);
        bool readTextLine(char * line, char * str, size_t size, uint8_t field_num, char field_delim);
        bool closeFile();
        uint8_t * getData(size_t buffer_size);
        bool setData(uint8_t * buffer, size_t buffer_size);
        bool getConfig(char * string, uint8_t size, uint8_t field_num = 0);
        bool getConfig(int16_t * number, uint8_t field_num = 0);
        bool getConfig(float * number, uint8_t field_num = 0);
        bool setConfig(char * string, uint8_t size, uint8_t field_num = 0);
        bool remove(char * path);
        bool chdir(char * path);
        bool exists(char * path);

    protected:
    
#if defined(SDCARD_BITBANG_DRIVER)
        //SPI_DRIVER_SELECT == 2  // Must be set in SdFat/SdFatConfig.h
        //SoftSpiDriver<miso, mosi, clock>
        SoftSpiDriver<SDCARD_SOFT_SPI_MISO, SDCARD_SOFT_SPI_MOSI, SDCARD_SOFT_SPI_CLK> _soft_spi;	
#endif
        //SPI_DRIVER_SELECT == 3  // Must be set in SdFat/SdFatConfig.h for SPI device refrence usage
        SdFat _sd_fat;    

#if defined(TEENSYDUINO)
        FsFile _file; // for teensyduino
#else
        File _file; // for avr arduinos
#endif
        // FAT16/FAT32
        //SdFat32 sd;
        //File32 _file;

        // exFAT
        //SdExFat sd;
        //ExFile file;

        // FAT16/FAT32 and exFAT
        //SdFs sd;
        //FsFile file;    

        // for getData place his binary object data on it
        uint8_t _cache[BUFFER_SIZE];

        uint16_t _buffer_count; 
};

} }

extern uctrl::module::SdCard sdcard_module;

#endif
