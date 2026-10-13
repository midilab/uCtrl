#ifndef __U_CTRL_SDCARD_HPP__
#define __U_CTRL_SDCARD_HPP__

#include <Arduino.h>
#include <SPI.h>
#include "SdFat/SdFat.h"

namespace uctrl { namespace module { 

#define BUFFER_SIZE 64	

class SdCard
{
    public:
        SdCard();
        ~SdCard();  
        
        void plug();					
        void init(SPIClass * spi_device = nullptr, uint8_t chip_select = 0, bool is_shared = false);	
        bool isReady();
        // card is physically there but its filesystem did not mount
        // (unsupported or missing format)
        bool needsFormat();
        // quick format the card(fat32 on avr/esp32, fat32/exfat on teensy)
        // then remount. true on success.
        bool formatCard();
        // mode: 0 = read, 1 = write new(remove existing first), 2 = write in place(create if missing, content preserved), 3 = read/write in place
        // converted to oflag_t(int on ARM SdFat, O_CREAT=0x200) inside openFile
        bool openFile(const char * path, uint8_t mode, uint8_t interrupted = 0);
        bool readTextLine(char * line, char * str, size_t size, uint8_t field_num, char field_delim);
        bool closeFile(uint8_t interrupted = 0);
        bool seek(uint32_t position, uint8_t interrupted = 0);
        uint32_t position(uint8_t interrupted = 0);
        uint32_t size(uint8_t interrupted = 0);
        uint8_t * getData(size_t buffer_size, uint8_t interrupted = 0);
        bool setData(uint8_t * buffer, size_t buffer_size, uint8_t interrupted = 0);
        bool getConfig(char * string, uint8_t size, uint8_t field_num = 0, uint8_t interrupted = 0);
        bool getConfig(int16_t * number, uint8_t field_num = 0, uint8_t interrupted = 0);
        bool getConfig(float * number, uint8_t field_num = 0, uint8_t interrupted = 0);
        bool setConfig(char * string, uint8_t size, uint8_t field_num = 0, uint8_t interrupted = 0);
        bool remove(char * path, uint8_t interrupted = 0);
        bool chdir(char * path, uint8_t interrupted = 0);
        bool exists(const char * path, uint8_t interrupted = 0);
        bool mkdir(const char * path, uint8_t interrupted = 0);

    protected:
    
#if defined(SDCARD_BITBANG_DRIVER)
        //SPI_DRIVER_SELECT == 2  // Must be set in SdFat/SdFatConfig.h
        //SoftSpiDriver<miso, mosi, clock>
        //volatile SoftSpiDriver<SDCARD_SOFT_SPI_MISO, SDCARD_SOFT_SPI_MOSI, SDCARD_SOFT_SPI_CLK> _soft_spi;	
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
        //volatile File32 _file;

        // exFAT
        //SdExFat sd;
        //ExFile file;

        // FAT16/FAT32 and exFAT
        //SdFs sd;
        //FsFile file;    

        // for getData place his binary object data on it
        uint8_t _cache[BUFFER_SIZE];

        uint16_t _buffer_count; 
        bool _is_shared = false;
        bool _ready = false;
        bool _card_present = false;
};

} }

//extern uctrl::module::SdCard sdcard_module;

#endif
