Tested on SdFat version 2.0.7
https://github.com/greiman/SdFat/releases/tag/2.0.7

Use formater always as possible
https://www.sdcard.org/downloads/formatter/

https://forum.arduino.cc/t/dont-format-sd-cards-with-os-utilities/222016

worst case for linux, use sd card formater...
mkfs.vfat -F 32 -s 64 -S 512 /dev/sdb1



    
#if defined(SDCARD_BITBANG_DRIVER)
        //SPI_DRIVER_SELECT == 2  // Must be set in SdFat/SdFatConfig.h
        //SoftSpiDriver<miso, mosi, clock>
        SoftSpiDriver<SDCARD_SOFT_SPI_MISO, SDCARD_SOFT_SPI_MOSI, SDCARD_SOFT_SPI_CLK> _soft_spi;	
#endif
        //SPI_DRIVER_SELECT == 3  // Must be set in SdFat/SdFatConfig.h for SPI device refrence usage