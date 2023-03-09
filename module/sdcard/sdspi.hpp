#ifndef __U_CTRL_SDCARD_SDSPI_HPP__
#define __U_CTRL_SDCARD_SDSPI_HPP__

#include "SdFat/SdFat.h"
#include <SPI.h>

class SdSpiClass : public SdSpiBaseClass {
 public:
  
  void setDevice(SPIClass * spiDevice) {
      m_spiDevice = spiDevice;
  }
  // Activate SPI hardware with correct speed and mode.
  void activate() {
    m_spiDevice->beginTransaction(m_spiSettings);
  }
  // Initialize the SPI bus.
  void begin(SdSpiConfig config) {
    (void)config;
    m_spiDevice->begin();
  }
  // Deactivate SPI hardware.
  void deactivate() {
    m_spiDevice->endTransaction();
  }
  // Receive a byte.
  uint8_t receive() {
    return m_spiDevice->transfer(0XFF);
  }
  // Receive multiple bytes.
  // Replace this function if your board has multiple byte receive.
  uint8_t receive(uint8_t* buf, size_t count) {
    for (size_t i = 0; i < count; i++) {
      buf[i] = m_spiDevice->transfer(0XFF);
    }
    return 0;
  }
  // Send a byte.
  void send(uint8_t data) {
    m_spiDevice->transfer(data);
  }
  // Send multiple bytes.
  // Replace this function if your board has multiple byte send.
  void send(const uint8_t* buf, size_t count) {
    for (size_t i = 0; i < count; i++) {
      m_spiDevice->transfer(buf[i]);
    }
  }
  // Save SPISettings for new max SCK frequency
  void setSckSpeed(uint32_t maxSck) {
    m_spiSettings = SPISettings(maxSck, MSBFIRST, SPI_MODE0);
  }

 private:
  SPISettings m_spiSettings;
  SPIClass * m_spiDevice = nullptr;
} sdSpi;

#endif
