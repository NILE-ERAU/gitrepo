//SPI_enc.h - Library for spi encoder
//Created by Tia, 3/11/22

#ifndef spi_enc
#define spi_enc

#include <Arduino.h>

class SPI_enc 
{
  public:
    void init();
    uint8_t spiWriteRead(uint8_t sendByte, uint8_t releaseLine);
    uint16_t getPos(); 
    void setZeroSPI();
};

#endif
