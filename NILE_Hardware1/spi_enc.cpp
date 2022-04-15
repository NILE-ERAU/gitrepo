//SPI_enc.cpp - Library for spi encoder
//Created by Tia, 3/11/22

#include "spi_enc.h"
#include <SPI.h>


//ENCODER SPI Commands
#define AMT_NOP       0x00
#define AMT_READ      0x10
#define AMT_ZERO      0x70

//MEGA SPI
#define SPI_MISO 50
#define SPI_MOSI 51
#define SPI_SCK 52
#define SPI_CSB 48

void SPI_enc::init()
{
  //configure SPI
  pinMode(SPI_SCK, OUTPUT);
  pinMode(SPI_MOSI, OUTPUT);
  pinMode(SPI_MISO, INPUT);
  pinMode(SPI_CSB, OUTPUT);
  
  digitalWrite(SPI_CSB, HIGH);

  SPI.setClockDivider(SPI_CLOCK_DIV32);
  SPI.begin(); 
}

uint8_t SPI_enc::spiWriteRead(uint8_t sendByte, uint8_t releaseLine)
{
  //holder for the received over SPI
  uint8_t data;

  //set cs low, cs may already be low but there's no issue calling it again except for extra time
  digitalWrite(SPI_CSB, LOW);

  //There is a minimum time requirement after CS goes low before data can be clocked out of the encoder.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3);

  //send the command  
  data = SPI.transfer(sendByte);
  delayMicroseconds(3); //There is also a minimum time after clocking that CS should remain asserted before we release it
  digitalWrite(SPI_CSB, releaseLine); //if releaseLine is high set it high else it stays low
  delayMicroseconds(3);
  return data;
}

uint16_t SPI_enc::getPos() 
{
  uint16_t pos = 0;
  uint8_t data;
  uint8_t data2;
  uint8_t data1 = spiWriteRead(AMT_READ,LOW);
  
  data = spiWriteRead(AMT_NOP,HIGH); 
  data = spiWriteRead(AMT_READ,HIGH); 
  data2 = spiWriteRead(AMT_READ,HIGH);
  
  //Serial.println(' ');
  pos = (data2 << 8) | (data1) ;
  return pos;
  //Serial.println(pos);
}

void SPI_enc::setZeroSPI() 
{
  spiWriteRead(AMT_NOP, LOW);

  //this is the time required between bytes as specified in the datasheet.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3); 
  
  spiWriteRead(AMT_ZERO, HIGH);
  delay(250); //250 second delay to allow the encoder to reset
}
