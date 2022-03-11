#include "adc_i2c.h"
#include <Wire.h>

ADCI2C::ADCI2C(int address) 
{
  adr = address;
}
void ADCI2C::init()
{
  Wire.begin();
}
int ADCI2C::read(int channel)
{
  byte reg = 0;
  switch(channel) {
  case 0: reg = 0x10; break; //read from Vin0 MOISTURE
  case 1: reg = 0x20; break; //read from Vin1 EE TEMP
  case 2: reg = 0x40; break; //read from Vin2 HVEC TEMP
  default: reg = 0; break;
  }
  Wire.beginTransmission(adr);
  Wire.write(reg); //selects configuration register
  Wire.endTransmission();
  
  delay(50);
  
  Wire.requestFrom(adr, 2);
  int data = 0;
  if (2 <= Wire.available()) {
    data = Wire.read();
    data &= 0x7;
    data = data << 8;
    data |= Wire.read();
  }
  return data;
}
