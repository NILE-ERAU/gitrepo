#include "temp_i2c.h"
#include <Wire.h>

TempI2C::TempI2C(int address) 
{
  adr = address;
}
void TempI2C::init()
{
  Wire.begin();
  Wire.beginTransmission(adr);
  Wire.write(0x01); //selects configuration register
  Wire.write(0x60); //sets to max accuracy
  Wire.endTransmission();
  delay(70);
  Wire.beginTransmission(adr);
  Wire.write(0x00);
  Wire.endTransmission();
}
double TempI2C::temp()
{
  Wire.requestFrom(adr, 2);
  int data = 0;
  if (2 <= Wire.available()) {
    data = Wire.read();
    data = data << 8;
    data |= Wire.read();
  }
  data = data >> 4;
  return double(data * 0.0625);
}
