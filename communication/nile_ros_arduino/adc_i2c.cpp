//Driver for AD7995 4-Channel 10 Bit ADC
//Nicodemus Phaklides
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
double ADCI2C::read(int channel)
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
    data &= 0x3;
    data = data << 8;
    data |= Wire.read();
  }

  if (channel == 1) {
    float Vs = 5*float(data)/4095;
    float Rs = (100000*Vs)/(5-Vs);
    float A = 0.001424496067374;
    float B = 1.511635517653385*pow(10,-4);
    float C = 2.445546167968906*pow(10,-7);
    double temperature = 1.0/(A + B*log(Rs) + C*pow(log(Rs),3)) - 273.15;
    return temperature;
  }
  else {
    return double(data);
  }
}
