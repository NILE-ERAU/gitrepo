//Driver for AD7995 4-Channel 10 Bit ADC
//Nicodemus Phaklides
#ifndef adc_i2c
#define adc_i2c

#include <Arduino.h>

class ADCI2C
{
public:
  ADCI2C(int address);
  void init();
  double read(int channel);
private:
  int adr;
};

#endif
