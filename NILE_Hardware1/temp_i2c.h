#ifndef temp_i2c
#define temp_i2c

#include <Arduino.h>

class TempI2C
{
public:
  TempI2C(int address);
  void init();
  double temp();
private:
  int adr;
};

#endif
