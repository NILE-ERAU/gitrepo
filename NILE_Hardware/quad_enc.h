#ifndef quad_enc
#define quad_enc

#include <Arduino.h>

class QuadEnc
{
public:
  QuadEnc(int clk, int dir);
  void init();
  int count();
private:
  int clk_pin;
  int dir_pin;
};

#endif
