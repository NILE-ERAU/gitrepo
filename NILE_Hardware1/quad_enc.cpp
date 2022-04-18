#include "quad_enc.h"

QuadEnc::QuadEnc(int clk, int dir) 
{
  clk_pin = clk;
  dir_pin = dir;
}
void QuadEnc::init()
{
  pinMode(clk_pin,INPUT);
  pinMode(dir_pin,INPUT);
}
int QuadEnc::count()
{
  if (digitalRead(clk_pin) == 0) {return -1 + 2*digitalRead(dir_pin);}
  else {return 0;}
}
