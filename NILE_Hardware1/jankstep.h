#ifndef jankstep
#define jankstep

#include <Arduino.h>

class JankStepper
{
  int clk_pin, dir_pin;
  volatile int steps;
  volatile unsigned int timer1_counter;
  volatile bool done;
  public:
    
	  JankStepper(int clk, int dir);
    void init();
    void stepMotor(long step_count);
    void motorRPM(long rpm);
    void startTimer1(void);
    void stopTimer1(void);
	  void isrFcn();
	  void moveStepper(long step_count, long rpm);
};

#endif
