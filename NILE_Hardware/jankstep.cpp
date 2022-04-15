//cpp file for interrupt driven stepper
#include "jankstep.h"
#include "Arduino.h"

// Microsteps per step   :     1      8     16   (Set with A4988 mode pins)
// RPM at 5kHz interrupt :   1500    187    93   (for a motor with 200 steps per rev)
// Approx. minimum RPM   :     74     10     5   (for a motor with 200 steps per rev)

// Use timer prescaler to allow lower speeds, e.g. divide clock by 8
// Use fewer microsteps per step to raise speeds

// Motor settings

#define MICROSTEPS_PER_STEP  40  // A4988 can be set to 1, 8 or 16
#define STEPS_PER_REV       400  // Test motor has 200 steps per rev
#define MAX_RPM             400  // RPM limit
#define AUTO_RAMP_DOWN           // Auto ramp down of speed at end of a movement

// Timer settings
#define CPU_MHZ        16000000L // 16 MHz oscillator
#define PRESCALER           1    // Timer 1 prescaler (no prescaler = 1, 8 for slower speeds)
#define TIMER_1_MIN         1000 // Stop speed for end of movement ramp down
#define TIMER_1_MAX         0xFFFF - ( CPU_MHZ / PRESCALER / (STEPS_PER_REV * MICROSTEPS_PER_STEP * MAX_RPM / 60L) )

JankStepper::JankStepper(int clk, int dir)
{
  clk_pin = clk;
  dir_pin = dir;
}
void JankStepper::init()
{
  pinMode(clk_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);
  Serial.begin(9600);

  startTimer1();                // Start the timer 1 interrupt
}

void JankStepper::stepMotor(long step_count)
{
  noInterrupts();
  steps = step_count;
  interrupts(); 
  if ( step_count < 0 )
  {
    digitalWrite(dir_pin, LOW);
    step_count = -step_count;
    
  }
  else digitalWrite(dir_pin, HIGH);

  Serial.println("stepMotor() run");

  // Updating steps variable is non-atomic so turn off interrupts before changing it
}

void JankStepper::motorRPM(long rpm)
{
  if ( rpm > MAX_RPM) rpm = MAX_RPM; // Interrup rate limit may lower this again

  Serial.print("Set rpm = "); Serial.print(rpm);

  long tc = 0xFFFF - ( CPU_MHZ / PRESCALER / (STEPS_PER_REV * MICROSTEPS_PER_STEP * rpm / 60L) );

  if (tc < 1) tc = 1;                           // This is the minimum speed value for the counter

  // To keep the interrupt handler burden acceptable a rate of 5000 Hz maximum is set (this also limits the max RPM)
  if ( (PRESCALER ==  1) && tc > 62336) tc = 62336;
  if ( (PRESCALER ==  8) && tc > 65136) tc = 65136;
  if ( (PRESCALER == 64) && tc > 65486) tc = 65486;

  Serial.print(", tc = "); Serial.println(tc);

  // Updating counter variable is non-atomic so turn off interrupts before changing it
  noInterrupts();
  timer1_counter = tc;
  interrupts();
}

void JankStepper::startTimer1(void)
{
  // Initialise timer 1
  noInterrupts();                      // disable interrupts
  TCCR1A = 0;                          // stop timer
  TCCR1B = 0;

  timer1_counter = TIMER_1_MIN;        // preload timer 65536-16MHz/1/frequency

  TCNT1 = timer1_counter;              // preload timer

  if (PRESCALER == 1) TCCR1B |= (1 << CS10);  // No prescaler
  if (PRESCALER == 8) TCCR1B |= (1 << CS11);  // divide clock by 8

  // Other unused prescaler divisions available
  //if (PRESCALER == 64) TCCR1B |= (1 << CS10) | (1 << CS11);   // divide clock by 64
  //if (PRESCALER == 256) TCCR1B |= (1 << CS12);                // divide clock by 256
  //if (PRESCALER == 1024) TCCR1B |= (1 << CS12) | (1 << CS10); // divide clock by 1024

  TIMSK1 |= (1 << TOIE1);              // enable timer overflow interrupt
  interrupts();                        // enable interrupts
}

void JankStepper::stopTimer1(void)
{
  noInterrupts();           // disable all interrupts
  TIMSK1 &= ~(1 << TOIE1);  // disable timer overflow interrupt
  interrupts();             // enable all interrupts
}

void JankStepper::isrFcn()
{ 
  TCNT1 = timer1_counter;
  
  if(steps == 0)
  {
	 done = true;
  } else {
	 digitalWrite(clk_pin, HIGH);
	 digitalWrite(clk_pin, LOW);
	 steps--;
   Serial.print("Steps: ");
   Serial.println(steps);
  }
  
}
void JankStepper::moveStepper(long step_count, long pwm)
{	
  motorRPM(pwm);
  if(done)
  {
  	done = false;
  	stepMotor(steps);
  }
}
