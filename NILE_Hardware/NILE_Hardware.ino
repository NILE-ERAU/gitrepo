//
//            /\
//           /  \
//          /    \
//         /     /\
//        /     /  \          __     _    _    _         ______
//       /     /    \        |   \  | |  | |  | |       |  ____|
//      /     /     /\       | |\ \ | |  | |  | |       | |____
//     /     /     /  \      | | \ \| |  | |  | |       |  ____|
//    /     /     /    \     | |  \   |  | |  | |____   | |____
//   /_____/_____/______\    |_|   \__|  |_|  |______|  |______|
//
//Control Code for the NILE Robotic System
//Spring 2022 - Robotics Capstone

//Nicodemus Phaklides - NILE_Hardware.ino (3/14), adc_i2c.h, quad_enc.h, temp_i2c.h
//Tia McKenzie - spi_enc.h

//-----------------------------------------------------------------------------------------------------------------------
//Required Libraries
#include "temp_i2c.h"
#include "adc_i2c.h"
#include "quad_enc.h"
#include "spi_enc.h"
#include <Wire.h>
 

//-----------------------------------------------------------------------------------------------------------------------
//Pin Assignments

//Digital Outputs
#define P_STATUS_LED 2 //Board status LED
#define P_HVEC 25 //DO NOT TURN ON UNLESS U KNOW WHAT U ARE DOING!!
#define P_WATER_SOLE 27 //Water solenoid
#define P_FERT_SOLE 26 //Fertilizer solenoid

//Motors
#define P_TROLLEY_PWM_F 5 //Trolley motor "Forward" PWM signal
#define P_TROLLEY_PWM_B 6 //Trolley motor "Backward" PWM signal
#define P_VERT_STEP 4 //Vertical stepper step pulse (PWM-enabled)
#define P_VERT_DIR 24 //Vertical stepper step direction
#define P_WHEEL_PWM_F 8 //Wheel motor "Forward" PWM signal
#define P_WHEEL_PWM_B 7 //Wheel motor "Backward" PWM signal

//Analog Sensors
#define P_TROLLEY_SW 29 //Trolley callibration limit switch
#define P_VERT_SW 28 //Vertical stepper callibration limit switch
#define P_TROLLEY_ENC_COUNT 10 //Trolley encoder count signal (from decoder, handled by quad_enc.h)
#define P_TROLLEY_ENC_DIR 23 //Trolley encoder direction signal (from decoder, handled by quad_enc.h)
#define P_TROLLEY_FLOW 11 //Trolley flowmeter, pulses
#define P_FLOW A9 //Fluid box flowmeter, pulses (I had to put this on an ADC pin because i thought i had ran out of interrupt pins)

//-----------------------------------------------------------------------------------------------------------------------
//Global Variables
unsigned long t = 0; //local time variable, hopefully won't overflow
unsigned long prev_t = 0; //previous time, used for derivative controller

//Joint Variables
double theta = 0;
double d = 0;
double v = 0;

//Raw Joint Variables
int d_count = 0; //count variable for trolley position
int v_count = 0; //vertical stepper count
int theta_count = 0; //rotation encoder count

//System Modes
bool roboControl_ = false;
bool roboHome_ = false;
bool HVEC_ = false;
bool waterPlants_ = false;



//Digital Sensors
ADCI2C EE_ADC(0x28); //End-Effector ADC
QuadEnc trolley_enc(P_TROLLEY_ENC_COUNT,P_TROLLEY_ENC_DIR); //Trolley Encoder
SPI_enc rotary_enc; //Rotary absolute encoder

//-----------------------------------------------------------------------------------------------------------------------
//Setup Function
void setup() {
  // Sets interrupt for PCINT0
  cli();
  PCICR |= 0b00000111;    // turn on all ports
  PCMSK0 |= 0b00010000; // PCINT0
  sei();

  //Serial Monitor for debugging (Serial.prinln())
  Serial.begin(9600);
  Serial.println("Starting...");

  
  //Digital Outputs
  pinMode(P_STATUS_LED, OUTPUT); //Board status LED
  pinMode(P_HVEC, OUTPUT); //DO NOT TURN ON UNLESS U KNOW WHAT U ARE DOING!!
  digitalWrite(P_HVEC, 0); //PLEASE FOR REAL
  pinMode(P_WATER_SOLE, OUTPUT); //Water solenoid
  pinMode(P_FERT_SOLE, OUTPUT);//Fertilizer solenoid
  
  //Motors
  pinMode(P_TROLLEY_PWM_F, OUTPUT); //Trolley motor "Forward" PWM signal
  pinMode(P_TROLLEY_PWM_B, OUTPUT); //Trolley motor "Backward" PWM signal
  pinMode(P_VERT_STEP, OUTPUT); //Vertical stepper step pulse (PWM-enabled)
  pinMode(P_VERT_DIR, OUTPUT); //Vertical stepper step direction
  pinMode(P_WHEEL_PWM_F, OUTPUT); //Wheel motor "Forward" PWM signal
  pinMode(P_WHEEL_PWM_B, OUTPUT); //Wheel motor "Backward" PWM signal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
  
  //Analog Sensors
  pinMode(P_TROLLEY_SW, INPUT); //Trolley callibration limit switch
  pinMode(P_VERT_SW, INPUT); //Vertical stepper callibration limit switch
  pinMode(P_TROLLEY_ENC_COUNT, INPUT); //Trolley encoder count signal (from decoder, handled by quad_enc.h)
  pinMode(P_TROLLEY_ENC_DIR, INPUT); //Trolley encoder direction signal (from decoder, handled by quad_enc.h)
  pinMode(P_TROLLEY_FLOW, INPUT); //Trolley flowmeter, pulses
  pinMode(P_FLOW, INPUT); //Fluid box flowmeter
  
  //Sensor initialization
  trolley_enc.init();
  rotary_enc.init();
  EE_ADC.init();
}
//-----------------------------------------------------------------------------------------------------------------------
// Robot Functions

int roboControl(double theta_d, double d_d, double v_d) {
  //Vairable Initialization
  static double elast_theta = 0;
  static double elast_d = 0;
  static double elast_v = 0;
  int roboControlState = 0;
 
  //Theta gains
  double Kp_theta = 20;
  double Ki_theta = 0;
  double Kd_theta = 0;
  double pwm_theta;
  double e_theta = theta_d - theta;
  double edot_theta = (e_theta - elast_theta)/(double)(t - prev_t);
  static double eint_theta = (eint_theta + elast_theta)*(double)(t - prev_t);

  //D gains
  double Kp_d = 0;
  double Ki_d = 0;
  double Kd_d = 0;
  double pwm_d;
  double e_d = d_d - d;
  double edot_d = (e_d - elast_d)/(double)(t - prev_t);
  static double eint_d = (eint_d + elast_d)*(double)(t - prev_t);
  
  //V gains
  double Kp_v = 0;
  double Ki_v = 0;
  double Kd_v = 0;
  double pwm_v;
  double e_v = v_d - v;
  double edot_v = (e_v - elast_v)/(double)(t - prev_t);
  static double eint_v = (eint_v + elast_v)*(double)(t - prev_t);

  //Control Equations
  pwm_theta = Kp_theta*e_theta + Ki_theta*eint_theta + Kd_theta*edot_d; //0-255
  pwm_d = Kp_d*e_d + Ki_d*eint_d + Kd_d*edot_d; //0-255
  pwm_v = Kp_v*e_v + Ki_v*eint_v + Kd_v*edot_v;
  
  if(e_theta < 0.001 && e_d < 0.001 && e_v < 0.001){
    roboControlState = 1;
    eint_theta = 0;
    eint_d = 0;
    eint_v = 0;
  } else {
    roboControlState = 0;
    driveRotation(pwm_theta);
    driveTrolley(pwm_d);
    driveStepper(pwm_v);
    elast_theta = e_theta;
    elast_d = e_d;
    elast_v = e_v;
  }

  return roboControlState;
  
}

double readRotation()
{
  uint16_t raw;
  double theta;
  raw = rotary_enc.getPos();  //receive number of counts
  theta = ((2*PI)/4095)*(double)raw;  //convert counts to radians
  return theta;
}

double readVerticalPos(){
  double countsPerRot = 400;
  double dispPerRot = 0.009525;
  return (v_count/countsPerRot)*dispPerRot;
}

double readTrolleyPos() {
  double countsPerRot = 1024;
  double wheel_r = 0.02905;
  return (d_count/countsPerRot)*(2*wheel_r*PI);
}

int readSoilMoist()
{
  return EE_ADC.read(0);
}

double readSoilTemp()
{
  return EE_ADC.read(1);
}

double readHVECTemp()
{
  return EE_ADC.read(2);
}

int driveRotation(double speed)
{
  if (speed>0)
  {
    analogWrite(P_WHEEL_PWM_F,(int)abs(speed));
  }
  else
  {
    analogWrite(P_WHEEL_PWM_B,(int)abs(speed));
  }
  return 1;
}

int driveTrolley(double speed)
{
  if (speed>0)
  {
    analogWrite(P_TROLLEY_PWM_F,(int)abs(speed));
  }
  else
  {
    analogWrite(P_TROLLEY_PWM_B,(int)abs(speed));
  }
  return 1;
}

//-----------------------------------------------------------------------------------------------------------------------
//Interrupt functions (Only PCINT0 enabled as of 3/14)

ISR (PCINT0_vect) // handle pin change interrupt for D8 to D13 here
 {    
  d_count += trolley_enc.count(); //handles the trolley counting encoder
 }
 
ISR (PCINT1_vect) // handle pin change interrupt for A0 to A5 here
 {

 }  
 
ISR (PCINT2_vect) // handle pin change interrupt for D0 to D7 here
 {
    
 }  

//-----------------------------------------------------------------------------------------------------------------------
//Main Loop
 

void loop() {
  t = millis();

  //ROS Fetching

  //Update Sensors
  theta = readRotation();
  d = readTrolleyPos();
  v = readVerticalPos();

  //System Modes


  prev_t = t;
}
