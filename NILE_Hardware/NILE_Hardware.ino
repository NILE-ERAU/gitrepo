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
#include <AccelStepper.h>

// Import libraries for ROS nodes
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

//-----------------------------------------------------------------------------------------------------------------------
//Pin Assignments

//Digital Outputs
#define P_STATUS_LED 2 //Board status LED
#define P_HVEC 25 //DO NOT TURN ON UNLESS U KNOW WHAT U ARE DOING!!
#define P_WATER_SOLE 27 //Water solenoid
#define P_FERT_SOLE 26 //Fertilizer solenoid
#define P_BUZZER 12

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
//Stepper Stuff
#define motorInterfaceType 1
AccelStepper vert(motorInterfaceType, P_VERT_STEP, P_VERT_DIR);
//-----------------------------------------------------------------------------------------------------------------------
//Global Variables
unsigned long t = 0; //local time variable, hopefully won't overflow
unsigned long prev_t = 0; //previous time, used for derivative controller
double prevSoilTemp = -273; //used to smooth out temp measurements, dont change init

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
// ROS-actuated functions

// Define function for robot homing node
// NOTE: NEED TO REWRITE HOMING INTERFACE FUNCTION
void homing(const std_msgs::String& cmd_msg) {
  // Initialize homing state variables
  static bool hometrolley = false;
  static bool homestepper = false;
  static bool homerot = false;

   // Store input string
  String input = cmd_msg.data;

  // Flag trolley homing variable
  if(input.equalsIgnoreCase("trolley") == true) {
    hometrolley = true;
  }

  else if(input.equalsIgnoreCase("stepper") == true) {
    homestepper = true;
  }

  else if(input.equalsIgnoreCase("rot") == true) {
    homerot = true;
  }
}

// Define function for receiving and processing target task-space coordinates
void movement(const std_msgs::String& cmd_msg) {
  // Receive string input and convert to char array
  String input = cmd_msg.data;
  char coords[input.length()+1];
  input.toCharArray(coords, input.length()+1);

  // Call robot control function to execute movement
  roboControl(coords);
}

//---------------------------------------------------------------------------------------------------
// Define ROS subscribers
// Node home_sub subscribes to topic "home" and references "homing" function
ros::Subscriber<std_msgs::String> home_sub("home", &homing);
// Node move_sub subscribes to topic "coordinates" and references "movement" function
ros::Subscriber<std_msgs::String> move_sub("coordinates", &movement);

// Node

//---------------------------------------------------------------------------------------------------
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
  vert.setMaxSpeed(2000);
  vert.setAcceleration(500);

  //Test Variables
  roboHome_ = true;
  roboControl_ = true;

  // Initialize ROS node and define topic subscribers
  nh.initNode();
  nh.subscribe(home_sub);
  nh.subscribe(move_sub);
}

//-----------------------------------------------------------------------------------------------------------------------
// Robot Functions

//int roboControl(double theta_d, double d_d, double v_d) {
int roboControl(char* coords) {

  // Parse the coordinates character-array into sections and store relevant variables
  // then convert char entries to doubles using atof() function
  double theta_d = atof(strtok(coords, " :,"));
  double d_d = atof(strtok(NULL, " :,"));
  double v_d = atof(strtok(NULL, " :,"));

  //Vairable Initialization
  static double elast_theta = 0, elast_d = 0;
  static double eint_theta = 0, eint_d = 0;
  int roboControlState = 0;
  static int controlMode = 1;
  double dt = (double)(t - prev_t)*0.001; //Seconds

  //Theta
  double Kp_theta = 300, Ki_theta = 10, Kd_theta = 10;
  double pwm_theta;
  double e_theta = (theta_d - theta);
  double edot_theta = (e_theta - elast_theta)/(double)(dt);
  eint_theta = eint_theta + elast_theta*(double)(dt);

  //D
  double Kp_d = 250, Ki_d = 10, Kd_d = 5;
  double pwm_d = 0;
  double e_d = d_d - d;
  double edot_d = (e_d - elast_d)/(double)(dt);
  eint_d = eint_d + elast_d*(double)(dt);

  //V
  double steps_v = 0;

  //Trolley Control
  if(controlMode == 1){
    if(abs(e_theta) < 0.01){
      //Set PWM_D Value
      pwm_theta = 0;
      driveRotation(pwm_theta);
      //Reset values
      eint_theta = 0;
      elast_theta = 0;
      controlMode++;
    } else {
      //Set PWM_D value
      pwm_theta = -1*(Kp_theta*e_theta + Ki_theta*eint_theta + Kd_theta*edot_theta);
      pwm_theta = (pwm_theta > 200) ? 200 : pwm_theta; //0-100
      pwm_theta = (pwm_theta < -200) ? -200 : pwm_theta; //0-100
      //Set PWM_D values
      elast_theta = e_theta;
      driveRotation(pwm_theta);
    }
  } else if(controlMode == 2){
    //Rotation Control
    if(abs(e_d) < 0.001){
      //Set PWM_D Value
      pwm_d = 0;
      driveTrolley(pwm_d);
      //Reset values
      eint_d = 0;
      elast_d = 0;
      controlMode++;
    } else {
      //Set PWM_D value
      pwm_d = Kp_d*e_d + Ki_d*eint_d + Kd_d*edot_d;
      pwm_d = (pwm_d > 255.0) ? 255 : pwm_d; //-255 to 255
      driveTrolley(pwm_d);
      //Set PWM_D values
      elast_d = e_d;
    }
  } else if(controlMode == 3){
      steps_v = v_d*400/0.009525;
      stepStepper(steps_v);
      controlMode++;
  } else {
      roboControlState = 1;
      controlMode = 1;
  }

  //Reference Values
  Serial.print("Rotation Position: ");
  Serial.print(theta, 4);
  Serial.print(", Rotation Error: ");
  Serial.print(e_theta);
  Serial.print(", Rotation PWM: ");
  Serial.print(pwm_theta);
  Serial.print(", Theta_d: ");
  Serial.println(theta_d);

  if(roboControlState){
    return 1;
    roboControlState = 0;
  } else {
    return roboControlState;
  }
}

int roboHome(){
  static bool homeTrolley = false;
  static bool homeStepper = true;
  static int trolleyMode = 1, stepperMode = 1, homeMode = 1;
  static int roboHome_ = 0;
  double pwm_trolley = -150;
  double pwm_theta = 50;

  //Trolley homing
  if(homeMode == 1){
    if(trolleyMode == 1){
      driveTrolley(-250);
      if(digitalRead(P_TROLLEY_SW) == 1){
        Serial.println("Case 1");
        driveTrolley(0);
        trolleyMode = 2;
      }
    } else if (trolleyMode == 2){
      Serial.println("Case 2");
      driveTrolley(250);
      if(digitalRead(P_TROLLEY_SW) == 0){
        driveTrolley(0);
        trolleyMode = 3;
      }

    }else{
      driveTrolley(-100);
      if(digitalRead(P_TROLLEY_SW)){
        Serial.println("Case 3");
        d_count = 0;
        driveTrolley(0);
        trolleyMode = 1;
        homeMode++;
      }
    }
  } else if(homeMode == 2){
    if(stepperMode == 1){
      driveStepper(1500);
      if(digitalRead(P_VERT_SW) == 1){
        Serial.println("Case 1");
        driveStepper(0);
        stepperMode = 2;
      }
    } else if(stepperMode == 2){
        Serial.println("Case 2");
        driveStepper(-1000);
        if(digitalRead(P_TROLLEY_SW) == 0){
          driveStepper(0);
          stepperMode = 3;
        }
    } else if (stepperMode == 3){
      Serial.println("Case 2");
      delay(2000);
      driveStepper(0);
      stepperMode = 4;
    }else{
      driveStepper(1000);
      if(digitalRead(P_VERT_SW)){
        Serial.println("Case 3");
        v_count = 0;
        driveStepper(0);
        delay(1000);
        stepperMode = 1;
        homeMode++;
      }
    }
  } else {
    roboHome_ = 1;
    homeMode = 1;
  }

  return roboHome_;
  nh.spinOnce();
}

double readRotation()
{
  uint16_t raw;
  double theta;
  raw = rotary_enc.getPos();  //receive number of counts
  theta = 2*PI/4095*(double)raw;  //convert counts to degrees
  /*Serial.print("Theta Value: ");
  Serial.print(theta,4);
  Serial.print(", Raw Encoder: ");
  Serial.println(raw);*/
  return theta;
}

double readVerticalPos(){
  double countsPerRot = 400;
  double dispPerRot = 0.009525;
  return (v_count/countsPerRot)*dispPerRot;
}

double readTrolleyPos() {
  double countsPerRot = 2048;
  double wheel_d = 0.02905;
  //Include the initial offset from the center post to the trolley offset 0.2275
  return (d_count/countsPerRot)*(wheel_d*PI);
}

int readSoilMoist()
{
  return EE_ADC.read(0);
}

double readSoilTemp()
{
  double soilTemp = EE_ADC.read(1);
  if (prevSoilTemp == -273) {
    prevSoilTemp = soilTemp;
  }
  double smoothedTemp = 0.02*soilTemp + 0.98*prevSoilTemp;
  prevSoilTemp = smoothedTemp;
  //Serial Plotter Stuff
  //Serial.print("Soil_Temp:");
  //Serial.print(soilTemp);
  //Serial.print(",");
  //Serial.print("Filtered_Soil_Temp:");
  //Serial.println(smoothedTemp);
  return smoothedTemp;
}

double readHVECTemp()
{
  return EE_ADC.read(2);
}

int driveRotation(double speed)
{
  //Positive speed is counterclockwise looking down
  if (speed>0)
  {
    analogWrite(P_WHEEL_PWM_F,(int)abs(speed));
  } else if(speed == 0){
    analogWrite(P_WHEEL_PWM_F, 0);
    analogWrite(P_WHEEL_PWM_B, 0);
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
  } else if (speed == 0){
    analogWrite(P_TROLLEY_PWM_F, 0);
    analogWrite(P_TROLLEY_PWM_B, 0);
  } else {
    analogWrite(P_TROLLEY_PWM_B,(int)abs(speed));
  }
  return 1;
}
//Drive stepper motor based on speed
int driveStepper(double speed_v) {
  if(speed_v == 0){
    vert.stop();
  } else {
    vert.setSpeed(speed_v);
    vert.runSpeed();
  }
  return 1;
}
int stepStepper(int steps){
  vert.moveTo(steps);
  while(vert.distanceToGo()){
    vert.run();
    v_count = v_count + steps;
  }
}
int pulseHVEC(int onTime, int offTime, int pulses) {
  //PERIOD IS DEFINED IN MILLISECONDS

  //Siren
  tone(P_BUZZER, 1000);
  delay(5000);
  noTone(P_BUZZER);
  delay(1000);

  for (int i = 0; i < pulses; i++) {
     digitalWrite(P_HVEC, HIGH);
     delay(onTime);
     digitalWrite(P_HVEC, LOW);
     delay(offTime);
  }
  //do the little tone
  digitalWrite(P_HVEC, 0);
  tone(P_BUZZER, 1046,500);
  delay(500);
  tone(P_BUZZER, 1318,500);
  delay(500);
  tone(P_BUZZER, 1568,500);
  delay(500);
  noTone(P_BUZZER);
  HVEC_ = false;
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
//Negative step moves stepper down


void loop() {
  int roboControlState = 0;
  int roboHomeState = 0;
  bool testBool = true;
  double testDouble = -1.01;
  t = millis();
  /*Serial.print("Step Size: ");
  Serial.println(t-prev_t);*/

  //Update Sensors
  theta = readRotation();
  d = -1*readTrolleyPos()+0.2275;
  v = readVerticalPos();

  roboHome_ = false;
  roboControl_ = false;

  // Publish coordinates

  // read and publish sensor readings upon ROS request


  //System Mode
  if(roboHome_){
    roboHomeState = roboHome();
    Serial.print("Trolley Position: ");
    Serial.println(readTrolleyPos());
    if(roboHomeState == 1){
      roboHome_ = false;
      roboControl_ = false;
    }
  }else if(roboControl_){
    char coords = "2.35, 0.5, 0";
    roboControlState = roboControl(coords);
    if(roboControlState == 3){
      Serial.println("Controller Stopped");
      roboControl_ = false;
    }
  }

  if (HVEC_) {
    pulseHVEC(5000,5000,1);
    HVEC_ = false;
  }

  prev_t = t;

  // Implement ROS timestamp update and a 1ms delay to keep ROS serial communication updated
  nh.spinOnce();
  delay(1);
}
