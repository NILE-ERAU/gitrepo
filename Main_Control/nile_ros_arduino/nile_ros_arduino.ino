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
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle nh;

//-----------------------------------------------------------------------------------------------------------------------
//Pin Assignments

//Digital Outputs
#define P_STATUS_LED 2 //Board status LED
#define P_HVEC 30 //DO NOT TURN ON UNLESS U KNOW WHAT U ARE DOING!!
#define P_WATER_SOLE 27 //Water solenoid
#define P_FERT_SOLE 26 //Fertilizer solenoid
#define P_BUZZER 12

//Motors
#define P_TROLLEY_PWM_F 5 //Trolley motor "Forward" PWM signal
#define P_TROLLEY_PWM_B 6 //Trolley motor "Backward" PWM signal
#define P_VERT_STEP 4 //Vertical stepper step pulse (PWM-enabled)
#define P_VERT_DIR 31 //Vertical stepper step direction
#define P_WHEEL_PWM_F 8 //Wheel motor "Forward" PWM signal
#define P_WHEEL_PWM_B 7 //Wheel motor "Backward" PWM signal

//Analog Sensors
#define P_TROLLEY_SW 29 //Trolley callibration limit switch
#define P_VERT_SW 28 //Vertical stepper callibration limit switch
#define P_TROLLEY_ENC_COUNT 10 //Trolley encoder count signal (from decoder, handled by quad_enc.h)
#define P_TROLLEY_ENC_DIR 23 //Trolley encoder direction signal (from decoder, handled by quad_enc.h)
#define P_TROLLEY_FLOW 11 //Trolley flowmeter, pulses
#define P_FLOW A9 //Fluid box flowmeter, pulses (I had to put this on an ADC pin because i thought i had ran out of interrupt pins)

#define COUNT_PER_LITER 932
#define MAX_V_HEIGHT 0.34 //thermometer above ground when zeroed
//-----------------------------------------------------------------------------------------------------------------------
//Stepper Stuff
#define motorInterfaceType 1
AccelStepper vert(AccelStepper::DRIVER, P_VERT_STEP, P_VERT_DIR);
//-----------------------------------------------------------------------------------------------------------------------
//Global Variables
unsigned long t = 0; //local time variable, hopefully won't overflow
unsigned long prev_t = 0; //previous time, used for derivative controller
unsigned long prev_t_1s = 0; //used for 1second coord publisher
double prevSoilTemp = -273; //used to smooth out temp measurements, dont change init
unsigned long flow_trolley = 0;
unsigned long flow_box = 0;
unsigned long flow_counts = 0;

//Joint Variables
double theta = 0;
double d = 0;
double v = 0;
//target positions
double theta_d = 0;
double d_d = 0;
double v_d = 0;

//Raw Joint Variables
int d_count = 0; //count variable for trolley position
long v_count = 0; //vertical stepper count
int theta_count = 0; //rotation encoder count
//HVEC Vars
int onTime = 0;
int offTime = 0;
int pulses = 0;
unsigned long HVEC_start_t = 0;
//System Modes
bool roboControl_ = false;
bool homeTrolley_ = false;
bool homeStepper_ = false;
bool homeRot_ = false;
bool HVEC_ = false;
bool hydrate_ = false;
bool stepperRunning_ = false;

//Digital Sensors
ADCI2C EE_ADC(0x28); //End-Effector ADC
QuadEnc trolley_enc(P_TROLLEY_ENC_COUNT,P_TROLLEY_ENC_DIR); //Trolley Encoder
SPI_enc rotary_enc; //Rotary absolute encoder

//-----------------------------------------------------------------------------------------------
// ROS-actuated functions

// Define function for robot homing node
// NOTE: NEED TO REWRITE HOMING INTERFACE FUNCTION

void waterPlants(std_msgs::Float64& cmd_msg){
  float numLiters = cmd_msg.data;
  flow_counts = (long)(numLiters * COUNT_PER_LITER);
  hydrate_ = true;

}


void homing(const std_msgs::String& cmd_msg) {
  // Initialize homing state variables
   // Store input string
  String input = cmd_msg.data;

  // Flag trolley homing variable
  if(input.equalsIgnoreCase("trolley") == true) {
    homeTrolley_ = true;
    // Call trolley homing function
  }

  else if(input.equalsIgnoreCase("stepper") == true) {
    homeStepper_ = true;
    // Call stepper homing function
  }

  else if(input.equalsIgnoreCase("rot") == true) {
    homeRot_ = true;
    // Call rotational homing function
  }
}

/* NOTE; as of now the pulseHVEC script is always running, introducing several delays*/
// Define function for receiving integer array of HVEC activation parameters
//int pulseHVEC(int onTime, int offTime, int pulses) {
int pulseHVEC(const std_msgs::Int16MultiArray& cmd_msg) {
  //PERIOD IS DEFINED IN MILLISECONDS

  //int input[] = {cmd_msg.data};
  onTime = (int)cmd_msg.data[0];
  offTime = (int)cmd_msg.data[1];
  pulses = (int)cmd_msg.data[2];

  onTime = constrain(onTime,2000,10000);
  offTime = constrain(offTime,2000,60000);
  pulses = constrain(pulses,0,100);


  HVEC_start_t = millis();
  HVEC_ = true;
}

// Define function for receiving and processing target task-space coordinates
//void movement(const std_msgs::String& cmd_msg) {
void movement(std_msgs::Float64MultiArray& cmd_msg) {
  // Store entries of data array in separate variables
  std_msgs::Float64MultiArray coord_array = cmd_msg;
  theta_d = (double) coord_array.data[0];
  d_d = (double) coord_array.data[1];
  v_d = (double)(MAX_V_HEIGHT-((double) coord_array.data[2]));

  //Safety first! No more robot pushups
  theta_d = constrain(theta_d,0,2*PI);
  d_d = constrain(d_d,0,0.85);
  v_d = constrain(v_d,0,MAX_V_HEIGHT);


  // Call robot movement control function to execute motion
  roboControl_ = true;

//  float coord_data[] = {onTime, offTime, pulses};
//  coord_array.data = coord_data;
//  coord_array.data_length = 3;
//test_pub.publish(&coord_array);
}

void sense(const std_msgs::String& cmd_msg);

//---------------------------------------------------------------------------------------------------
// Define Arduino-ROS subscribers
// Node home_sub subscribes to topic "home" and references "homing" function
ros::Subscriber<std_msgs::String> home_sub("home", &homing);
// Node move_sub subscribes to topic "coordinates" and references "movement" function
ros::Subscriber<std_msgs::Float64MultiArray> move_sub("coordinates", &movement);
// Node sensor_sub subscribes to topic "sensor" and references "sense" function
ros::Subscriber<std_msgs::String> sensor_sub("sensor", &sense);
// Node shock_sub subscribes to topic "shock" and references "pulseHVEC" function
ros::Subscriber<std_msgs::Int16MultiArray> shock_sub("shock", &pulseHVEC);
// Node water_sub subscribes to topic "water" and references "waterPlants" function
ros::Subscriber<std_msgs::Float64> water_sub("water", &waterPlants);

// Define data types used for Arduino publishing to ROS
std_msgs::Float64MultiArray encoder_array;
std_msgs::UInt16 soil_moisture, complete;
std_msgs::Float64 soil_temp, hvec_temp;

// Define Arduino-ROS publishers
//Node encoder_pub publishes to topic "encoder" and posts encoder_array data
ros::Publisher encoder_pub("encoder", &encoder_array);
//Node moisture_pub publishes to topic "moist" and posts soil_moisture data
ros::Publisher moisture_pub("moist", &soil_moisture);
//Node temp_pub publishes to topic "temp" and posts soil_temp data
ros::Publisher temp_pub("temp", &soil_temp);
// Node hvec_pub publishes to topic "hvec" and posts hvec_temp data
//ros::Publisher hvec_pub("hvec", &hvec_temp);
// Node complete_pub publishes to topic "complete" and posts if command has been completed
ros::Publisher complete_pub("complete", &complete);

void readSoilMoist()
{
  // Read the moisture sensor value
  int soil_reading = EE_ADC.read(0);
  soil_moisture.data = soil_reading;
  // Set to 50 for debugging purposes only
  //soil_moisture.data = 50;
  // Publish the reading to ROS topic "moist"
  moisture_pub.publish(&soil_moisture);
}

// Define function for reading soil temperature sensor
void readSoilTemp()
{
  // Read the ADC for the soil temperature and smooth the value
  double soilTemp = EE_ADC.read(1);
  if (prevSoilTemp == -273) {
    prevSoilTemp = soilTemp;
  }
  double smoothedTemp = 0.02*soilTemp + 0.98*prevSoilTemp;
  prevSoilTemp = smoothedTemp;

  // Store the data in ROS-compatible variable
   soil_temp.data = (float)(smoothedTemp);
   // Set to 40 for debugging purposes only
   // Publish the reading to ROS topic "temp"
   temp_pub.publish(&soil_temp);
}
void sense(const std_msgs::String& cmd_msg) {
  // Receive string input and set appropriate flag
  // Call function for reading soil moisture and temperature sensors
  readSoilMoist();
  readSoilTemp();
  complete.data = 1;
  complete_pub.publish(&complete);

}

void setup(){
    // Initialize ROS node and define Arduino ROS topic subscribers
  nh.initNode();
  nh.subscribe(home_sub);
  nh.subscribe(move_sub);
  nh.subscribe(sensor_sub);
  nh.subscribe(shock_sub);
  nh.subscribe(water_sub);

  // Initialize onboard LED
  pinMode(LED_BUILTIN, OUTPUT);

  //Define Arduino ROS publishers
  nh.advertise(encoder_pub);
  nh.advertise(moisture_pub);
  nh.advertise(temp_pub);
  nh.advertise(complete_pub);
  //nh.advertise(test_pub);

// Sets interrupt for PCINT0
  cli();
  PCICR |= 0b00000111;    // turn on all ports
  PCMSK0 |= 0b00010000; // PCINT0
  PCMSK1 |= 0b00001000; // PCINT1
  PCMSK2 |= 0b00000011; // PCINT2
  sei();

  //Serial Monitor for debugging (//Serial.prinln())
  // Serial.begin(57600);
  //Serial.println("Starting...");

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
  vert.setPinsInverted(true, false, false);

  //Test Variables

}

//-----------------------------------------------------------------------------------------------
// Robot control functions
// Define function for receiving sensor selector string and reading appropriate sensor


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
    vert.setSpeed((int)speed_v);
    vert.runSpeed();
  }
  return 1;
}

int stepStepper(int steps){
  vert.moveTo(steps);
  vert.run();
  while(vert.distanceToGo() && !((vert.speed() < 0) && digitalRead(P_VERT_SW))){
    vert.run();
  }
  vert.stop();
}



int robotControl() {
//int roboControl(char* coords) {

//  // Parse the coordinates character-array into sections and store relevant variables
//  // then convert char entries to doubles using atof() function
//  double theta_d = atof(strtok(coords, " :,"));
//  double d_d = atof(strtok(NULL, " :,"));
//  double v_d = atof(strtok(NULL, " :,"));

  //Vairable Initialization
  static double elast_theta = 0, elast_d = 0;
  static double eint_theta = 0, eint_d = 0;
  int roboControlState = 0;
  static int controlMode = 1;
  double dt = (double)(t - prev_t)*0.001; //Seconds

  //Theta
  double Kp_theta = 250, Ki_theta = 10, Kd_theta = 10;
  double pwm_theta;
  double e_theta = (theta_d - theta);
  double edot_theta = (e_theta - elast_theta)/(double)(dt);
  eint_theta = eint_theta + elast_theta*(double)(dt);

  //D
  double Kp_d = 250, Ki_d = 10, Kd_d = 5;
  static double pwm_d = 0;
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
      int pwm_sign = 0;
      if (pwm_theta >= 0) {
        pwm_sign = 1;
      }
      else {
        pwm_sign = -1;
      }
      pwm_theta = constrain(abs(pwm_theta), 75, 250);
      pwm_theta = pwm_theta*pwm_sign;

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
      controlMode=3;
    } else {
      //Set PWM_D value
      pwm_d = Kp_d*e_d + Ki_d*eint_d + Kd_d*edot_d;
      pwm_d = (pwm_d > 255.0) ? 255 : pwm_d; //-255 to 255
      driveTrolley(pwm_d);
      //Set PWM_D values
      elast_d = e_d;
    }
  } else if(controlMode == 3){
      stepperRunning_ = true;
      steps_v = (v_d*400/0.009525);
      stepStepper((int)steps_v);
      controlMode = 4;
  } else {
      stepperRunning_ = false;
      roboControlState = 1;
      controlMode = 1;
  }

  return roboControlState;
}

int trolleyMode = 1;

int homeTrolley() {
  if(trolleyMode == 1){
      driveTrolley(-250);
      if(digitalRead(P_TROLLEY_SW) == 1){
        //Serial.println("Case 1");
        driveTrolley(0);
        trolleyMode = 2;
      }

     // Trolley homing, checking for switch to be released
    } else if (trolleyMode == 2){
      //Serial.println("Case 2");
      driveTrolley(250);
      if(digitalRead(P_TROLLEY_SW) == 0){
        driveTrolley(0);
        trolleyMode = 3;
      }

    // Trolley homing, repress limit switch
    }else{
      driveTrolley(-100);
      if(digitalRead(P_TROLLEY_SW)){
        //Serial.println("Case 3");
        d_count = 0;
        driveTrolley(0);
        trolleyMode = 1;
        return 1;
      }
    }
    return 0;
}

int stepperMode = 1;
unsigned long stepper_t = 0;

int homeStepper() {
if(stepperMode == 1){
      stepperRunning_ = true;
      driveStepper(-1500);
      if(digitalRead(P_VERT_SW) == 1){
        //Serial.println("Case 1");
        driveStepper(0);
        stepperMode = 2;
      }
    // Stepper homing, checking for switch to be released
    } else if(stepperMode == 2){
        //Serial.println("Case 2");
        driveStepper(1000);
        if(digitalRead(P_VERT_SW) == 0){
          driveStepper(0);
          stepperMode = 3;
          stepper_t = millis();
        }
    // Stepper homing, repress limit switch
    } else if (stepperMode == 3){
      //Serial.println("Case 2");
      driveStepper(0);
      if (millis() - stepper_t > 2000) {
         stepperMode = 4;
      }
    }else{
      driveStepper(-500);
      if(digitalRead(P_VERT_SW)){
        //Serial.println("Case 3");
        driveStepper(0);
        vert.setCurrentPosition(0);
        stepperMode = 1;
        stepperRunning_ = false;
        return 1;
        
      }
    }
    return 0;
}



//------------------------------------------------------------------------------------------------
// Functions for reading system sensors
// Define function for reading rotary encoder value
double readRotation()
{
  uint16_t raw;
  double theta;
  // NOTE: uncomment once connected to live system
  raw = rotary_enc.getPos();  //receive number of counts
  theta = 2*PI/4095*(double)raw;  //convert counts to degrees

  return theta;
}

// Define function for reading vertical translational joint encoder value (stepper motor)
double readVerticalPos(){
  double countsPerRot = 400;
  double dispPerRot = 0.009525;
<<<<<<< HEAD
  double pos = 0.3955;
  v_count = vert.currentPosition();
  double vertDisp = (v_count/countsPerRot)*dispPerRot;

  // Return calculated output, uncomment when connected to live system
  return vertDisp;
=======
  double pos = 0.3955; //m
  double vertDisp = (v_count/countsPerRot)*dispPerRot; //displacement down from 0

  // Return calculated output, uncomment when connected to live system
  // Returns the distance from frame 3 to the ground
  return offset-vertDisp;
  
>>>>>>> 6121c8473da479bab44fb09f3022fa75d79f5cf1
}

// Define function for reading trolley encoder value
double readTrolleyPos() {
  double countsPerRot = 2048;
  double wheel_d = 0.02905;


  //Include the initial offset from the center post to the trolley offset 0.2275
  // Return calculated output, uncomment when connected to live system
  return (d_count/countsPerRot)*(wheel_d*PI);
}


// Define function for reading High-Voltage Elimination Circuit (HVEC) temperature
// UNUSED AS WE NEVER PUT A THERMISTOR ON IT
void readHVECTemp()
{
  // Read the ADC for HVEC temperature and store in ROS-compatible variable
  double hvec_reading = EE_ADC.read(2);
  //hvec_temp.data = float(hvec_reading);
  // Set to 30 for debugging purposes only
  //hvec_temp.data = float(30);
  // Publish the reading to ROS topic 'hvec'
  //hvec_pub.publish(&hvec_temp);
}

//-----------------------------------------------------------------------------------------------------------------------
//Interrupt functions (Only PCINT0 enabled as of 3/14)

ISR (PCINT0_vect) // handle pin change interrupt for D8 to D13 here
 {
  if(hydrate_) {
    flow_trolley++;
  }
  else {
    d_count += trolley_enc.count(); // handles the trolley counting encoder
  }
}

ISR (PCINT1_vect) // handle pin change interrupt for A0 to A5 here
 {

 }

ISR (PCINT2_vect) // handle pin change interrupt for PCINT16-23
 {
  flow_box++;
 }

// Define function for watering the growing zone
int runWater() {

  // Activate the solenoid
  digitalWrite(P_WATER_SOLE, HIGH);

  // Deactivate solenoid after desired volume has been dispensed
  if(flow_box >= flow_counts) {
    digitalWrite(P_WATER_SOLE, LOW);
    hydrate_ = false;
    return 1;
  }

  else {
    return 0;
  }
}

int runHVEC() {
  unsigned long HVEC_t = millis() - HVEC_start_t;

  if (pulses > 0) {
    if (HVEC_t <= offTime) {
      digitalWrite(P_HVEC, LOW);
    }
    else if (HVEC_t <= (offTime+onTime)) {
      digitalWrite(P_HVEC, HIGH);
    }
    else {
      digitalWrite(P_HVEC, LOW);
      pulses--;
      HVEC_start_t = millis();
    }
    return 0;
  }
  else {
    digitalWrite(P_HVEC, LOW);
    return 1;
  }

}

//----------------------------------------------------------------------------------------------------
// Main Loop
// Negative steps move stepper down
void loop(){
  //Update encoders
  t = millis();

  if (stepperRunning_) {
    theta = theta;
  } else {
    theta = (readRotation());
  }
  d = (-1*readTrolleyPos()+0.2275);
  v = (readVerticalPos());


  //float location[] = {theta, d, v};
  float location[] = {(float)theta, (float)d, (float)(MAX_V_HEIGHT-v)};

  // Publish joint encoder values to ROS
  //double encoder_talk.data = [theta, d, v];
  encoder_array.data = location;
  encoder_array.data_length = 3;
  complete.data = 1;

  //Publish updated encoder values every second
  if (t - prev_t_1s > 1000) {
    encoder_pub.publish(&encoder_array);
    prev_t_1s = t;
  }


  nh.spinOnce();

  if(homeTrolley_){
    if (homeTrolley() == 1) {
      homeTrolley_ = false;
      complete_pub.publish(&complete);
    }
  }

  if(homeStepper_){
    if (homeStepper() == 1) {
      homeStepper_ = false;
      complete_pub.publish(&complete);
    }
  }

  if(roboControl_){
    if (robotControl() == 1) {
      roboControl_ = false;
      complete_pub.publish(&complete);
    }
  }

  if(HVEC_){
    if (runHVEC() == 1) {
      HVEC_ = false;
      complete_pub.publish(&complete);
    }
  }

  if(hydrate_){
    if (runWater() == 1) {
      hydrate_ = false;
      complete_pub.publish(&complete);
    }
  }


  prev_t = t;
  delay(1);
}
