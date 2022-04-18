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

#define COUNT_PER_LITER 932
//-----------------------------------------------------------------------------------------------------------------------
//Stepper Stuff
#define motorInterfaceType 1
AccelStepper vert(motorInterfaceType, P_VERT_STEP, P_VERT_DIR);
//-----------------------------------------------------------------------------------------------------------------------
//Global Variables
unsigned long t = 0; //local time variable, hopefully won't overflow
unsigned long prev_t = 0; //previous time, used for derivative controller
double prevSoilTemp = -273; //used to smooth out temp measurements, dont change init
unsigned long flow_trolley = 0;
unsigned long flow_box = 0;

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

//-----------------------------------------------------------------------------------------------
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
    // Call trolley homing function
  }

  else if(input.equalsIgnoreCase("stepper") == true) {
    homestepper = true;
    // Call stepper homing function
  }

  else if(input.equalsIgnoreCase("rot") == true) {
    homerot = true;
    // Call rotational homing function
  }  
}

// Define function for receiving sensor selector string and reading appropriate sensor
void sense(const std_msgs::String& cmd_msg) {
  // Receive string input and set appropriate flag
  String input = cmd_msg.data;

  if(input.equalsIgnoreCase("moisture") == true) {
    // Call function for reading soil moisture sensor
    readSoilMoist();
    // Blink onboard LED once
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200); 
  }

  else if(input.equalsIgnoreCase("soil_temp") == true) {
    // Call function for reading soil temperature sensor
    readSoilTemp();    
    // Blink onboard LED twice
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);    
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
  }

  else if(input.equalsIgnoreCase("hvec_temp") == true) {
    // Call function for reading hvec temperature
    readHVECTemp();
    // Blink LED three times
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);    
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);    
  }
  digitalWrite(LED_BUILTIN, LOW);
}
/* NOTE; as of now the pulseHVEC script is always running, introducing several delays*/
// Define function for receiving integer array of HVEC activation parameters
//int pulseHVEC(int onTime, int offTime, int pulses) {
int pulseHVEC(const std_msgs::Int16MultiArray& cmd_msg) {
  //PERIOD IS DEFINED IN MILLISECONDS

  int input[] = {cmd_msg.data};
  int onTime = input[0];
  int offTime = input[1];
  int pulses = input[2];
  

  // Blink onboard LED number of times corresponding to second entry of input for debugging
  // purposes only
//  for(int i = 0; i < offTime; i++) {
    // Blink onboard LED 
  digitalWrite(LED_BUILTIN, HIGH);
  delay(200);
  digitalWrite(LED_BUILTIN, LOW);
//    nh.spinOnce();
//  }

  //Siren
  // Note these delays cause the ROS interface to lose synchronization and drop connection
  tone(P_BUZZER, 1000);
  delay(2000);
  nh.spinOnce();
  noTone(P_BUZZER);
  delay(1000);
  nh.spinOnce();

  for (int i = 0; i < pulses; i++) {
     digitalWrite(P_HVEC, HIGH);
     delay(onTime);
     digitalWrite(P_HVEC, LOW);
     delay(offTime);
     nh.spinOnce();
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

//---------------------------------------------------------------------------------------------------
// Define ROS subscribers
// Node home_sub subscribes to topic "home" and references "homing" function
ros::Subscriber<std_msgs::String> home_sub("home", &homing);
// Node move_sub subscribes to topic "coordinates" and references "movement" function
ros::Subscriber<std_msgs::Float64MultiArray> move_sub("coordinates", &movement);
// Node sensor_sub subscribes to topic "sensor" and references "sense" function
ros::Subscriber<std_msgs::String> sensor_sub("sensor", &sense);
// Node shock_sub subscribes to topic "shock" and references "pulseHVEC" function
ros::Subscriber<std_msgs::Int16MultiArray> shock_sub("shock", &pulseHVEC);

std_msgs::Float64MultiArray encoder_array, coord_array;
std_msgs::UInt16 soil_moisture;
std_msgs::Float64 soil_temp, hvec_temp;

//Node encoder_pub publishes to topic "encoder" and posts encoder_array data
ros::Publisher encoder_pub("encoder", &encoder_array);
//Node moisture_pub publishes to topic "moist" and posts soil_moisture data
ros::Publisher moisture_pub("moist", &soil_moisture);
//Node temp_pub publishes to topic "temp" and posts soil_temp data
ros::Publisher temp_pub("temp", &soil_temp);
// Node hvec_pub publishes to topic "hvec" and posts hvec_temp data
ros::Publisher hvec_pub("hvec", &hvec_temp);

//// Test Node test_pub publishes to topic "test" and posts coord_array data
//ros::Publisher test_pub("test", &coord_array);


void setup(){
    // Initialize ROS node and define Arduino ROS topic subscribers
  nh.initNode();
  nh.subscribe(home_sub);
  nh.subscribe(move_sub);
  nh.subscribe(sensor_sub);
  nh.subscribe(shock_sub);

  // Initialize onboard LED
  pinMode(LED_BUILTIN, OUTPUT);

  //Define Arduino ROS publishers
  nh.advertise(encoder_pub);
  nh.advertise(moisture_pub);
  nh.advertise(temp_pub);  
  nh.advertise(hvec_pub);
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

  //Test Variables
  roboHome_ = true;
  roboControl_ = true;

}

//-----------------------------------------------------------------------------------------------
// Robot control functions

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

int robotControl(double theta_d, double d_d, double v_d) {
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
      pwm_theta = (pwm_theta > 100) ? 100 : pwm_theta; //0-100
      pwm_theta = (pwm_theta < -100) ? -100 : pwm_theta; //0-100
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
      controlMode = 4;
  } else {
      roboControlState = 1;
      controlMode = 1;
  }

  return roboControlState;
}

// Define function for receiving and processing target task-space coordinates
//void movement(const std_msgs::String& cmd_msg) {
void movement(std_msgs::Float64MultiArray& cmd_msg) {
  // Receive string input and convert to char array
//  String input = cmd_msg.data;
//  char coords[input.length()+1];
//  input.toCharArray(coords, input.length()+1);
  coord_array = cmd_msg;
  double theta_d = (double) coord_array.data[0];
  double d_d = (double) coord_array.data[1];
  double v_d = (double) coord_array.data[2];

  // Call robot movement control function to execute motion
  robotControl(theta_d, d_d, v_d);

//  float coord_data[] = {onTime, offTime, pulses};
//  coord_array.data = coord_data;
//  coord_array.data_length = 3;
//  test_pub.publish(&coord_array);   
}

//------------------------------------------------------------------------------------------------
// Functions for reading system sensors
// Define function for reading rotary encoder value
double readRotation()
{
  uint16_t raw;
  double theta;
//  raw = rotary_enc.getPos();  //receive number of counts
//  theta = 2*PI/4095*(double)raw;  //convert counts to degrees
  // Set theta to 100 for debugging purposes ONLY
  theta = 100;
  return theta;
}

// Define function for reading vertical translational joint encoder value (stepper motor)
double readVerticalPos(){
  double countsPerRot = 400;
  double dispPerRot = 0.009525;
  // Return dispPerRot for debugging purposes ONLY
  return dispPerRot;

  // Return calculated output
  //return (v_count/countsPerRot)*dispPerRot;
  
}

// Define function for reading trolley encoder value
double readTrolleyPos() {
  double countsPerRot = 2048;
  double wheel_d = 0.02905;
  // Return wheel_d for debugging purposes ONLY
  return wheel_d;
  
  //Include the initial offset from the center post to the trolley offset 0.2275
  // Return calculated output
  //return (d_count/countsPerRot)*(wheel_d*PI);
}

// Define function for reading soil moisture sensor
void readSoilMoist()
{
  // Read the moisture sensor value
  int soil_reading = EE_ADC.read(0);
  soil_moisture.data = soil_reading;
  // Set to 50 for debugging purposes only
  soil_moisture.data = 50;
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
   soil_temp.data = float(smoothedTemp);
   // Set to 40 for debugging purposes only
   soil_temp.data = float(40);
   // Publish the reading to ROS topic "temp"
   temp_pub.publish(&soil_temp);
  //Serial Plotter Stuff
  ////Serial.print("Soil_Temp:");
  ////Serial.print(soilTemp);
  ////Serial.print(",");
  ////Serial.print("Filtered_Soil_Temp:");
  ////Serial.println(smoothedTemp);
}

// Define function for reading High-Voltage Elimination Circuit (HVEC) temperature
void readHVECTemp()
{
  // Read the ADC for HVEC temperature and store in ROS-compatible variable
  double hvec_reading = EE_ADC.read(2);
  hvec_temp.data = float(hvec_reading);
  // Set to 30 for debugging purposes only
  hvec_temp.data = float(30);
  // Publish the reading to ROS topic 'hvec'
  hvec_pub.publish(&hvec_temp); 
}

//-----------------------------------------------------------------------------------------------------------------------
//Interrupt functions (Only PCINT0 enabled as of 3/14)

ISR (PCINT0_vect) // handle pin change interrupt for D8 to D13 here
 {
  if(digitalRead(P_TROLLEY_FLOW) == 1) {
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

//----------------------------------------------------------------------------------------------------
// Main Loop
// Negative steps move stepper down
void loop(){
  //Update encoders
  float theta = float(readRotation());
  float d = float(-1*readTrolleyPos()+0.2275);
  float v = float(readVerticalPos());
  
  //float location[] = {theta, d, v};
  float location[] = {theta, d, v};

  // Publish joint encoder values to ROS
  //double encoder_talk.data = [theta, d, v];
  encoder_array.data = location;
  encoder_array.data_length = 3;

  // Publish updated encoder values
  encoder_pub.publish(&encoder_array);
  
  nh.spinOnce();
  delay(1);
}
