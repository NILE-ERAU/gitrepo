// Import libraries for ROS nodes
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>

ros::NodeHandle nh;

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
  //roboControl(coords);
}

//---------------------------------------------------------------------------------------------------
// Define ROS subscribers
// Node home_sub subscribes to topic "home" and references "homing" function
ros::Subscriber<std_msgs::String> home_sub("home", &homing);
// Node move_sub subscribes to topic "coordinates" and references "movement" function
ros::Subscriber<std_msgs::String> move_sub("coordinates", &movement);

std_msgs::Float64MultiArray encoder_array;

//Node encoder_pub publishers to topic "encoder" and posts encoder_array data
ros::Publisher encoder_pub("encoder", &encoder_array);

void setup(){
    // Initialize ROS node and define topic subscribers
  nh.initNode();
  nh.subscribe(home_sub);
  nh.subscribe(move_sub);


  //Define ROS publishers
  nh.advertise(encoder_pub);

}


//-----------------------------------------------------------------------------------------------------------------------
// Robot Functions

//int roboControl(double theta_d, double d_d, double v_d) {
//int roboControl(char* coords) {
//
//  // Parse the coordinates character-array into sections and store relevant variables
//  // then convert char entries to doubles using atof() function
//  double theta_d = atof(strtok(coords, " :,"));
//  double d_d = atof(strtok(NULL, " :,"));
//  double v_d = atof(strtok(NULL, " :,"));
//
//  //Vairable Initialization
//  static double elast_theta = 0, elast_d = 0;
//  static double eint_theta = 0, eint_d = 0;
//  int roboControlState = 0;
//  static int controlMode = 1;
//  double dt = (double)(t - prev_t)*0.001; //Seconds
//
//  //Theta
//  double Kp_theta = 250, Ki_theta = 10, Kd_theta = 10;
//  double pwm_theta;
//  double e_theta = (theta_d - theta);
//  double edot_theta = (e_theta - elast_theta)/(double)(dt);
//  eint_theta = eint_theta + elast_theta*(double)(dt);
//
//  //D
//  double Kp_d = 250, Ki_d = 10, Kd_d = 5;
//  static double pwm_d = 0;
//  double e_d = d_d - d;
//  double edot_d = (e_d - elast_d)/(double)(dt);
//  eint_d = eint_d + elast_d*(double)(dt);
//
//  //V
//  double steps_v = 0;
//
//  //Trolley Control
//  if(controlMode == 1){
//    if(abs(e_theta) < 0.01){
//      //Set PWM_D Value
//      pwm_theta = 0;
//      driveRotation(pwm_theta);
//      //Reset values
//      eint_theta = 0;
//      elast_theta = 0;
//      controlMode++;
//    } else {
//      //Set PWM_D value
//      pwm_theta = -1*(Kp_theta*e_theta + Ki_theta*eint_theta + Kd_theta*edot_theta);
//      pwm_theta = (pwm_theta > 100) ? 100 : pwm_theta; //0-100
//      pwm_theta = (pwm_theta < -100) ? -100 : pwm_theta; //0-100
//      //Set PWM_D values
//      elast_theta = e_theta;
//      driveRotation(pwm_theta);
//    }
//  } else if(controlMode == 2){
//    //Rotation Control
//    if(abs(e_d) < 0.001){
//      //Set PWM_D Value
//      pwm_d = 0;
//      driveTrolley(pwm_d);
//      //Reset values
//      eint_d = 0;
//      elast_d = 0;
//      controlMode++;
//    } else {
//      //Set PWM_D value
//      pwm_d = Kp_d*e_d + Ki_d*eint_d + Kd_d*edot_d;
//      pwm_d = (pwm_d > 255.0) ? 255 : pwm_d; //-255 to 255
//      driveTrolley(pwm_d);
//      //Set PWM_D values
//      elast_d = e_d;
//    }
//  } else if(controlMode == 3){
//      steps_v = v_d*400/0.009525;
//      stepStepper(steps_v);
//      controlMode = 4;
//  } else {
//      roboControlState = 1;
//      controlMode = 1;
//  }
//
//  //Reference Values
//  //Serial.print("Rotation Position: ");
//  //Serial.print(theta, 4);
//  //Serial.print(", Rotation Error: ");
//  //Serial.print(e_theta);
//  //Serial.print(", Rotation PWM: ");
//  //Serial.print(pwm_theta);
//  //Serial.print(", Theta_d: ");
//  //Serial.println(theta_d);
//
//  return roboControlState;
//}

void loop(){
    float location[] = {1, 2, 3};

  // Publish joint encoder values to ROS
  //double encoder_talk.data = [theta, d, v];
  encoder_array.data = location;
  encoder_array.data_length = 3;

  // Publish updated encoder values
  encoder_pub.publish(&encoder_array);
  
  nh.spinOnce();
  delay(1);
}
