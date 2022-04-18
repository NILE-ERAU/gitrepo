/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

ros::NodeHandle  nh;

// Define function for receiving user input through ROS
void receive(const std_msgs::String& cmd_msg) {

  // Store the user input in separate variable
  String user = cmd_msg.data;

  if(user.equalsIgnoreCase("on") == true) {
    digitalWrite(2, HIGH);
  }
  else if (user.equalsIgnoreCase("off") == false) {
    digitalWrite(2, LOW);
  }
  else 
    digitalWrite(2, LOW);
}
//void messageCb( const std_msgs::Empty& toggle_msg){
//  digitalWrite(2, HIGH);   // blink the led
//  delay(500);
//  digitalWrite(2, LOW);
//  delay(500);
//}

ros::Subscriber<std_msgs::String> sub("LED", &receive);

void setup()
{ 
  pinMode(2, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{  
//  digitalWrite(2, HIGH);
//  delay(500);
//  digitalWrite(2,LOW);
//  delay(500);
  nh.spinOnce();
  delay(1);
}
