#include <AccelStepper.h>

// Motor pin assignment
#define P_VERT_STEP 4 //Vertical stepper step pulse pin (PWM-enabled)
#define P_VERT_DIR 24 //Vertical stepper step direction pin

// Define interface type associated with a stepper driver
#define motorInterfaceType 1

// Define stepper objects and pins used to drive them
AccelStepper vert(motorInterfaceType, P_VERT_STEP, P_VERT_DIR);

void setup() {
  // Initialize serial communication for Arduino terminal
  Serial.begin(9600);
  Serial.println("Starting...");
  // Define Arduino pin operation
  pinMode(P_VERT_STEP, OUTPUT); //Vertical stepper step pulse pin (PWM-enabled) 
  pinMode(P_VERT_DIR, OUTPUT); //Vertical stepper step direction pin

  // Initialize motor acceleration and speed parameters
  vert.setMaxSpeed(2000);
  vert.setAcceleration(500);
}

/* Manual method for driving stepper motor */
//void loop() {
//  digitalWrite(P_VERT_DIR, HIGH);
//  
//  for (int x = 0; x < 1600; x++){
//    digitalWrite(P_VERT_STEP, HIGH);
//    delayMicroseconds(2000);
//    digitalWrite(P_VERT_STEP, LOW);
//    delayMicroseconds(2000);
//  }
//  
//  delay(1000);
//
//  digitalWrite(P_VERT_DIR, LOW);
//  for (int x = 0; x < 1600; x++){
//    digitalWrite(P_VERT_STEP, HIGH);
//    delayMicroseconds(2000);
//    digitalWrite(P_VERT_STEP, LOW);
//    delayMicroseconds(2000);
//  }
//}

void loop() {
  int steps = 0;

  // Read user input if available
  if (Serial.available()){

    // Convert input string to character array  
    String input = Serial.readString();
    char command[input.length()+1];
    input.toCharArray(command, input.length()+1);

    // Convert character array to integer data type
    steps = atoi(command);

      // Drive the motor the desired number of steps
      vert.moveTo(steps);
      while(vert.distanceToGo()){
        vert.run();
      }   
  }
}
