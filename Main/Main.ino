#include <phys253.h>
#include <avr/interrupt.h>
#include <QueueList.h>  /* http://playground.arduino.cc/Code/QueueList */
#include <StackList.h>  /* http://playground.arduino.cc/Code/StackList */
#include <LiquidCrystal.h>

#define NORTH 'N'
#define SOUTH 'S'
#define EAST 'E'
#define WEST 'W'
#define RIGHT 'R'
#define LEFT 'L'
#define FORWARD 'F'
#define UNDEFINED 'U'
#define LEFT_MOTOR 1 //Motor input
#define RIGHT_MOTOR 2 //Motor input

#define LEFT_IR 0 //Analog Input
#define RIGHT_IR 2 //Analog Input
#define PROPORTIONAL 7 //Analog Input (Don't plug anything in)
#define DERIVATIVE 6 //Analog Input (Don't plug anything in)
#define LEFT_INTERSECTION 5 //Analog Input
#define RIGHT_INTERSECTION 1 //Analog Input
#define LEFT_TAPE 4 //Analog Input
#define RIGHT_TAPE 3 //Analog Input
#define LEFT_FWD_COLLISION 7 //Digital Input
#define RIGHT_FWD_COLLISION 5 //Digital Input
#define STOP_BUTTON 1 //Digital Input
#define detectionPin_passenger1 4 //Digital Input
#define detectionPin_passenger2 6 //Digital Input
#define IR_THRESH 6 //Volts

int checkpoint; // Checkpoints on the map that the robot wishes to reach
int counter; // Displaying driving stats
int timeLastIntersection; // Time at last intersection
int leftWheelCounter; // Counts left wheel encoder
int rightWheelCounter; // Counts right wheel encoder
bool lost; // If the robot is lost, this is true
bool smallreverse; //Determines if we just did a small reverse. This is for pathFinding purposes
bool drop; // This will tell us if we are ready to dropOff
bool passenger; // Passenger carrying status
bool collision; // Collision flag
char turnDir; // The direction of the next turn
char dir;  char* dir_p; // Direction, N,S,E,W
int cN;    int* cN_p; // Holds the current node (cN) in memory -> Node which robot is approaching
QueueList <int> fN; // Holds all of the future nodes (fN) in memory

void setup() {
  #include <phys253setup.txt>

  ServoTINAH RCServo0; //Controls the rotation and position of the base
  ServoTINAH RCServo1; //Controls ascending and descending of arm
  ServoTINAH RCServo2; //Controls claw actuation
  
  //Set pin mode for microswitch passenger detection
  pinMode(detectionPin_passenger1, INPUT);
  pinMode(detectionPin_passenger2, INPUT);
  digitalWrite(detectionPin_passenger1, LOW);
  digitalWrite(detectionPin_passenger2, LOW);
  
  enableExternalInterrupt(INT1,LOW);
  enableExternalInterrupt(INT2, FALLING); // Left wheel interupt
  enableExternalInterrupt(INT3, FALLING); // Right wheel interupt
  initialize();
  Serial.begin(9600); Serial.println("Start code");
}

void loop() {
  navigate();
}
