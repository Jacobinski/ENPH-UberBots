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
#define DETECTION_SWITCH 4 //Digital Input
#define IR_THRESH 5 //Volts

int cat = 0;

void setup() {
  #include <phys253setup.txt>

  ServoTINAH RCServo0; //Controls the rotation and position of the base
  ServoTINAH RCServo1; //Controls ascending and descending of arm
  ServoTINAH RCServo2; //Controls claw actuation
  
  //Set pin mode for microswitch passenger detection
  pinMode(DETECTION_SWITCH, INPUT);
  digitalWrite(DETECTION_SWITCH, LOW);
  
  enableExternalInterrupt(INT1,LOW);
<<<<<<< HEAD
  enableExternalInterrupt(INT2, FALLING);
  //nav_init();
=======
  nav_init();
>>>>>>> f3106b8db0fb00a9d3c2b72b21b498e4b4c90705
  Serial.begin(9600); Serial.println("Start code");
}

void loop() {
<<<<<<< HEAD
  motor.speed(LEFT_MOTOR, 65);
  motor.speed(RIGHT_MOTOR, 65);
  cat++;
  followTape();
  if(cat = 300){ 
    LCD.clear();
    LCD.setCursor(0,0);
    LCD.print("Count: ");
    LCD.print(leftWheelCounter);
    LCD.setCursor(0,1);
    LCD.print("Distance: ");
    LCD.print(distanceTraveled(leftWheelCounter));
    cat = 0;
  }
=======
  navigate();  
>>>>>>> f3106b8db0fb00a9d3c2b72b21b498e4b4c90705
}
