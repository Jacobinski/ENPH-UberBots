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
#define PROPORTIONAL 7 //Analog Input (Don't plug anything in)
#define DERIVATIVE 6 //Analog Input (Don't plug anything in)
#define LEFT_INTERSECTION 5 //Analog Input
#define RIGHT_INTERSECTION 1 //Analog Input
#define LEFT_TAPE 4 //Analog Input
#define RIGHT_TAPE 3 //Analog Input
#define LEFT_FWD_COLLISION 12 //Digital Input
#define RIGHT_FWD_COLLISION 3 //Digital Input
#define STOP_BUTTON 1 //Digital Input

void setup() {
  #include <phys253setup.txt>
  enableExternalInterrupt(INT1,LOW);
  nav_init();
  Serial.begin(9600); Serial.println("Start code");
}

void loop() {
  navigate();  
}
