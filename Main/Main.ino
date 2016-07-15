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

bool passenger; // Passenger carrying status
char turnDir; // The direction of the next turn
char dir;  char* dir_p; // Direction, N,S,E,W
int cN;    int* cN_p; // Holds the current node (cN) in memory
QueueList <int> fN; // Holds all of the future nodes (fN) in memory

//Prototypes for functions the complier misses
StackList<int> pathFind(int start, int finish, int collision);
void enableExternalInterrupt(unsigned int INTX, unsigned int mode);

void setup() {
  #include <phys253setup.txt>
  enableExternalInterrupt(INT1,LOW);
  passenger = false;
  turnDir = UNDEFINED;
  dir = NORTH; dir_p = &dir;
  cN = 19; cN_p = &cN; // Initial value of current position. Defined as the node the vehicle is approaching.
  StackList <int> path = pathFind(19,1,999); 
  while(!path.isEmpty()){
    fN.push(path.pop()); //path only exists in setup() [bug?]. This is a fix
  }
  Serial.begin(9600); Serial.println("Start code");
}

void loop() {
  /*while(!fN.isEmpty()){
  	if (turnDir == UNDEFINED) {
  		turnDir = turnDirection(cN,fN.peek(),dir); //Get next turn direction
  	}
    while (detectCollision()){motor.stop_all();delay(10000);}
    if (detectIntersection(turnDir)){ // See if we need to turn
      updateParameters(cN_p, fN.pop(), dir_p); // Account for the new position at the intersection.
      turn(turnDir); 
      turnDir = UNDEFINED; // Clear the turn direction
    }
    else{
      followTape();
    }
  }*/
  while(!detectIntersection(FORWARD)){
    r_followTape();
  }
  
  // Once the motor has run out of commands
    motor.stop_all();
    LCD.clear();
    LCD.home();
    LCD.print("Done");
    delay(10000);
}
