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
char dir;    char* dir_p; // Direction, N,S,E,W
int cN;      int* cN_p; // Holds the current node (cN) in memory
QueueList <int> fN; // Holds all of the future nodes (fN) in memory

//Prototype to fix errors
StackList<int> pathFind(int start, int finish, int collision);
void enableExternalInterrupt(unsigned int INTX, unsigned int mode);

void setup() {
  #include <phys253setup.txt>
  enableExternalInterrupt(INT1,LOW);
  passenger = false;
  turnDir = UNDEFINED;
  dir = SOUTH; dir_p = &dir;
  cN = 19; cN_p = &cN; // Initial value of current position. Defined as the node the vehicle is approaching.
  StackList <int> path = pathFind(19,7,999); 
  while(!path.isEmpty()){
    fN.push(path.pop()); //path only exists in setup() [bug?]. This is a fix
  }
  //Serial.begin(9600); Serial.println("Start code");
}

void loop() {
  while(!fN.isEmpty()){
  	if (turnDir == UNDEFINED) {
  		turnDir = turnDirection(cN_p,fN.pop(),dir_p); //Get next turn direction
  	}



  }

  bool intersection = detectIntersection();
  if (!fN.isEmpty()){
    if (intersection == true){
      turnDir = turnDirection(cN_p,fN.pop(),dir_p);
      Serial.println("Intersection Detected");
      LCD.clear();
      LCD.home();
      LCD.print(turnDir);
      turn(turnDir);
      //followTape();
    }
    else{
      Serial.println("Following Tape");
      followTape();
    }
  }
  else{
    motor.stop_all();
    LCD.clear();
    LCD.home();
    LCD.print("Done");
    delay(10000);
  }
}
