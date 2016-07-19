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
#define BACKWARD 'B'
#define UNDEFINED 'U'

int c; // Counter
int finish; // ending location
bool passenger; // Passenger carrying status
char turnDir; // The direction of the next turn
char dir;  char* dir_p; // Direction, N,S,E,W
int cN;    int* cN_p; // Holds the current node (cN) in memory
QueueList <int> fN; // Holds all of the future nodes (fN) in memory

//Prototypes for functions the complier misses
StackList<int> pathFind(int start, int finish, char direction);
void enableExternalInterrupt(unsigned int INTX, unsigned int mode);
char turnDirection(int cN, int fN, char dir);
bool detectCollision();
void turn(char dir);
void updateParameters(int *cN, int fN, char *dir);
bool detectIntersection(char dir);
int getNode(int cN, char dir);

void setup() {
  #include <phys253setup.txt>
  enableExternalInterrupt(INT1,LOW);
  passenger = false;
  turnDir = UNDEFINED;
  c = 0;
  finish = 1;
  dir = NORTH; dir_p = &dir;
  cN = 19; cN_p = &cN; // Initial value of current position. Defined as the node the vehicle is approaching.
  Serial.begin(9600); Serial.println("Start code");
}

void loop() {
  while(!fN.isEmpty()){
  	if (turnDir == UNDEFINED) {
  		turnDir = turnDirection(cN,fN.peek(),dir); //Get next turn direction
  	}
    if (detectCollision()){
       //int rev_node = cN;
       //cN = fN.pop();
       while(!fN.isEmpty()) {fN.pop();} //Clear the fN list
       turn(BACKWARD);
       char cD = dir; //get current Direction
       if (cD == NORTH) {dir = SOUTH;}
       if (cD == SOUTH) {dir = NORTH;}
       if (cD == EAST) {dir = WEST;}
       if (cD == WEST) {dir = EAST;}
       int nxt = getNode(cN,dir);
       updateParameters(cN_p, nxt, dir_p);
       break; //Avoid the loop
    }
    //while (detectCollision()){motor.stop_all();delay(10000);}
    if (detectIntersection(turnDir)){ // See if we need to turn
      updateParameters(cN_p, fN.pop(), dir_p); // Account for the new position at the intersection.
      turn(turnDir); 
      turnDir = UNDEFINED; // Clear the turn direction
    }
    else{
      followTape();
      c = c + 1;
      if (c == 20){
        LCD.clear();
        LCD.setCursor(0,0);
        LCD.print("cN:");
        LCD.print(cN);
        LCD.print(" fN:");
        LCD.print(fN.peek());
        LCD.setCursor(0,1);
        LCD.print("dir:");
        LCD.print(dir)
        LCD.print(" turn:");
        LCD.print(turnDir);
      }
    }
  }
    if (fN.isEmpty() && cN != finish){
        LCD.clear();
        LCD.setCursor(0,0);
        LCD.print("cN ");
        LCD.print(cN);
        LCD.print(" dir ");
        LCD.print(dir);
        delay(4000);
        StackList <int> path = pathFind(cN,finish,dir); 
        while(!path.isEmpty()){
          LCD.clear();
          LCD.setCursor(0,0);
          LCD.print(path.peek());
          fN.push(path.pop());
          delay(500);
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

