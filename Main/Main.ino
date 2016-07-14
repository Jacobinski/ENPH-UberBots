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
int pN;      int* pN_p; // Holds the previous node (pN) in memory
int cN;      int* cN_p; // Holds the current node (cN) in memory
QueueList <int> fN; // Holds all of the future nodes (fN) in memory

//Prototype to fix errors
StackList<int> pathFind(int start, int finish, int collision);
void enableExternalInterrupt(unsigned int INTX, unsigned int mode);

void setup() {
  #include <phys253setup.txt>
  Serial.begin(9600);
  passenger = false;
  dir = SOUTH; dir_p = &dir;
  cN = 19; cN_p = &cN;
  pN = 1; pN_p = &pN; //This initial value does not matter
  Serial.println("Start code");
  StackList <int> path = pathFind(19,7,999); 
  while(!path.isEmpty()){
    fN.push(path.pop()); //path only exists in setup() [bug?]. This is a fix
  }
  enableExternalInterrupt(INT1,LOW);
  delay(200);
}

void loop() {
  /*
 while(!fN.isEmpty()){
  	Serial.println(fN.pop());
    }
  delay(1000000);
  */
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
