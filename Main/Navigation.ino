/*
  Title:
  Navigation
  Description:
  This tab is responsible for navigating the map by calling functions from
  other tabs. If no passenger is present, it will create a map to navigate. 
  If a passenger is present, it will attempt to reach the drop-off area and
  then resume navigating the map.
    
  Created: July 19, 2016
    By: Jacob Budzis
  Modified: July 19, 2016
    By: Jacob Budzis
*/

#define NORTH 'N'
#define SOUTH 'S'
#define EAST 'E'
#define WEST 'W'
#define RIGHT 'R'
#define LEFT 'L'
#define FORWARD 'F'
#define BACKWARD 'B'
#define UNDEFINED 'U'

int counter; // Counter
int checkpoint; // Checkpoints on the map that the robot wishes to reach
bool passenger; // Passenger carrying status
char turnDir; // The direction of the next turn
char dir;  char* dir_p; // Direction, N,S,E,W
int cN;    int* cN_p; // Holds the current node (cN) in memory -> Node which robot is approaching
QueueList <int> fN; // Holds all of the future nodes (fN) in memory

const int checkpointNodes[6] = {2,5,13,19,17,4}; //Actually {1,5,13,20,17,4}

void nav_init(){
  passenger = false;
  turnDir = UNDEFINED;
  counter = 0; // For display
  checkpoint = 4; 
  dir = SOUTH; dir_p = &dir;
  cN = 19; cN_p = &cN;
  while(!startbutton()){
      counter += 1;
      if (counter == 30){
        LCD.clear();
        LCD.setCursor(0,0);
        LCD.print("Starting node:");
        LCD.print(cN);
        LCD.setCursor(0,1);
        LCD.print("Press start when ready");
        counter = 0;
      }
      if(stopbutton()){
        if (cN == 19) {cN = 2;}
        else          {cN = 19;}
        delay(300); // Just to make sure we have time to unpress
      }
  }
}

void navigate(){
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
       turnDir = UNDEFINED; //Reset turn
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
      counter += 1;
      if (counter == 20){
        LCD.clear();
        LCD.setCursor(0,0);
        LCD.print("cN:");
        LCD.print(cN);
        LCD.print(" fN:");
        LCD.print(fN.peek());
        LCD.setCursor(0,1);
        LCD.print("dir:");
        LCD.print(dir);
        LCD.print(" turn:");
        LCD.print(turnDir);
        counter = 0;
      }
    }
  }
    if (fN.isEmpty()){  // Proceed to new checkpoint
        motor.stop(LEFT_MOTOR);
        motor.stop(RIGHT_MOTOR);
        StackList <int> path = pathFind(cN,checkpointNodes[checkpoint],dir); 
        while(!path.isEmpty()){
          LCD.clear();
          LCD.setCursor(0,0);
          LCD.print(path.peek());
          fN.push(path.pop());
          delay(500);
          } 
        checkpoint = (checkpoint+1) % 6; // Cycle checkpoints
    }
    else{
      motor.stop_all();
      LCD.clear();
      LCD.home();
      LCD.print("Done");
      delay(10000);
    }
}
