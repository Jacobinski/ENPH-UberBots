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

int checkpoint; // Checkpoints on the map that the robot wishes to reach
bool passenger; // Passenger carrying status
bool collision; // Collision flag
char turnDir; // The direction of the next turn
char dir;  char* dir_p; // Direction, N,S,E,W
int cN;    int* cN_p; // Holds the current node (cN) in memory -> Node which robot is approaching
QueueList <int> fN; // Holds all of the future nodes (fN) in memory

const int checkpointNodes[6] = {2,5,13,19,17,4}; //Actually {1,5,13,20,17,4}
const int path0 = {5};
const int path1 = {9,10,12,13};
const int path2 = {13,19};
const int path3 = {18,17);
const int path4 = {13,12,9,5,4};
const int path5 = {3,2};

void nav_init(){
  passenger = false;
  collision = false;
  turnDir = UNDEFINED;
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
    // Get next turn direction
    if (turnDir == UNDEFINED) {
      turnDir = turnDirection(cN,fN.peek(),dir); //Get next turn direction
    }
    // Detect Collisions
    if (detectCollision()){
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
       collision = true;
       break; //Avoid the loop
    }
    // Make decisions at intersection
    if (detectIntersection(turnDir)){ // See if we need to turn
      updateParameters(cN_p, fN.pop(), dir_p); // Account for the new position at the intersection.
      turn(turnDir); 
      turnDir = UNDEFINED; // Clear the turn direction
    }
    // Default
    else{
      followTape();
      int counter += 1;
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
    if (fN.isEmpty()){  
        //This is some hacky-stuff. If we're at an end-intersection, proceed forward until a collision
       if (cN == 1 || cN == 7 || cN == 8 || cN == 11 || cN == 15 || cN == 16 || cN == 20){
          while(!detectCollision()){followTape();}
           turn(BACKWARD);
           char cD = dir; //get current Direction
           if (cD == NORTH) {dir = SOUTH;}
           if (cD == SOUTH) {dir = NORTH;}
           if (cD == EAST) {dir = WEST;}
           if (cD == WEST) {dir = EAST;}
           int nxt = getNode(cN,dir);
           updateParameters(cN_p, nxt, dir_p);
           turnDir = UNDEFINED; //Reset turn
           collision = true;
       }  
        motor.stop(LEFT_MOTOR);
        motor.stop(RIGHT_MOTOR);
        if (collision == true){
          StackList <int> path = pathFind(cN,checkpointNodes[checkpoint],dir); 
          while(!path.isEmpty()){LCD.clear();LCD.setCursor(0,0);LCD.print(path.peek());fN.push(path.pop());delay(500);} 
          collision = false; //NOTE: If path is empty, we will return to this fN.isEmpty() immediately, and move onto the next checkpoint in the below else if{}
        }
        else if (collision == false){
          checkpoint = (checkpoint+1) % 6; // Cycle checkpoints
          int d; // counter variable
          if (checkpoint == 0){for(d = 0 ; d < sizeof(path0); d++){fN.push(path0[d]);}}
          if (checkpoint == 1){for(d = 0 ; d < sizeof(path1); d++){fN.push(path1[d]);}}
          if (checkpoint == 2){for(d = 0 ; d < sizeof(path2); d++){fN.push(path2[d]);}}
          if (checkpoint == 3){for(d = 0 ; d < sizeof(path3); d++){fN.push(path3[d]);}}
          if (checkpoint == 4){for(d = 0 ; d < sizeof(path4); d++){fN.push(path4[d]);}}
          if (checkpoint == 5){for(d = 0 ; d < sizeof(path5); d++){fN.push(path5[d]);}}
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
