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
#define NUMCHECKPOINTS 7

int checkpoint; // Checkpoints on the map that the robot wishes to reach
int counter; // Displaying driving stats
bool passenger; // Passenger carrying status
bool collision; // Collision flag
char turnDir; // The direction of the next turn
char dir;  char* dir_p; // Direction, N,S,E,W
int cN;    int* cN_p; // Holds the current node (cN) in memory -> Node which robot is approaching
QueueList <int> fN; // Holds all of the future nodes (fN) in memory

const int checkpointNodes[NUMCHECKPOINTS] = {1,7,11,16,20,18,3};

void nav_init(){
  passenger = false;
  collision = false;
  turnDir = UNDEFINED;
  counter = 0;
  dir = SOUTH; dir_p = &dir;
  cN = 19; cN_p = &cN; checkpoint = 4; 
  while(!startbutton()){
      int counter = counter + 1;
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
        if (cN == 19) {cN = 2; checkpoint = 0;}
        else          {cN = 19; checkpoint = 6;}
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
       char cD = dir;  //get current Direction
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
    // Detect passenger
    if (analogRead(QRD) > QRD_THRESH){
      // Jenny's arm function
      passenger = true;
      StackList <int> dropoff_path1 = pathFind(cN,4,dir); 
      StackList <int> dropoff_path2 = pathFind(cN,17,dir); 
      if (dropoff_path1.count() < dropoff_path2.count()){ //We want the shorter path
          while(!dropoff_path1.isEmpty()){LCD.clear();LCD.setCursor(0,0);LCD.print(dropoff_path1.peek());fN.push(dropoff_path1.pop());delay(500);} 
          fN.push(17); //Direction to face
      } 
      else {
          while(!dropoff_path2.isEmpty()){LCD.clear();LCD.setCursor(0,0);LCD.print(dropoff_path2.peek());fN.push(dropoff_path2.pop());delay(500);} 
          fN.push(4); //Direction to face
      }
    }
    // Default
    else{
      followTape();
      counter = counter + 1;
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
       }  
       // If we have a passenger, drop it off
       if (passenger == true){ 
          checkpoint = 5; //Always go to bottom right corner afterwards. (5 {+ 1 from .isEmpty()})
          int ti = millis(); //Initial time
          int tf = millis(); //Final time
          while (tf-ti < 2000){ //Go forward for 2 seconds
            followTape();
            tf = millis();
          }
          //DROP OFF PASSENGER
       }
        motor.stop(LEFT_MOTOR);
        motor.stop(RIGHT_MOTOR);
        if (collision == true){
          StackList <int> path = pathFind(cN,checkpointNodes[checkpoint],dir); 
          while(!path.isEmpty()){LCD.clear();LCD.setCursor(0,0);LCD.print(path.peek());fN.push(path.pop());delay(500);} 
          collision = false; //NOTE: If path is empty, we will return to this fN.isEmpty() immediately, and move onto the next checkpoint in the below else if{}
        }
        else if (collision == false){
          checkpoint = (checkpoint+1) % NUMCHECKPOINTS; // Cycle checkpoints
          StackList <int> path = pathFind(cN,checkpointNodes[checkpoint],dir); 
          while(!path.isEmpty()){LCD.clear();LCD.setCursor(0,0);LCD.print(path.peek());fN.push(path.pop());delay(500);} 
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
