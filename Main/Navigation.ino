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
#define BACKWARD 'B' //This is when we rotate the robot 180 degrees
#define REVERSE 'R' //This is when we go straight backwards
#define UNDEFINED 'U'
#define NUMCHECKPOINTS 7
#define SMALLREVERSETIME 2000 //Threshold for straight reverse vs. U turn 

int checkpoint; // Checkpoints on the map that the robot wishes to reach
int counter; // Displaying driving stats
int timeLastIntersection; // Time at last intersection
int leftWheelCounter; // Counts left wheel encoder
int rightWheelCounter; // Counts right wheel encoder
bool lost; // If the robot is lost, this is true
bool smallreverse; //Determines if we just did a small reverse. This is for pathFinding purposes
bool passenger; // Passenger carrying status
bool collision; // Collision flag
char turnDir; // The direction of the next turn
char dir;  char* dir_p; // Direction, N,S,E,W
int cN;    int* cN_p; // Holds the current node (cN) in memory -> Node which robot is approaching
QueueList <int> fN; // Holds all of the future nodes (fN) in memory

const int checkpointNodes[NUMCHECKPOINTS] = {1,7,11,16,20,18,3};

void initialize(){
  passenger = false;
  collision = false;
  smallreverse = false;
  lost = false;
  leftWheelCounter = 0;
  rightWheelCounter = 0;
  timeLastIntersection = millis();
  turnDir = UNDEFINED;
  counter = 0;
  dir = SOUTH; dir_p = &dir;
  cN = 19; cN_p = &cN; checkpoint = 4; 
  while(!startbutton()){
      int counter = counter + 1;
      if (counter == 300){
        LCD.clear();
        LCD.setCursor(0,0);
        LCD.print("Initial node:");
        LCD.print(cN);
        LCD.setCursor(0,1);
        LCD.print("Press Start");
        counter = 0;
      }
      if(stopbutton()){
        if (cN == 19) {cN = 2; checkpoint = 0;}
        else          {cN = 19; checkpoint = 4;}
        delay(300); // Just to make sure we have time to unpress
      }
  }
  RCServo1.write(180); //Arm starts up
  delay(700);
  RCServo0.write(90); //Position of the base starts in the middle
  RCServo2.write(0); //Claw starts open
}

void navigate(){
  while(!fN.isEmpty()){
    // Get next turn direction
    if (turnDir == UNDEFINED) {
      turnDir = turnDirection(cN,fN.peek(),dir); //Get next turn direction
    }

    // Detect Collisions
    if (detectCollision()){
      nav_Collision();
      break; //Avoid the loop
    }

    // Make decisions at intersection
    if (detectIntersection(turnDir)){ // See if we need to turn
      timeLastIntersection = millis(); // Update turn
      resetWheelCounters();
      updateParameters(cN_p, fN.pop(), dir_p); // Account for the new position at the intersection.
      turn(turnDir); 
      turnDir = UNDEFINED; // Clear the turn direction
    }

    // Detect passenger -> Quadruple check to avoid noise
    double left_ir = analogRead(LEFT_IR);
    double right_ir = analogRead(RIGHT_IR);
    if (((left_ir*5.0/1024.0) > 0.5 || (right_ir*5.0/1024.0) > 0.5)){
      for(int i=0; i<4;i++){
        left_ir = left_ir + analogRead(LEFT_IR);
        right_ir = right_ir + analogRead(RIGHT_IR);
      }
    }
    // Check the quadruple read
    if (((left_ir*5.0/1024.0) > IR_THRESH || (right_ir*5.0/1024.0) > IR_THRESH) && passenger == false){
      armPickup();
    }
    // Default
    else{
      followTape();
      counter = counter + 1;
      if (counter == 20){
        LCD.clear();
        LCD.setCursor(0,0);
        LCD.print("L: ");
        LCD.print(distanceTraveled(leftWheelCounter));
        LCD.setCursor(0,1);
        LCD.print("R: ");
        LCD.print(distanceTraveled(rightWheelCounter));
        counter = 0;
      }
      /*if (counter == 20){
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
      }*/
      /*if (counter == 20){
        LCD.clear();
        LCD.setCursor(0,0);
        LCD.print("L: ");
        LCD.print(analogRead(LEFT_IR));
        LCD.setCursor(0,1);
        LCD.print("R: ");
        LCD.print(analogRead(RIGHT_IR));
      }*/
      
    }
  }
  if (fN.isEmpty()){  
        //If we're at an end-intersection, proceed forward until a collision
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
       // If we have a passenger not at the end point, find an endpoint
       if ((passenger == true) && (cN != 4 || cN != 17)){ 
          StackList <int> dropoff_path1 = pathFind(cN,4,dir); 
          StackList <int> dropoff_path2 = pathFind(cN,17,dir);
          turnDir = UNDEFINED; //Reset turn
          if(dropoff_path1.isEmpty() && dropoff_path2.isEmpty()){ //If no available path, turn around
             turn(BACKWARD);
             char cD = dir;  //get current Direction
             if (cD == NORTH) {dir = SOUTH;}
             if (cD == SOUTH) {dir = NORTH;}
             if (cD == EAST) {dir = WEST;}
             if (cD == WEST) {dir = EAST;}
             int nxt = getNode(cN,dir);
             updateParameters(cN_p, nxt, dir_p);
             StackList <int> dropoff_path1 = pathFind(cN,4,dir); 
             StackList <int> dropoff_path2 = pathFind(cN,17,dir);
          }
          while(!fN.isEmpty()) {fN.pop();} //Clear the fN list 
          //Case 1: At node 17
          if(cN == 17){
            fN.push(4);
            checkpoint = 2; 
          }
          //Case 2: At node 4
          else if(cN == 4){
            fN.push(17);
            checkpoint = 0;
          }
          //Case 3: Closer to node 4
          else if(dropoff_path1.count() < dropoff_path2.count()){
              while(!dropoff_path1.isEmpty()){fN.push(dropoff_path1.pop());} 
              fN.push(17); //Direction to face
              checkpoint = 2; 
          }
          //Case 4: Closer to node 17
          else if(dropoff_path2.count() < dropoff_path1.count()){
              while(!dropoff_path2.isEmpty()){fN.push(dropoff_path2.pop());} 
              fN.push(4); //Direction to face
              checkpoint = 0; 
          }
       } 
       // If we have a passenger at the end point, drop it off
       if ((passenger == true) && (cN == 4 || cN == 17)){ 
         armDropOff();
       }  

       // If we don't have a passenger, either we have a collision -> reroute, or we just need a new route -> determine route
       motor.stop(LEFT_MOTOR);
       motor.stop(RIGHT_MOTOR);
       if (collision == true  && smallreverse == false){
         StackList <int> path = pathFind(cN,checkpointNodes[checkpoint],dir); 
         while(!path.isEmpty()){fN.push(path.pop());}  
         collision = false; //NOTE: If path is empty, we will return to this fN.isEmpty() immediately, and move onto the next checkpoint in the below else if{}
       }
       else if (collision == true && smallreverse == true){
         //Turn left -> figure out which line we will be on. Get reverse node -> find what is "left" of it

         //StackList <int> path = pathFind_noFwd(cN,checkpointNodes[checkpoint],dir); 
         //while(!path.isEmpty()){fN.push(path.pop());}  
         smallreverse = false;
         collision = false;
       }
       else if (collision == false){
         checkpoint = (checkpoint+1) % NUMCHECKPOINTS; // Cycle checkpoints
         StackList <int> path = pathFind(cN,checkpointNodes[checkpoint],dir); 
         while(!path.isEmpty()){fN.push(path.pop());}  
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

/*
  Function: nav_Collision

  Description:
  This function handles collisions for the navigation tab.
  It will reverse the robot, update the direction, set
  the collision variable to be true, and reset the turn
  direction.
*/
void nav_Collision(){
  while(!fN.isEmpty()) {fN.pop();} //Clear the fN list
  if (millis() - timeLastIntersection > SMALLREVERSETIME){
    turn(BACKWARD);
    char cD = dir;  //get current Direction
    if (cD == NORTH) {dir = SOUTH;}
    if (cD == SOUTH) {dir = NORTH;}
    if (cD == EAST) {dir = WEST;}
    if (cD == WEST) {dir = EAST;}
    int nxt = getNode(cN,dir);
    updateParameters(cN_p, nxt, dir_p); // Pointing opposite direction as previous, at the old node
  }
  else{
    smallreverse = true;
    reverse();
    char rD = dir;  //get reverse Direction
    if (dir == NORTH) {rD = SOUTH;}
    if (dir == SOUTH) {rD = NORTH;}
    if (dir == EAST) {rD = WEST;}
    if (dir == WEST) {rD = EAST;}
    int prev = getNode(cN,rD);
    updateParameters(cN_p, prev, dir_p);  // Pointing same direction as previous, at the old node
  }
  turnDir = UNDEFINED; //Reset turn
  collision = true;
}

/*
  Function: nav_Intersection

  Description:
  This function handles intersection for the navigation tab.
  It will update the time since the last intersection variable,
  turn the robot, update the robot's direction parameters, and
  reset the next turn direction.
*/
void nav_Intersection(){
  timeLastIntersection = millis(); // Update turn
  updateParameters(cN_p, fN.pop(), dir_p); // Account for the new position at the intersection.
  turn(turnDir); 
  turnDir = UNDEFINED; // Clear the turn direction
}

/*
  Function: armPickup

  Description:
  This function is to be called when the robot's arm is to aquire a passenger.
  It will stop the vehicle, decide the direction to pick up the passenger, set
  passenger = true, and clear the future nodes.
*/
void armPickup(){
  LCD.clear(); LCD.setCursor(0,0); LCD.print("L IR:"); LCD.print(left_ir*5.0/1024.0); LCD.setCursor(0,1); LCD.print("R IR:"); LCD.print(right_ir*5.0/1024.0);
  motor.speed(LEFT_MOTOR,0);
  motor.speed(RIGHT_MOTOR,0);
  delay(2000);
      
  if((left_ir*5.0/1024.0) > IR_THRESH){
    //Base rotation - Find the direction of the strongest IR signal
    LCD.clear(); LCD.print("Rotation"); delay(1000);
    RCServo0.write(160);
    //Arm height - lower arm to height of passenger
    LCD.clear(); LCD.print("Lowering arm"); delay(1000);
    RCServo1.write(30);
    //Claw closing
    LCD.clear(); LCD.print("Closing claw"); delay(1000);
    RCServo2.write(52);
    //Microswitch detection of passenger
    if(digitalRead(detectionPin_passenger1) == LOW || digitalRead(detectionPin_passenger2) == LOW){
      LCD.clear(); LCD.print("Passenger"); delay(1000);
    }
    //lift arm
    LCD.clear(); LCD.print("Lifting arm"); delay(1000); 
    RCServo1.write(180);
    //Rotate base
    LCD.clear(); LCD.print("Rotation"); delay(1000);
    RCServo0.write(90);   
   }
   else if((right_ir*5.0/1024.0) > IR_THRESH){
     //Base rotation - Find the direction of the strongest IR signal
     LCD.clear(); LCD.print("Rotation"); delay(1000);
     RCServo0.write(0);
     //Arm height - lower arm to height of passenger
     LCD.clear(); LCD.print("Lowering arm"); delay(1000);
     RCServo1.write(30); 
     //Claw closing
     LCD.clear(); LCD.print("Closing claw"); delay(1000);
     RCServo2.write(52);
     //Microswitch detection of passenger
     if(digitalRead(detectionPin_passenger1) == LOW || digitalRead(detectionPin_passenger2) == LOW){
       LCD.clear(); LCD.print("Passenger"); delay(1000);
     }
     //lift arm
     LCD.clear(); LCD.print("Lifting arm"); delay(1000);
     RCServo1.write(180);
     //Rotate base
     LCD.clear(); LCD.print("Rotation"); delay(1000);
     RCServo0.write(90);         
   }
   passenger = true;
   while(!fN.isEmpty()) {fN.pop();} //Clear the fN list
}

/*
  Function: armDropOff

  Description:
  This function is to be called when the robot is in the region between node 4 & 17
  ready for a drop-off to be made. It will automatically drive to the middle of the map
  using a time-delay and drop off the passenger, updating passenger = false.
*/
void armDropOff(){
  int ti = millis(); //Initial time
  int tf = millis(); //Final time
  while (tf-ti < 1500){ //Go forward for 1.5 seconds
    if(detectCollision()){nav_Collision(); break;} //Deal with collisions
    followTape();
    tf = millis();
  }
  motor.speed(LEFT_MOTOR,0);
  motor.speed(RIGHT_MOTOR,0);
  passenger = false;

  if(dir == EAST){
    //Rotate base
    LCD.clear(); LCD.print("Rotation"); delay(1000);
    RCServo0.write(0);
    //Lower arm
    LCD.clear(); LCD.print("Lowering Arm"); delay(1000);
    RCServo1.write(30);
    //open claw
    LCD.clear(); LCD.print("Opening claw"); delay(1000);
    RCServo2.write(0);
    delay(500);
    //reset
    LCD.clear(); LCD.print("Reset"); delay(1000);
    RCServo1.write(180); //Arm starts up
    delay(1000);
    RCServo0.write(90); //Position of the base starts in the middle
    RCServo2.write(0); //Claw starts open
  }

  if(dir == WEST){
    //Rotate base
    LCD.clear(); LCD.print("Rotation"); delay(1000);
    RCServo0.write(150);
    //Lower arm
    LCD.clear(); LCD.print("Lowering Arm"); delay(1000);
    RCServo1.write(30);
    //open claw
    LCD.clear(); LCD.print("Opening claw"); delay(1000);
    RCServo2.write(0);
    delay(500);
    //reset
    LCD.clear(); LCD.print("Reset"); delay(1000);
    RCServo1.write(180); //Arm starts up
    delay(1000);
    RCServo0.write(90); //Position of the base starts in the middle
    RCServo2.write(0); //Claw starts open
  } 
}


/*
  Function: nav_Lost

  Description:
  Figure out where we are. This involves the turn rules of:
    i) See if you are at a LFR intersection
    ii) If not, turn Right if possible, if not, then turn left
    iii) If at a LFR intersection, set a variable to be true, then try all paths until
        a non-LR node is found. If it is LF then we are at 6. If it is FR, we are at 14
    iv) Set the checkpoint to be such that we either go to 6 -> 7 or 14 -> 16
  This function initially clears all fN, cN, dir, turnDir, then 
  updates their values when a node is found. A new checkpoint is
  also chosen.
*/
void nav_Lost(int intsct){
  bool found = false; // Are we still lost?
  bool fourInt = false; // Have we found the four-way intersection?

  //Clear variables
  dir = UNDEFINED;
  cN = 999;
  while(!fN.isEmpty()) {fN.pop();}
  turnDir = UNDEFINED; 

  LCD.clear(); LCD.setCursor(0,0); LCD.print("LOST");
  delay(1000);

  while (found == false){
    if (intsct != 999 && fourInt == false){
      if (intsct == LFR_i) {LCD.clear(); LCD.setCursor(0,0); LCD.print("LFR I"); delay(2000); fourInt = true; turn(RIGHT);}
      else if (intsct == FR_i || intsct == LR_i || intsct == R_i) {LCD.clear(); LCD.setCursor(0,0); LCD.print("LR FR R"); delay(2000); turn(RIGHT);}
      else if (intsct == LF_i || intsct == L_i) {LCD.clear(); LCD.setCursor(0,0); LCD.print("LF L"); delay(2000); turn(LEFT);}
    }
    else if (intsct != 999 && fourInt == true){
      if (intsct == LR_i){ //Turn around and try again
        motor.speed(LEFT_MOTOR,-vel); motor.speed(RIGHT_MOTOR,-vel);
        delay(500);
        motor.speed(LEFT_MOTOR,0); motor.speed(RIGHT_MOTOR,0);
        LCD.clear(); LCD.setCursor(0,0); LCD.print("LR I"); delay(2000);
        turn(BACKWARD);
      }
      if (intsct == LFR_i) {LCD.clear(); LCD.setCursor(0,0); LCD.print("LFR I"); delay(2000); turn(RIGHT);} //We just arrived from reverse
      if (intsct == LF_i) {
        turn(LEFT);
        cN = 7;
        dir = WEST;
        checkpoint = 1;
        found = true;
        LCD.clear(); LCD.home(); LCD.print("FOUND 7"); 
      }
      if (intsct == FR_i) {
        turn(RIGHT);
        cN = 16;
        dir = EAST;
        checkpoint = 3;
        found = true;
        LCD.clear(); LCD.home(); LCD.print("FOUND 16"); 
      }
    }
    else{
      if(fourInt == true){LCD.clear(); LCD.setCursor(0,0); LCD.print("T");}
      else{LCD.clear(); LCD.setCursor(0,0); LCD.print("F");}
      
      if(detectCollision()){
        turn(BACKWARD);
      } else {
        followTape();
      }
    }
    intsct = detectValidPaths(); // What type of intersection are we at
  }
}
