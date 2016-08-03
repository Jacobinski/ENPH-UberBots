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
#define SMALLREVERSETIME 1000 //Threshold for straight reverse vs. U turn 

int checkpoint; // Checkpoints on the map that the robot wishes to reach
int counter; // Displaying driving stats
int timeLastIntersection; // Time at last intersection
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
  delay(1000);
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
      timeLastIntersection = millis(); // Update turn
      updateParameters(cN_p, fN.pop(), dir_p); // Account for the new position at the intersection.
      turn(turnDir); 
      turnDir = UNDEFINED; // Clear the turn direction
    }
    // Detect passenger
    double left_ir = analogRead(LEFT_IR);
    double right_ir = analogRead(RIGHT_IR);
    if (((left_ir*5.0/1024.0) > 1.0 || (right_ir*5.0/1024.0) > 1.0)){
      for(int i=0; i<4;i++){
        left_ir = left_ir + analogRead(LEFT_IR);
        right_ir = right_ir + analogRead(RIGHT_IR);
      }
    }
    if (((left_ir*5.0/1024.0) > IR_THRESH || (right_ir*5.0/1024.0) > IR_THRESH) && passenger == false){
      LCD.clear();
      LCD.setCursor(0,0);
      LCD.print("L IR:");
      LCD.print(left_ir*5.0/1024.0);
      LCD.setCursor(0,1);
      LCD.print("R IR:");
      LCD.print(right_ir*5.0/1024.0);
      motor.speed(LEFT_MOTOR,0);
      motor.speed(RIGHT_MOTOR,0);

      useArm(left_ir,right_ir);
      passenger = true;
      
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
        LCD.clear();
        LCD.home();
        LCD.print("Case 1");
        delay(2000);
        fN.push(4);
        checkpoint = 3; 
      }
      //Case 2: At node 4
      else if(cN == 4){
        LCD.clear();
        LCD.home();
        LCD.print("Case 2");
        delay(2000);
        fN.push(17);
        checkpoint = 0;
      }
      //Case 3: Closer to node 4
      else if(dropoff_path1.count() < dropoff_path2.count()){
          while(!dropoff_path1.isEmpty()){LCD.clear();LCD.setCursor(0,0);LCD.print(dropoff_path1.peek());fN.push(dropoff_path1.pop());delay(500);} 
          LCD.clear();
          LCD.home();
          LCD.print("Case 3");
          delay(2000);
          fN.push(17); //Direction to face
          checkpoint = 2; 
      }
      //Case 4: Closer to node 17
      else if(dropoff_path2.count() < dropoff_path1.count()){
          while(!dropoff_path2.isEmpty()){LCD.clear();LCD.setCursor(0,0);LCD.print(dropoff_path2.peek());fN.push(dropoff_path2.pop());delay(500);} 
          LCD.clear();
          LCD.home();
          LCD.print("Case 4");
          delay(2000);
          fN.push(4); //Direction to face
          checkpoint = 0; 
      }
    }
    // Default
    else{
      followTape();
      counter = counter + 1;
      /*if (counter == 20){
        LCD.clear();
        LCD.setCursor(0,0);
        LCD.print("L IR:");
        LCD.print(analogRead(LEFT_IR)*5.0/1024.0);
        LCD.setCursor(0,1);
        LCD.print("R IR:");
        LCD.print(analogRead(RIGHT_IR)*5.0/1024.0);
        counter = 0;
      }*/
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
          while(!detectCollision()){
            followTape();
            double left_ir = analogRead(LEFT_IR);
            double right_ir = analogRead(RIGHT_IR); 
            if (((left_ir*5.0/1024.0) > 1.0 || (right_ir*5.0/1024.0) > 1.0)){
              for(int i=0; i<4;i++){
                left_ir = left_ir + analogRead(LEFT_IR);
                right_ir = right_ir + analogRead(RIGHT_IR);
              }
            }
            if (((left_ir*5.0/1024.0) > IR_THRESH || (right_ir*5.0/1024.0) > IR_THRESH) && passenger == false){
              LCD.clear();
              LCD.setCursor(0,0);
              LCD.print("L IR:");
              LCD.print(left_ir*5.0/1024.0);
              LCD.setCursor(0,1);
              LCD.print("R IR:");
              LCD.print(right_ir*5.0/1024.0);
              motor.speed(LEFT_MOTOR,0);
              motor.speed(RIGHT_MOTOR,0);
              useArm(left_ir,right_ir);
              //passenger = true; MOVING THIS IN TO FUNCTION
            }
          }
          if(detectCollision()){
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
          if(passenger == true){
              StackList <int> dropoff_path1 = pathFind(cN,4,dir); 
              StackList <int> dropoff_path2 = pathFind(cN,17,dir);
              turnDir = UNDEFINED; //Reset turn
              while(!fN.isEmpty()) {fN.pop();} //Clear the fN list 
              //Case 1: Closer to node 4
              if(dropoff_path1.count() < dropoff_path2.count()){
                  while(!dropoff_path1.isEmpty()){LCD.clear();LCD.setCursor(0,0);LCD.print(dropoff_path1.peek());fN.push(dropoff_path1.pop());delay(500);} 
                  LCD.clear();
                  LCD.home();
                  LCD.print("Case 3");
                  delay(2000);
                  fN.push(17); //Direction to face
                  checkpoint = 2; 
              }
              //Case 2: Closer to node 17
              else if(dropoff_path2.count() < dropoff_path1.count()){
                  while(!dropoff_path2.isEmpty()){LCD.clear();LCD.setCursor(0,0);LCD.print(dropoff_path2.peek());fN.push(dropoff_path2.pop());delay(500);} 
                  LCD.clear();
                  LCD.home();
                  LCD.print("Case 4");
                  delay(2000);
                  fN.push(4); //Direction to face
                  checkpoint = 0; 
              }
          }
       }  
       
       // If we have a passenger, drop it off
       else if (passenger == true){ 
          int ti = millis(); //Initial time
          int tf = millis(); //Final time
          while (tf-ti < 1500){ //Go forward for 1.5 seconds
            followTape();
            tf = millis();
          }
          motor.speed(LEFT_MOTOR,0);
          motor.speed(RIGHT_MOTOR,0);
          passenger = false;
          
          //DROP OFF PASSENGER - Jenny-------------------------------------------------------------------

          if(dir == EAST){
            //Rotate base
            LCD.clear();
            LCD.print("Rotation");
            delay(1000);
            RCServo0.write(0);

            //Lower arm
            LCD.clear();
            LCD.print("Lowering Arm");
            delay(1000);
            RCServo1.write(30);
          
            //open claw
            LCD.clear();
            LCD.print("Opening claw");
            delay(1000);
            RCServo2.write(0);
            delay(500);

            //reset
            LCD.clear();
            LCD.print("Reset");
            delay(1000);
            RCServo1.write(180); //Arm starts up
            delay(1000);
            RCServo0.write(90); //Position of the base starts in the middle
            RCServo2.write(0); //Claw starts open
          }

          if(dir == WEST){
            //Rotate base
            LCD.clear();
            LCD.print("Rotation");
            delay(1000);
            RCServo0.write(160);

            //Lower arm
            LCD.clear();
            LCD.print("Lowering Arm");
            delay(1000);
            RCServo1.write(30);
          
            //open claw
            LCD.clear();
            LCD.print("Opening claw");
            delay(1000);
            RCServo2.write(0);
            delay(500);

            //reset
            LCD.clear();
            LCD.print("Reset");
            delay(1000);
            RCServo1.write(180); //Arm starts up
            delay(1000);
            RCServo0.write(90); //Position of the base starts in the middle
            RCServo2.write(0); //Claw starts open
          }
        
          //-------------------------------------------------------------------------------------------  
       }
        motor.stop(LEFT_MOTOR);
        motor.stop(RIGHT_MOTOR);
        if (collision == true){
          StackList <int> path = pathFind(cN,checkpointNodes[checkpoint],dir); 
          //while(!path.isEmpty()){LCD.clear();LCD.setCursor(0,0);LCD.print(path.peek());fN.push(path.pop());delay(500);}
          while(!path.isEmpty()){fN.push(path.pop());}  
          collision = false; //NOTE: If path is empty, we will return to this fN.isEmpty() immediately, and move onto the next checkpoint in the below else if{}
        }
        else if (collision == false){
          checkpoint = (checkpoint+1) % NUMCHECKPOINTS; // Cycle checkpoints
          StackList <int> path = pathFind(cN,checkpointNodes[checkpoint],dir); 
          //while(!path.isEmpty()){LCD.clear();LCD.setCursor(0,0);LCD.print(path.peek());fN.push(path.pop());delay(500);} 
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

void useArm(double left_ir, double right_ir){
   // --------------------------------------------------------------------- Jenny's arm function --------------------------------------------------------------------------------------------
        int tryNumber = 0;
        
        if((left_ir*5.0/1024.0) > IR_THRESH){
          //Base rotation:
          //rotates towards the direction of the strongest IR signal
          LCD.clear(); LCD.print("Rotation"); delay(1000);
          RCServo0.write(160);
          //Arm height adjustment:
          //lower arm to height of passenger
          LCD.clear(); LCD.print("Lowering arm"); delay(1000);
          RCServo1.write(30);
        
          for (tryNumber = 1; tryNumber < 4; tryNumber++) {
            if (tryNumber == 2) {
              RCServo1.write(0); //lift arm
              delay(1000);
              RCServo2.write(50); //open claw
              //try readjusting left first
              LCD.clear(); LCD.print("Retrying left");
              //Base rotation:
              //rotate base a bit left
              RCServo0.write(160 + 15);
              //Arm height adjustment:
              //lower arm to height of passenger
              RCServo1.write(30);
            }
            if (tryNumber == 3) {
              RCServo1.write(0); //lift arm
              delay(1000);
              RCServo2.write(50); //open claw
              //try readjusting left first
              LCD.clear(); LCD.print("Retrying right");
              //Base rotation:
              //rotate base a bit right
              RCServo0.write(160 - 15);
              //Arm height adjustment:
              //lower arm to height of passenger
              RCServo1.write(30);
            }

            //Claw actuation:
            //claw closes around the passenger
            LCD.clear(); LCD.print("Closing claw"); delay(1000);
            RCServo2.write(52);

            //Microswitch detection:
            //microswitch detects whether or not a passenger has been obtained
            if (digitalRead(DETECTION_SWITCH) == HIGH) {
              LCD.clear(); LCD.print("Passenger"); delay(1000);
              //Arm height adjustment:
              //lift arm (with passenger)
              LCD.clear(); LCD.print("Lifting arm"); delay(1000);
              RCServo1.write(180);
              //Base rotation:
              //rotate base towards the middle
              LCD.clear(); LCD.print("Rotation"); delay(1000);
              RCServo0.write(90);
              passenger = true;
              tryNumber = 4;
            }
          }
              
       }

        if((right_ir*5.0/1024.0) > IR_THRESH){
          //Base rotation:
          //rotates towards the direction of the strongest IR signal
          LCD.clear(); LCD.print("Rotation"); delay(1000);
          RCServo0.write(5);
          //Arm height adjustment:
          //lower arm to height of passenger
          LCD.clear(); LCD.print("Lowering arm"); delay(1000);
          RCServo1.write(30);
        
          for (tryNumber = 1; tryNumber < 4; tryNumber++) {
            if (tryNumber == 2) {
              RCServo1.write(0); //lift arm
              delay(1000);
              RCServo2.write(50); //open claw
              //try readjusting left first
              LCD.clear(); LCD.print("Retrying left");
              //Base rotation:
              //rotate base a bit left
              RCServo0.write(15 + 15);
              //Arm height adjustment:
              //lower arm to height of passenger
              RCServo1.write(30);
            }
            if (tryNumber == 3) {
              RCServo1.write(0); //lift arm
              delay(1000);
              RCServo2.write(50); //open claw
              //try readjusting left first
              LCD.clear(); LCD.print("Retrying right");
              //Base rotation:
              //rotate base a bit right
              RCServo0.write(15 - 15);
              //Arm height adjustment:
              //lower arm to height of passenger
              RCServo1.write(30);
            }

            //Claw actuation:
            //claw closes around the passenger
            LCD.clear(); LCD.print("Closing claw"); delay(1000);
            RCServo2.write(52);

            //Microswitch detection:
            //microswitch detects whether or not a passenger has been obtained
            if (digitalRead(DETECTION_SWITCH) == HIGH) {
              LCD.clear(); LCD.print("Passenger"); delay(1000);
              //Arm height adjustment:
              //lift arm (with passenger)
              LCD.clear(); LCD.print("Lifting arm"); delay(1000);
              RCServo1.write(180);
              //Base rotation:
              //rotate base towards the middle
              LCD.clear(); LCD.print("Rotation"); delay(1000);
              RCServo0.write(90);
              tryNumber = 4;
            }
          }         
       }
      // --------------------------------------------------------------------- END ------------------------------------------------------------------------------------------------------------
      
}

