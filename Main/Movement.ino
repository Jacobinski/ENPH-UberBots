/*
  Title:
  Movement

  Description:
  This tab is responsible for the interface between code and movement. It will 
  include functions that allow for PID tape following and turning of the robot.
. 
  Arduino Input:
    * Analog Pin 3: Right QRD for forward tape following.
    * Analog Pin 5: Left QRD for forward tape following.
    * Knob 6: Derivative gain control.
    * Knob 6: Proportional gain control.
  Arduino Output:
    * Motor 0: Right drive train motor.
    * Motor 1: Left drive train motor.
    
  Created: June 30, 2016
    By: Jacob Budzis
  Modified: June 30, 2016
    By: Jacob Budzis
*/

//These definitions determine the intersection paths available
#define LFR_i 1 
#define FR_i 2
#define LF_i 3
#define LR_i 4
#define R_i 5
#define L_i 6

int lmthresh = 40; //Threshold before left QRD detects paper 
int rmthresh = 40; //Threshold before right QRD detects paper
int lithresh = 100;
int rithresh = 100;
double error = 0; //Current Error
double lasterr = 0; //Previous Error (i.e One time step ago)
double recerr = 0; //Error in Last Tape State (i.e. was on -1, now is on 0)
double q = 0; //#Clock pulses in last state, relates to recerr
double m = 0; //#Clock oulses in current state, relating to error
double d; //PID Derivative. m = y/x = (error-lasterr)/(q+m)
double p; //PID Proportional. 
double con; //Control Applied = kd*d + kp*p;
double vel = 80; // Current to motor
int t = 0; //Counter

/*
  Function: followTape

  Description:
  This function uses PID control to allow the robot to follow the tape. This will 
  only allow for straight-line following.

  Code Inputs:
    * None
  Code Outputs:
    * None
  TINAH Inputs:
    * knob 6: Derivative gain
    * knob 7: Proportional gain
    * analog 3: Right tape QRD
    * analog 5: Left tape QRD
*/
void followTape(){ 

  int kd = 60;//knob(DERIVATIVE)/4; //Derivative Gain Multiplier 
  int kp = 20;//knob(PROPORTIONAL)/4; //Proportional Gain Multiplier
  int left = analogRead(LEFT_TAPE); //Left QRD Signal
  int right = analogRead(RIGHT_TAPE); //Right QRD Signal

   if ((left>lmthresh)&&(right>rmthresh)) error = 0;
   if ((left>lmthresh)&&(right<rmthresh)) error = -1;
   if ((left<lmthresh)&&(right>rmthresh)) error = +1;
   if ((left<lmthresh)&&(right<rmthresh))
   {
     if (lasterr>0) error = 5;
     if (lasterr<=0) error = -5;
   }
   if (!(error==lasterr))
   {
     recerr=lasterr;
     q=m;
     m=1;
   }
  
   p=kp*error;
   d=(int)((float)kd*(float)(error-recerr)/(float)(q+m));
   con = p+d;
   m=m+1;
   motor.speed(LEFT_MOTOR,vel+con); //left
   motor.speed(RIGHT_MOTOR,vel-con); //right
   lasterr=error;
}


/*
  Function: detectIntersection

  Description:
  This function returns if an intersection was detected. The direction
  of the intersection to be detected is passed to this function.

  Code Inputs:
    * dir: Character corresponding to which side QRD must detect
           the intersection. If F (forward) is passed to this function,
           it will return true if EITHER of the QRDs detect something
  Code Outputs:
    * Boolean value if interesection detected.
  TINAH Inputs:
    * digital 0: Left QRD intersection
    * digital 1: Right QRD intersection
*/
bool detectIntersection(char dir){

  bool output = false;
  int left = analogRead(LEFT_INTERSECTION);
  int right = analogRead(RIGHT_INTERSECTION);

  if (dir == LEFT){
    if (left > lithresh) output = true;
  }
  else if (dir == RIGHT){
    if (right > rithresh) output = true;
  }
  else if (dir == FORWARD){
    if (left > lithresh || right > rithresh) output = true;
  }
  else if (dir == BACKWARD){
    if (left > lithresh || right > rithresh) output = true;
  }

  return output;
}

/*
  Function: detectValidPaths

  Description:
  Rotates the robot corresponding to the turn direction given to the 
  function. This occurs by moving the wheels in opposite directions.

  Definitions:
    #define LFR 1  Left, Forward, Right
    #define FR 2   Foward, Right
    #define LF 3   Left, Forward
    #define LR 4   Left, Right
    #define R 5    Right
    #define L 6    Left
*/
int detectValidPaths(){
  bool L_path = false;
  bool R_path = false;
  bool F_path = false;
  int output = 999; // 999 is the continue on value
  int t = 150; // Overshoot time
  int corr = 0;
  
  if(analogRead(LEFT_INTERSECTION) > lithresh){L_path = true; corr -= 2;}  
  if(analogRead(RIGHT_INTERSECTION) > rithresh){R_path = true; corr += 2; }

  if(L_path == true || R_path == true){
   int ti = millis(); //Initial time
   int tf = millis(); //Final time
   motor.speed(LEFT_MOTOR,vel+corr); //left
   motor.speed(RIGHT_MOTOR,vel-corr); //right
   while(analogRead(LEFT_INTERSECTION) > lithresh || analogRead(RIGHT_INTERSECTION) > rithresh){
      // Keep on overshooting the intersection
   }
   while(tf-ti < t){
      if(L_path == false){if(analogRead(LEFT_INTERSECTION) > lithresh){L_path = true;}}
      if(R_path == false){if(analogRead(RIGHT_INTERSECTION) > rithresh){R_path = true;}}
      if(analogRead(LEFT_TAPE) > lmthresh || analogRead(RIGHT_TAPE) > rmthresh){
        followTape();
      }
      else{
        motor.speed(LEFT_MOTOR,vel+corr); //left
        motor.speed(RIGHT_MOTOR,vel-corr); //right
      }
      tf = millis(); 
    }
    motor.speed(LEFT_MOTOR,0); //left
    motor.speed(RIGHT_MOTOR,0); //right
    //Try to update variables one more time
    if(analogRead(LEFT_TAPE) > lmthresh || analogRead(RIGHT_TAPE) > rmthresh){F_path = true;}
    if(analogRead(LEFT_INTERSECTION) > lithresh){L_path = true;}
    if(analogRead(RIGHT_INTERSECTION) > rithresh){R_path = true;}
  }

  if(L_path == true && F_path == true && R_path == true) {output = 1;}
  if(L_path == false && F_path == true && R_path == true) {output = 2;}
  if(L_path == true && F_path == true && R_path == false) {output = 3;}
  if(L_path == true && F_path == false && R_path == true) {output = 4;}
  if(L_path == false && F_path == false && R_path == true) {output = 5;}
  if(L_path == true && F_path == false && R_path == false) {output = 6;}

  return output;
}

/*
  Function: turn

  Description:
  Rotates the robot corresponding to the turn direction given to the 
  function. This occurs by moving the wheels in opposite directions.

  Code Inputs:
    * Direction: (char) LEFT, RIGHT, FOWARD, BACKWARD
  Code Outputs:
    * None
  TINAH Inputs:
    * Motor : Right Motor
    * Motor : Left Motor
*/
void turn(char dir){

   int L = 0; //Left middle QRD Signal
   int R = 0; //Right middle QRD Signal
   int ti; //Initial turn time
   int tf; //Final turn time
   int turnTime = 600; //ms
   delay(50);

   if (dir == LEFT){
      motor.speed(LEFT_MOTOR,-vel); //left
      motor.speed(RIGHT_MOTOR,vel); //right
      delay(200);
      ti = millis(); //Initial time
      tf = millis(); //Final time
      while(L < lmthresh){;
        L = analogRead(LEFT_TAPE);
        tf = millis(); //Final time
        if(tf-ti>3000){  //Fix the turn
          motor.speed(LEFT_MOTOR,vel); //left
          motor.speed(RIGHT_MOTOR,-vel); //right
        }
      }
      lasterr = 0; //Reset PID
      ti = millis(); //Initial time
      tf = millis(); //Final time
      while(tf-ti < turnTime){
          followTape();
          tf = millis(); //Final time
      }
   } 
   else if (dir == RIGHT){
      motor.speed(LEFT_MOTOR,vel); //left
      motor.speed(RIGHT_MOTOR,-vel); //right
      delay(200); //Pause for 0.5s
      ti = millis(); //Initial time
      tf = millis(); //Final time
      while(R < rmthresh){
          R = analogRead(RIGHT_TAPE);
          tf = millis(); //Final time
          if(tf-ti>3000){  //Fix the turn
            motor.speed(LEFT_MOTOR,-vel); //left
            motor.speed(RIGHT_MOTOR,vel); //right
        }
      }
      //motor.stop_all();   
      lasterr = 0; //Reset PID
      ti = millis(); //Initial time
      tf = millis(); //Final time
      while(tf-ti < turnTime){
          followTape();
          tf = millis(); //Final time
      }   
   } 
   else if (dir == FORWARD){
      motor.speed(LEFT_MOTOR,vel+con); //left
      motor.speed(RIGHT_MOTOR,vel-con); //right
      delay(100); //Just pass the intersection
   }
   else if (dir == BACKWARD){
      int i = 0; // Turn counter
      int V = vel; // Velocity for turn
      bool stopTurn = false;

        //This is a left-hand reverse turn
        motor.speed(LEFT_MOTOR,-V);
        motor.speed(RIGHT_MOTOR,0);
        delay(700); //Reverse for 0.3 sec
        //MAYBE OVERSHOOT TO SEE IF NEAR AN INTERSECTION, THEN PULL FORWARD AND DO THE TURN
        while(stopTurn == false){
          if(i % 2 == 1){
              motor.speed(LEFT_MOTOR,-V);
              motor.speed(RIGHT_MOTOR,0);
              ti = millis(); //Initial time
              tf = millis(); //Final time
              while(tf-ti < 300){
                if ( analogRead(LEFT_TAPE) > lmthresh || analogRead(RIGHT_TAPE) > rmthresh) stopTurn = true;
                tf = millis(); //Final time
              }
          } else {
              motor.speed(LEFT_MOTOR,0);
              motor.speed(RIGHT_MOTOR,V);
              ti = millis(); //Initial time
              tf = millis(); //Final time
              while(tf-ti < 300){
                if ( analogRead(LEFT_TAPE) > lmthresh || analogRead(RIGHT_TAPE) > rmthresh) stopTurn = true;
                tf = millis(); //Final time
              }
          }
          i = i + 1;
          if(i == 20){  //Get unstuck
            motor.speed(LEFT_MOTOR,-V);
            motor.speed(RIGHT_MOTOR,-V);
            delay(100);
            i = 1;

          }
        }
        motor.speed(LEFT_MOTOR,0);
        motor.speed(RIGHT_MOTOR,0);
        lasterr = 5; //Set PID to compensate

      ti = millis(); //Initial time
      tf = millis(); //Final time
      while(tf-ti < 50){ //Accelerate
          followTape();
          tf = millis(); //Final time
      }
   }
}

/*
  Function: detectCollision

  Description:
  Detects if the front bumper hit an object. Returns a boolean.

  Code Inputs:
    * None
  Code Outputs:
    * True if collision, else falser
  TINAH Inputs:
    * Switch: Left collision sensor
    * Switch: Right collision sensor
*/
bool detectCollision(){
  bool output = false;
  int left = digitalRead(LEFT_FWD_COLLISION); 
  int right = digitalRead(RIGHT_FWD_COLLISION); 
  if(left == LOW || right == LOW) {output = true;}
  return output;
}


/*
  Function: reverse

  Description:
  Runs the robot straight backwards until an intersection is detected.

  Code Inputs:
    * detectIntersection() function
  Code Outputs:
    * None
  TINAH Inputs:
    * Motor : Right Motor
    * Motor : Left Motor

*/
void reverse(){
   motor.speed(LEFT_MOTOR,0);
   motor.speed(RIGHT_MOTOR,0);
   delay(50); //Pause briefly
   motor.speed(LEFT_MOTOR,-vel);
   motor.speed(RIGHT_MOTOR,-vel);
   while(!detectIntersection(FORWARD)){
      //Do nothing, just keep reversing
   }
   delay(100);
   motor.speed(LEFT_MOTOR,0); //Stop the motors again.
   motor.speed(RIGHT_MOTOR,0); 
}

