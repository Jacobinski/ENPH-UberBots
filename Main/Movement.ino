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

int lthresh = 30; //Threshold before left QRD detects paper 
int rthresh = 30; //Threshold before right QRD detects paper
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

  int kd = knob(DERIVATIVE)/4; //Derivative Gain Multiplier 
  int kp = knob(PROPORTIONAL)/4; //Proportional Gain Multiplier
  int left = analogRead(LEFT_TAPE); //Left QRD Signal
  int right = analogRead(RIGHT_TAPE); //Right QRD Signal

   if ((left>lthresh)&&(right>rthresh)) error = 0;
   if ((left>lthresh)&&(right<rthresh)) error = -1;
   if ((left<lthresh)&&(right>rthresh)) error = +1;
   if ((left<lthresh)&&(right<rthresh))
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
    if (left > lthresh) output = true;
  }
  else if (dir == RIGHT){
    if (right > rthresh) output = true;
  }
  else if (dir == FORWARD){
    if (left > lthresh || right > rthresh) output = true;
  }
  else if (dir == BACKWARD){
    if (left > lthresh || right > rthresh) output = true;
  }

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
      
   // Begin by going past the intersection. 
   // This will give us space to make a wider turn.

   if (dir == LEFT){
      delay(200); //Overshoot
      motor.speed(LEFT_MOTOR,-vel); //left
      motor.speed(RIGHT_MOTOR,vel); //right
      delay(400);
      ti = millis(); //Initial time
      tf = millis(); //Final time
      while(L < lthresh){;
        L = analogRead(LEFT_TAPE);
        tf = millis(); //Final time
        if(tf-ti>1000){  //Fix the turn
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
      delay(200); //Overshoot
      motor.speed(LEFT_MOTOR,vel); //left
      motor.speed(RIGHT_MOTOR,-vel); //right
      delay(400); //Pause for 0.5s
      ti = millis(); //Initial time
      tf = millis(); //Final time
      while(R < rthresh){
          R = analogRead(RIGHT_TAPE);
          tf = millis(); //Final time
          if(tf-ti>1000){  //Fix the turn
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
      delay(200); //Just pass the intersection
   }
   else if (dir == BACKWARD){
      int i = 0; // Turn counter
      int V = vel; // Velocity for turn
      bool stopTurn = false;

        motor.speed(LEFT_MOTOR,-V);
        motor.speed(RIGHT_MOTOR,0);
        delay(500); //Reverse for 0.3 sec
        //MAYBE OVERSHOOT TO SEE IF NEAR AN INTERSECTION, THEN PULL FORWARD AND DO THE TURN
        while(stopTurn == false){
          if(i % 2 == 1){
              motor.speed(LEFT_MOTOR,-V);
              motor.speed(RIGHT_MOTOR,0);
              ti = millis(); //Initial time
              tf = millis(); //Final time
              while(tf-ti < 300){
                if ( analogRead(LEFT_TAPE) > lthresh || analogRead(RIGHT_TAPE) > rthresh) stopTurn = true;
                tf = millis(); //Final time
              }
          } else {
              motor.speed(LEFT_MOTOR,0);
              motor.speed(RIGHT_MOTOR,V);
              ti = millis(); //Initial time
              tf = millis(); //Final time
              while(tf-ti < 300){
                if ( analogRead(LEFT_TAPE) > lthresh || analogRead(RIGHT_TAPE) > rthresh) stopTurn = true;
                tf = millis(); //Final time
              }
          }
          i = i + 1;
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
   motor.speed(LEFT_MOTOR,0); //Stop the motors again.
   motor.speed(RIGHT_MOTOR,0); 
}

