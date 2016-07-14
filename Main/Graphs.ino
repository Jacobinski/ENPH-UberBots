/*
  Title:
  Graphs

  Description:
  This tab holds navigation algorithms and code of the robot. These algorithms allow
  the robot to know where it is and where it needs to go. Most of the functions will
  take some input relating to current position and orientation (N,S,E,W) and output
  a sequence of characters representing the turning sequence of the robot.  

  Arduino Input:
    * None
  Arduino Output:
    * None
    
  Created: July 10, 2016
    By: Jacob Budzis
  Modified: July 10, 2016
    By: Jacob Budzis
*/

#define NODES 20  //Intersections
#define DIRECTIONS 4
#define NORTH 'N'
#define SOUTH 'S'
#define EAST 'E'
#define WEST 'W'
#define RIGHT 'R'
#define LEFT 'L'
#define FORWARD 'F'
#define UNDEFINED 'U'

const int distance[NODES][NODES] = //Non-zero values are valid connections
{  
  {0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, // Node 1
  {1,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, // Node 2
  {0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, // Node 3
  {0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0}, // Node 4
  {0,1,0,1,0,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0}, // Node 5
  {0,0,0,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0}, // Node 6
  {0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, // Node 7
  {0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, // Node 8
  {0,0,0,0,1,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0}, // Node 9
  {0,0,0,0,0,0,0,0,1,0,1,1,0,0,0,0,0,0,0,0}, // Node 10
  {0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0}, // Node 11
  {0,0,0,0,0,0,0,0,1,1,0,0,1,0,0,0,0,0,0,0}, // Node 12
  {0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,1,0,1,0}, // Node 13
  {0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,1,0,0,0,0}, // Node 14
  {0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0}, // Node 15
  {0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0}, // Node 16
  {0,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,0}, // Node 17
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0}, // Node 18
  {0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,1}, // Node 19
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0}  // Node 20
};
const char validDirections[NODES][DIRECTIONS] =
{
  //NORTH, EAST, SOUTH, WEST
  {'U','U','S','U'}, // Node 1
  {'N','E','S','U'}, // Node 2
  {'N','E','U','U'}, // Node 3
  {'N','E','U','W'}, // Node 4
  {'N','E','S','W'}, // Node 5
  {'N','U','S','W'}, // Node 6
  {'U','E','U','U'}, // Node 7
  {'U','U','S','U'}, // Node 8
  {'N','U','S','W'}, // Node 9
  {'N','E','U','W'}, // Node 10
  {'U','U','S','U'}, // Node 11
  {'N','E','S','U'}, // Node 12
  {'N','E','S','W'}, // Node 13
  {'N','E','S','U'}, // Node 14
  {'U','U','S','U'}, // Node 15
  {'U','U','U','W'}, // Node 16
  {'N','E','U','W'}, // Node 17
  {'N','U','U','W'}, // Node 18
  {'N','U','S','W'}, // Node 19
  {'U','U','S','U'}  // Node 20
};
const char nodeDirections[NODES][NODES] = //Defined from edges LEAVING the [x][] position
{  
  {'U','S','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'}, // Node 1
  {'N','U','S','U','E','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'}, // Node 2
  {'U','N','U','E','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'}, // Node 3
  {'U','U','W','U','N','U','U','U','U','U','U','U','U','U','U','U','E','U','U','U'}, // Node 4
  {'U','W','U','S','U','N','U','U','E','U','U','U','U','U','U','U','U','U','U','U'}, // Node 5
  {'U','U','U','U','S','U','W','N','U','U','U','U','U','U','U','U','U','U','U','U'}, // Node 6
  {'U','U','U','U','U','E','U','U','U','U','U','U','U','U','U','U','U','U','U','U'}, // Node 7
  {'U','U','U','U','U','S','U','U','U','U','U','U','U','U','U','U','U','U','U','U'}, // Node 8
  {'U','U','U','U','W','U','U','U','U','N','U','S','U','U','U','U','U','U','U','U'}, // Node 9
  {'U','U','U','U','U','U','U','U','W','U','N','E','U','U','U','U','U','U','U','U'}, // Node 10
  {'U','U','U','U','U','U','U','U','U','S','U','U','U','U','U','U','U','U','U','U'}, // Node 11
  {'U','U','U','U','U','U','U','U','S','N','U','U','E','U','U','U','U','U','U','U'}, // Node 12
  {'U','U','U','U','U','U','U','U','U','U','U','W','U','N','U','U','S','U','E','U'}, // Node 13
  {'U','U','U','U','U','U','U','U','U','U','U','U','S','U','N','E','U','U','U','U'}, // Node 14
  {'U','U','U','U','U','U','U','U','U','U','U','U','U','S','U','U','U','U','U','U'}, // Node 15
  {'U','U','U','U','U','U','U','U','U','U','U','U','U','W','U','U','U','U','U','U'}, // Node 16
  {'U','U','U','W','U','U','U','U','U','U','U','U','N','U','U','U','U','E','U','U'}, // Node 17
  {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','W','U','N','U'}, // Node 18
  {'U','U','U','U','U','U','U','U','U','U','U','U','W','U','U','U','U','S','U','N'}, // Node 19
  {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','S','U'}  // Node 20
};

/*
  Function: getDirection

  Description:
  Returns the direction to given the current node (cN) and the
  next node (nN). If no values are given returns a value of U to
  signfify an undefined output 

  Code Inputs:
    * cN: (Int) Current node of the robot
    * fN: (Int) Future node of the robot
  Code Outputs:
    * dir: (char) Direction to turn to get to new node.
*/
char getDirection(int cN, int fN){

  char dir = UNDEFINED;
  if (cN <= 20 && cN > 0 && fN <= 20 && fN > 0){
    dir = nodeDirections[cN-1][fN-1];
   } else {
    motor.stop_all();
    LCD.clear();
    LCD.home();
    LCD.println("Error in getDirection");
    LCD.print("cN:");  LCD.print(cN);
    LCD.print(" nN:"); LCD.print(fN);
    delay(100000); // 100 seconds
  }
  return dir;
}

/*
  Function: getIntersectionDirections

  Description:
  Returns the allowed directions (N,E,S,W) in the form of a pointer (array
  of 4 values) corresponding to the given node. The values in the array will
  be as follows
    - [1] north (N) or undefined (U)
    - [2] east (E) or undefined (U)
    - [3] south (S) or undefined (U)
    - [4] west (W) or undefined (U)
  A node value that is undefined will return [U,U,U,U]

  Code Inputs:
    * node: (int) Node for which to get directions
  Code Outputs:
    * dir_p: (char*) An array of 4 values corresponding to the description's 
                     array values.
*/
char *getIntersectionDirections(int node){

  char *turn = (char*)malloc(4);
  for(int i = 0; i < 4; i++){
    if (node > 20 || node < 1)
      turn[i] = UNDEFINED;
    else{
      turn[i] = validDirections[node][i];
    }
  }
  return turn;
}

/*
  Function: turnDirection

  Description:
  This function returns the direction to move turn the robot from the
  current node to the future node. It is passed the future node, as well 
  as pointers to the current direction and current node. This function
  will automatically update the values of the current node & current
  direction to reflect the turn direction.
  The output will either be forward, left, right (F,L,R).

  Code Inputs:
    * *cN: The current node pointer
    * fN: The future node
    * *dir: The current direction pointer of the robot
  Code Outputs:
    * Character (F,L,R)
*/
char turnDirection(int *cN, int fN, char *dir){

  char p = *dir; // Previous Direction
  char n = getDirection(*cN,fN); // Next Direction
  char turn = UNDEFINED;

  if (p == NORTH && n == NORTH)  {turn = FORWARD;}
  if (p == NORTH && n == EAST)   {turn = RIGHT;}
  if (p == NORTH && n == WEST)   {turn = LEFT;}
  if (p == SOUTH && n == SOUTH)  {turn = FORWARD;}
  if (p == SOUTH && n == EAST)   {turn = LEFT;}
  if (p == SOUTH && n == WEST)   {turn = RIGHT;}
  if (p == WEST && n == NORTH)   {turn = RIGHT;}
  if (p == WEST && n == SOUTH)   {turn = LEFT;}
  if (p == WEST && n == WEST)    {turn = FORWARD;}
  if (p == EAST && n == NORTH)   {turn = LEFT;}
  if (p == EAST && n == SOUTH)   {turn = RIGHT;}
  if (p == EAST && n == EAST)    {turn = FORWARD;}

  /* The circle causes problems for directions. Going from node 9 to node
   12 starts off south, then moves north. To get the final direction, figure
   out what direction the robot will approach the new node. This is done
   by finding the opposite direction of future towards current.*/
  char od = getDirection(fN,*cN); // opposite direction 
  char nd = UNDEFINED; // new direction
  if (od == NORTH) {nd = SOUTH;}
  if (od == SOUTH) {nd = NORTH;}
  if (od == EAST) {nd = WEST;}
  if (od == WEST) {nd = EAST;}
  *dir = nd; // Change the current direction to the future direction.
  *cN = fN; // Change the current node to the future node.

  return turn;
}

/*
  Function: get

  Description:
  Returns the direction to given the current node (cN) and the
  next node (nN). If no values are given returns a value of U to
  signfify an undefined output 

  Code Inputs:
    * cN: (Int) Current node of the robot
    * nN: (Int) Next node of the robot
  Code Outputs:
    * dir: (char) Direction to turn to get to new node.
*/
char getRandomTurn(){
  int rand = random(1,3);
  char turn = UNDEFINED;
  if (rand == 1)
    turn = LEFT;
  else if (rand == 2)
    turn = RIGHT;
  else if (rand == 3)
    turn = FORWARD;
  else
  {
    motor.stop_all();
    LCD.clear();
    LCD.home();
    LCD.print("Error in getRandomTurn");
    delay(100000); // 100 seconds
  }
  return turn;
}

StackList<int> pathFind(int start, int finish, int collision){
  // The collision will be set to 0 if no collision, or the node value
  // where a collision was detected, allowing no link to be made between
  // cN and collisionNode for a direct path.  
  // Return an array of nodes between cN and fN
  // This is an implementation of Dijkstra's algorithm

  StackList <int> output;
  int sN = start; // Start node
  int cN = start; // Current node
  int fN = finish; // Finish node
  bool done = false;
  bool check[20]; // Nodes already checked
  int dist[20]; // Distances from start to each node
  int prev[20]; // Best connection to previous node
  for (int c=0;c<20; c++){check[c]=false;dist[c]=999;prev[c]=999;}
  
  dist[cN-1] = 0; // Initialize the distance from start -> start = 0
  check[cN-1] = true; // Starting node is checked
  
  while (done != true){
    // Obtain lengths from current node to adjacent nodes -> replace distance if smaller than current value.
    for(int i = 1; i <= NODES; i++){ 
      int len = distance[cN-1][i-1]; // length. Note: Distance is a global variable 
      if(len != 0 && (dist[cN-1] + len) < dist[i-1] ){
          dist[i-1] = dist[cN-1] + len;
          prev[i-1] = cN; 
      }
      //Serial.print("Loop #"); Serial.println(i);
      //for (int a = 0; a<20; a++){Serial.print(dist[a]); Serial.print(" ");} Serial.println();
      //for (int a = 0; a<20; a++){Serial.print(prev[a]); Serial.print(" ");} Serial.println();
      //for (int a = 0; a<20; a++){Serial.print(check[a]); Serial.print(" ");} Serial.println();
    }
    // Find the next minimum node length. If this it the final node, end the algorithm.
    int nextNode;
    int nextLen = 999; 
    for(int j = 1; j <= NODES; j++){ 
      int nodeLen = dist[j-1];
      if (check[j-1] == false && nodeLen < nextLen){ //Prevents backwards travel & finds the shortest next node
        nextLen = nodeLen;
        nextNode = j; 
      }
    }
    // Check to see if we will redo this loop
    cN = nextNode;
    check[cN-1] = true;
    if (cN == fN)
      done = true;
  }

  bool stackLoop = false;
  while (stackLoop != true){
      output.push(cN);
      if (cN == sN){
        stackLoop = true;
        output.pop(); //Get rid of the cN = start entry
      }else{
        cN = prev[cN-1];
      }
  }
  return output;
}



