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
#define BACKWARD 'B'
#define UNDEFINED 'U'

const int nodeDistance[NODES][NODES] = //Non-zero values are valid connections (cm)
{  //Note. We want to avoid 4->17. Make it longer thab 4-5-9-12-13-17 (length 418), but shorter than 4-3-2-5-9-12-13-19-18-17 (length 962)
  {0,87,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, // Node 1
  {87,0,99,0,115,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, // Node 2
  {0,99,0,132,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, // Node 3
  {0,0,132,0,74,0,0,0,0,0,0,0,0,0,0,0,600,0,0,0}, // Node 4
  {0,115,0,74,0,66,0,0,65,0,0,0,0,0,0,0,0,0,0,0}, // Node 5
  {0,0,0,0,66,0,68,89,0,0,0,0,0,0,0,0,0,0,0,0}, // Node 6
  {0,0,0,0,0,68,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, // Node 7
  {0,0,0,0,0,89,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, // Node 8
  {0,0,0,0,65,0,0,0,0,90,0,140,0,0,0,0,0,0,0,0}, // Node 9
  {0,0,0,0,0,0,0,0,90,0,57,90,0,0,0,0,0,0,0,0}, // Node 10
  {0,0,0,0,0,0,0,0,0,57,0,0,0,0,0,0,0,0,0,0}, // Node 11
  {0,0,0,0,0,0,0,0,140,90,0,0,65,0,0,0,0,0,0,0}, // Node 12
  {0,0,0,0,0,0,0,0,0,0,0,65,0,66,0,0,74,0,115,0}, // Node 13
  {0,0,0,0,0,0,0,0,0,0,0,0,66,0,89,68,0,0,0,0}, // Node 14
  {0,0,0,0,0,0,0,0,0,0,0,0,0,89,0,0,0,0,0,0}, // Node 15
  {0,0,0,0,0,0,0,0,0,0,0,0,0,68,0,0,0,0,0,0}, // Node 16
  {0,0,0,600,0,0,0,0,0,0,0,0,74,0,0,0,0,132,0,0}, // Node 17
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,132,0,99,0}, // Node 18
  {0,0,0,0,0,0,0,0,0,0,0,0,115,0,0,0,0,99,0,87}, // Node 19
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,87,0}  // Node 20
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
const int intersectionShapes[NODES][DIRECTIONS] = 
{
  //0 -> Nothing expected (Invalid direction of approach || end node)
  //1 -> Forward, Left, Right 
  //2 -> Forward, Right
  //3 -> Forward, Left
  //4 -> Left, Right
  //5 -> Right
  //6 -> Left
  //N , E , S , W
  { 0 , 0 , 0 , 0 }, // Node 1
  { 2 , 0 , 3 , 4 }, // Node 2
  { 0 , 0 , 6 , 5 }, // Node 3
  { 0 , 3 , 4 , 2 }, // Node 4
  { 1 , 1 , 1 , 1 }, // Node 5
  { 3 , 4 , 2 , 0 }, // Node 6
  { 0 , 0 , 0 , 0 }, // Node 7
  { 0 , 0 , 0 , 0 }, // Node 8
  { 3 , 4 , 2 , 0 }, // Node 9
  { 0 , 3 , 4 , 2 }, // Node 10
  { 0 , 0 , 0 , 0 }, // Node 11
  { 2 , 0 , 3 , 4 }, // Node 12
  { 1 , 1 , 1 , 1 }, // Node 13
  { 2 , 0 , 3 , 4 }, // Node 14
  { 0 , 0 , 0 , 0 }, // Node 15
  { 0 , 0 , 0 , 0 }, // Node 16
  { 0 , 3 , 4 , 2 }, // Node 17
  { 0 , 6 , 5 , 0 }, // Node 18
  { 3 , 4 , 2 , 0 }, // Node 19
  { 0 , 0 , 0 , 0 }  // Node 20
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
  Returns the direction given the current node (cN) and the
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
  Function: getNode

  Description:
  Takes inputs of an intersection node, and a direction. Will return
  the node that is attached in that direction, or 999 to indicate
  no connection exists.

  Code Inputs:
    * cN: (Int) Current node of the robot
    * dir: (char) The direction of the robot
  Code Outputs:
    * output: (int) The node in that direction, of 999 to indicate no node.
*/
int getNode(int cN, char dir){

  int output = 999; // This should be 1 to 20. This is the error value
  for (int i=1;i<=NODES; i++){
    if (nodeDirections[cN-1][i-1] == dir) output = i;
  }
  return output;
}

/*
  Function: getShape

  Description:
  Takes inputs of an intersection node, and a direction. Will return
  the shape that the robot expects to see upon arrival at that intersection.
  The available "shapes" are as follows.

  //0 -> Nothing expected (Invalid direction of approach || end node)
  //1 -> Forward, Left, Right 
  //2 -> Forward, Right
  //3 -> Forward, Left
  //4 -> Left, Right
  //5 -> Right
  //6 -> Left

  Code Inputs:
    * cN: (Int) Current node of the robot
    * dir: (char) The direction of the robot
  Code Outputs:
    * output: (int) The shape in that direction. 0 if invalid
*/
int getShape(int cN, char dir){
  int output = 0;
  int dirInt = 999;
  if (dir == NORTH) {dirInt = 0;}
  if (dir == EAST) {dirInt = 1;}
  if (dir == SOUTH) {dirInt = 2;}
  if (dir == WEST) {dirInt = 3;}
  output = intersectionShapes[cN-1][dirInt];
  return output;
}

/*
  Function: turnDirection

  Description:
  This function returns the direction to move turn the robot from the
  current node to the future node. This output will either be FORWARD,
  BACKWARD, LEFT, or RIGHT (F,B,L,R).

  Code Inputs:
    * cN: The current node 
    * fN: The future node
    * dir: The current direction of the robot
  Code Outputs:
    * direction : F,B,L,R
*/
char turnDirection(int cN, int fN, char dir){

  char p = dir; // Previous Direction
  char n = getDirection(cN,fN); // Next Direction
  char turn = UNDEFINED;

  if (p == NORTH && n == NORTH)  {turn = FORWARD;}
  if (p == NORTH && n == EAST)   {turn = RIGHT;}
  if (p == NORTH && n == WEST)   {turn = LEFT;}
  if (p == NORTH && n == SOUTH)  {turn = BACKWARD;}
  if (p == SOUTH && n == SOUTH)  {turn = FORWARD;}
  if (p == SOUTH && n == EAST)   {turn = LEFT;}
  if (p == SOUTH && n == WEST)   {turn = RIGHT;}
  if (p == SOUTH && n == NORTH)  {turn = BACKWARD;}
  if (p == WEST && n == NORTH)   {turn = RIGHT;}
  if (p == WEST && n == SOUTH)   {turn = LEFT;}
  if (p == WEST && n == WEST)    {turn = FORWARD;}
  if (p == WEST && n == EAST)    {turn = BACKWARD;}
  if (p == EAST && n == NORTH)   {turn = LEFT;}
  if (p == EAST && n == SOUTH)   {turn = RIGHT;}
  if (p == EAST && n == EAST)    {turn = FORWARD;}
  if (p == EAST && n == WEST)    {turn = BACKWARD;}

  return turn;
}

/*
  Function: updateParameters

  Description:
  This function will update the robots direction and current node. To reflect
  new values. This should be called after every intersection, however may
  also be called after two-point turns (if we choose to implement them).
  The cN and dir inputs are pointers that will be changed upon function calls.

  Code Inputs:
    * *cN: The current node pointer 
    * fN: The future node
    * *dir: The current direction of the robot
  Code Outputs:
    * None
*/
void updateParameters(int *cN, int fN, char *dir){
  //The circle causes problems for directions. Going from node 9 to node
  //12 starts off south, then moves north. To get the final direction, figure
  //out what direction the robot will approach the new node. This is done
  //by finding the opposite direction of future towards current.
  char od = getDirection(fN,*cN); // opposite direction 
  char nd = UNDEFINED; // new direction
  if (od == NORTH) {nd = SOUTH;}
  if (od == SOUTH) {nd = NORTH;}
  if (od == EAST) {nd = WEST;}
  if (od == WEST) {nd = EAST;}
  *dir = nd; // Change the current direction to the future direction.
  *cN = fN; // Change the current node to the future node.
}

/*
  Function: pathFind

  Description:
  This function will find the shortest distance by using the Dijkstra
  algorithm. It will use the distance[][] array above for determining
  distances. It will avoid reversing by considering the current 
  direction of the robot. 

  Code Inputs:
    * start: The current node of the robot (node that the robot is approaching)
    * finish: The desired final node of the robot.
    * direction: The current direction of the robot
  Code Outputs:
    * StackList: A stack of future commands. The first to be executed will be 
                 on top of the stack.
*/
StackList<int> pathFind(int start, int finish, char direction){
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
  int distance[20][20]; 
  for(int a=0;a<20;a++){for(int b=0;b<20;b++){distance[a][b]=nodeDistance[a][b];}} //Copy nodeDistance to clean array
  for(int c=0;c<20;c++){check[c]=false;dist[c]=999;prev[c]=999;} //Assign above array values

  dist[cN-1] = 0; // Initialize the distance from start -> start = 0
  check[cN-1] = true; // Starting node is checked

  //Set the reverse direction to be impossible by modifying distance[][]
  char rev = UNDEFINED;
  if (direction == NORTH) {rev = SOUTH;}
  if (direction == SOUTH) {rev = NORTH;}
  if (direction == EAST) {rev = WEST;}
  if (direction == WEST) {rev = EAST;}
  int bck = getNode(start,rev);
  distance[start-1][bck-1] = 0; // Can never go backwards 
    
  while (done != true){
    // Obtain lengths from current node to adjacent nodes -> replace distance if smaller than current value.
    for(int i = 1; i <= NODES; i++){ 
      int len = distance[cN-1][i-1]; // length. Note: Distance is a global variable 
      if(len != 0 && (dist[cN-1] + len) < dist[i-1] ){
          dist[i-1] = dist[cN-1] + len;
          prev[i-1] = cN; 
      }
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
    if (cN == fN){done = true;}
    if (nextLen == 999) {done = true;} //Nothing left 
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

/*
  Function: pathFind_noFwd

  Description:
  This function will find the shortest distance by using the Dijkstra
  algorithm. It will use the distance[][] array above for determining
  distances. It will avoid reversing by considering the current 
  direction of the robot. It will not travel forward

  Code Inputs:
    * start: The current node of the robot (node that the robot is approaching)
    * finish: The desired final node of the robot.
    * direction: The current direction of the robot
  Code Outputs:
    * StackList: A stack of future commands. The first to be executed will be 
                 on top of the stack.
*/
StackList<int> pathFind_noFwd(int start, int finish, char direction){
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
  int distance[20][20]; 
  for(int a=0;a<20;a++){for(int b=0;b<20;b++){distance[a][b]=nodeDistance[a][b];}} //Copy nodeDistance to clean array
  for(int c=0;c<20;c++){check[c]=false;dist[c]=999;prev[c]=999;} //Assign above array values

  dist[cN-1] = 0; // Initialize the distance from start -> start = 0
  check[cN-1] = true; // Starting node is checked

  //Set the forward direction to be impossible by modifying distance[][]
  int fwd = getNode(start,direction);
  distance[start-1][fwd-1] = 0; // Never go forward (guaranteed to exist as this function is only called upon straight reverse)

  //Set the reverse direction to be impossible by modifying distance[][]
  char rev = UNDEFINED;
  if (direction == NORTH) {rev = SOUTH;}
  if (direction == SOUTH) {rev = NORTH;}
  if (direction == EAST) {rev = WEST;}
  if (direction == WEST) {rev = EAST;}
  int bck = getNode(start,rev);
  if (bck != 999){distance[start-1][bck-1] = 0;} // Can never go backwards (if backwards exists, 999 is the not-existing value)
    
  while (done != true){
    // Obtain lengths from current node to adjacent nodes -> replace distance if smaller than current value.
    for(int i = 1; i <= NODES; i++){ 
      int len = distance[cN-1][i-1]; // length. Note: Distance is a global variable 
      if(len != 0 && (dist[cN-1] + len) < dist[i-1] ){
          dist[i-1] = dist[cN-1] + len;
          prev[i-1] = cN; 
      }
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
    if (cN == fN){done = true;}
    if (nextLen == 999) {done = true;} //Nothing left 
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
