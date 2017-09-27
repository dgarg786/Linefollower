// North : 0 , South : 1 , East : 2 , West : 3
// tstraight:0 tLeft : 1 ; tright : 2   tabout : 3
# define MAX 100000
float error=0, P=0, I=0, D=0, PID_value=0;
float previous_error=0, previous_I=0;
int sensor[5]={0, 0, 0, 0, 0};
int initial_motor_speed=180;
int left_motor_speed = 0;
int right_motor_speed = 0;
void calculate_pid();
void motor_control();
int flag = 0;
int path[40][40];
int nodeIndex;
int sPath[40];
int indexSpath=1;
int parent = 0;
char c;
int finalNode;
int xcod[40];
int ycod[40];
bool isdir[40][4];
bool Sdir[40];
int top = 1;
int presentx = 0 , presenty = 0 ;
int parent1[40];
int dirFromPrev[40][40]; 
int thres = 15;

int currentdirection = 0;

void setup()
{
 pinMode(44,OUTPUT);
 pinMode(53,INPUT); 
 pinMode(4,OUTPUT); //Left Motor Pin 1
 pinMode(5,OUTPUT); //Left Motor Pin 2
 pinMode(6,OUTPUT); //Right Motor Pin 1
 pinMode(7,OUTPUT);  //Right Motor Pin 2
 digitalWrite(45,LOW);
 xcod[0] = 0;
 ycod[0] = 0;
 top++;
 Serial.begin(9600); //Enable Serial Communications
}

void loop()
{
  //if(flag == 0){
  getNode(0);
  Serial.println("I am in void loop");
  delay(60000);
  //}
  /*if(digitalRead(53)==HIGH){
    dijkstra();
    final_shortest_path();
    //Serial.println("djvh");
  }*/
  //Serial.println(digitalRead(53));
  //digitalWrite(44,HIGH);
  //turnBack();
  //delay(5000);
  //delay(10000);
  //turnToDir(currentdirection,3);
  //delay(10000);
}

void forwardSignal(){
  digitalWrite(4,HIGH);
  digitalWrite(5,LOW);
  digitalWrite(6,HIGH);
  digitalWrite(7,LOW);
}

void leftSignal(){
  digitalWrite(4,LOW);
  digitalWrite(5,HIGH);
  digitalWrite(6,HIGH);
  digitalWrite(7,LOW);
}

void rightSignal(){
  digitalWrite(4,HIGH);
  digitalWrite(5,LOW);
  digitalWrite(6,LOW);
  digitalWrite(7,HIGH);
}

void stopSignal(){
  digitalWrite(4,LOW);
  digitalWrite(5,LOW);
  digitalWrite(6,LOW);
  digitalWrite(7,LOW);
}

void straight(){
  read_sensor_values();
  //detectnode();
  updatePosition();
  calculate_pid();
  motor_control();
}

void read_sensor_values()
{
  sensor[0]=digitalRead(26);
  sensor[1]=digitalRead(27);
  sensor[2]=digitalRead(28);
  sensor[3]=digitalRead(29);
  sensor[4]=digitalRead(30);
  /*for(int i=0;i<5;i++){
    Serial.print(sensor[i]);
    Serial.print("  ");
  }
  Serial.println();*/
  if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==1))
  error=4;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==1)&&(sensor[4]==1))
  error=3;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==1)&&(sensor[4]==0))
  error=2;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==0))
  error=1;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==1)&&(sensor[3]==0)&&(sensor[4]==0))
  error=0;
  else if((sensor[0]==0)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==0)&&(sensor[4]==0))
  error=-1;
  else if((sensor[0]==0)&&(sensor[1]==1)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==0))
  error=-2;
  else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==0))
  error=-3;
  else if((sensor[0]==1)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==0))
  error=-4;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==0))
  if(previous_error < 0)error = -5;
  else error = 5;    
  /*else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==0))
    turnBack();
  else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==1)){
    turnLeft();  
  }*/
}

void calculate_pid()
{
    float Kp=175,Ki=0,Kd=100;
    P = error;
    I = I + P;
    D = error-previous_error;
    /*Serial.print("  ");
    Serial.print(P);
    Serial.print("  ");
    Serial.print(I);
    Serial.print("  ");
    Serial.print(D);*/
    PID_value = (Kp*P) + (Ki*I) + (Kd*D);
    /*Serial.print("  ");
    Serial.print(PID_value);*/
    previous_I=I;
    previous_error=error;
}

void motor_control()
{
    // Calculating the effective motor speed:
      left_motor_speed = initial_motor_speed-PID_value/6;
      right_motor_speed = initial_motor_speed+PID_value/6;
    // The motor speed should not exceed the max PWM value
    constrain(left_motor_speed,0,255);
    constrain(right_motor_speed,0,255);
    /*Serial.print("    ");
    Serial.print(left_motor_speed);
    Serial.print("    ");
    Serial.print(right_motor_speed);
    Serial.println();*/
  
    //analogWrite(9,left_motor_speed);   //Left Motor Speed
    //analogWrite(10,right_motor_speed);  //Right Motor Speed
    //following lines of code are to make the bot move forward
    /*The pin numbers and high, low values might be different
    depending on your connections */
    analogWrite(4,constrain(right_motor_speed,0,255));
    digitalWrite(5,LOW);
    analogWrite(6,constrain(left_motor_speed,0,255));
    digitalWrite(7,LOW);
}

void turnLeft() {
  Serial.print("Co-ordinates      ");
  Serial.print(presentx);
  Serial.print("   ");
  Serial.print(presenty);
  Serial.println();
  Serial.println("L");
  currentdirection = nextDir(currentdirection,1); 
  //read_sensor_values();
  while(digitalRead(28)==1){
    leftSignal();
    /*delay(2);
    stopSignal();
    delay(1);*/
  }
  //stopSignal();
  //delay(100);
  //read_sensor_values();
  while(digitalRead(28)==0){
    leftSignal();
    /*delay(2);
    stopSignal();
    delay(1);*/
  }
  stopSignal();
  delay(20);
}

void turnBack() {
  Serial.print("Co-ordinates      ");
  Serial.print(presentx);
  Serial.print("   ");
  Serial.print(presenty);
  Serial.println();
  Serial.println("B");
  currentdirection=nextDir(currentdirection,3);
  rightSignal();
  delay(1000);
  //read_sensor_values();
  /*while(digitalRead(28)==1){
    rightSignal();
    /*delay(2);
    stopSignal();
    delay(1);*/
  //}*/
  /*while(digitalRead(28)==0){
    rightSignal();
    /*delay(2);
    stopSignal();
    delay(1);
  }*/
  stopSignal();
  delay(20);
}

void turnRight() {
  Serial.print("Co-ordinates      ");
  Serial.print(presentx);
  Serial.print("   ");
  Serial.print(presenty);
  Serial.println();
  Serial.println("R");
  currentdirection = nextDir(currentdirection,2);
  //read_sensor_values();
  while(digitalRead(28)==1){
    rightSignal();
    /*delay(2);
    stopSignal();
    delay(1);*/
  }
  //read_sensor_values();
  while(digitalRead(28)==0){
    rightSignal();
    /*delay(2);
    stopSignal();
    delay(1);*/
  }
  stopSignal();
  delay(20);
}

int updateNodeData(int inDir) {
    delay(100);
    read_sensor_values();
    int nodeIndex = top;
    top++;
    xcod[nodeIndex] = presentx;
    ycod[nodeIndex] = presenty;
    int x = xcod[nodeIndex]-xcod[parent];
    int y = ycod[nodeIndex]-ycod[parent];
    path[nodeIndex][parent] = sqrt(sq(x)+sq(y));//square root of ((xcod of nodeindex - xcod of parent)^2 + (ycod of nodeindex - ycod of parent)^2)  i.e., distance between two points
    path[parent][nodeIndex] = sqrt(sq(x)+sq(y));//square root of ((xcod of nodeindex - xcod of parent)^2 + (ycod of nodeindex - ycod of parent)^2)  i.e., distance between two points
    dirFromPrev[parent][nodeIndex] = inDir;
    dirFromPrev[nodeIndex][parent] = oppDir(inDir);
    parent = nodeIndex;
    isdir[nodeIndex][0] = false;
    isdir[nodeIndex][1] = false;
    isdir[nodeIndex][2] = false;
    isdir[nodeIndex][3] = false;
    /*Serial.print("sensor[0] = ");
    Serial.print(sensor[0]);
    Serial.print("sensor[4] = ");
    Serial.print(sensor[4]);
    Serial.println();*/
    if( sensor[0] == 1 )isdir[nodeIndex][nextDir(currentdirection,1)] = true;
    if( sensor[4] == 1)isdir[nodeIndex][nextDir(currentdirection,2)] = true;
    forwardSignal();
    delay(300);
    stopSignal();
    read_sensor_values();
    if( sensor[2] == 1)isdir[nodeIndex][inDir] = true;
    //add the goal condition
    if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==1)){
      flag=1;
      finalNode = nodeIndex;
    }
    /*if(flag == 1 && nodeIndex == 0){
      dijkstra();
    }*/
    return nodeIndex;
}
bool detectnode() {
  Serial.print("Node detecting ........");
  read_sensor_values();
  for(int i=0;i<5;i++){
    Serial.print(sensor[i]);
    Serial.print("  ");
  }
  Serial.println();
  //read_sensor_values();
    if( sensor[0] == 1 )return true;
    if(  sensor[4] == 1)return true;
    if(sensor[0] == 0 && sensor[1] == 0 && sensor[2] == 0 && sensor[3] == 0 && sensor[4] == 0)return true;
    return false;
}



void getBackPrevNode(int inDir){
  Serial.print("I am going to previous node");
  Serial.println();
  turnToDir(currentdirection, oppDir(inDir));
  dirFromPrev[nodeIndex][parent] = inDir;
  while(detectnode() == false) {
    straight();
    delay(20);
  }  
  forwardSignal();
  delay(300);
  stopSignal();
  read_sensor_values();
}

void getNode(int inDir) {
  //Serial.print("CD   ");
  //Serial.print(currentdirection);
  Serial.print("Getting node");
  Serial.println();
  turnToDir(currentdirection,inDir);
  while(detectnode()==false) {
    straight();
    delay(20);
  }
  Serial.println("NODE");
  delay(100);
  stopSignal();
  if(isvisit(presentx, presenty,inDir)==true) {
    getBackPrevNode(inDir);
    return;
  }
  nodeIndex = updateNodeData(inDir);
  /*Serial.print('a');
  Serial.print("  ");
  Serial.print(nodeIndex);
  Serial.print("  ");
  Serial.print(isdir[nodeIndex][nextDir(inDir,1)]);
  Serial.print("  ");
  Serial.print(isdir[nodeIndex][inDir]);
  Serial.print("  ");
  Serial.print(isdir[nodeIndex][nextDir(inDir,2)]);
  Serial.print("  ");
  Serial.print("function1   ");
  Serial.print(nextDir(inDir,1));
  Serial.println();
  Serial.print("function2   ");
  Serial.print(presentx);
  Serial.println();
  Serial.print("function2   ");
  Serial.print(presenty);*/
  //Serial.println();
  int indir1 = nextDir(inDir,1);    
  if(isdir[nodeIndex][indir1]) {
    isdir[nodeIndex][indir1] = false;
    getNode(indir1);
  }
  parent = nodeIndex;
  presentx=xcod[nodeIndex];
  presenty=ycod[nodeIndex];
  if(isdir[nodeIndex][inDir]) {
    isdir[nodeIndex][inDir] = false;
    getNode(inDir);
  }
  parent = nodeIndex;
  presentx=xcod[nodeIndex];
  presenty=ycod[nodeIndex];
  int indir2 = nextDir(inDir,2);
  if(isdir[nodeIndex][indir2]) {
    isdir[nodeIndex][indir2] = false;
    getNode(indir2);
  }
  presentx=xcod[nodeIndex];
  presenty=ycod[nodeIndex];
  getBackPrevNode(inDir);

}

bool isvisit(int newNodeX, int newNodeY,int inDir) {
  Serial.print("Checking if the node is visited or not");
  Serial.println();
  for(int i = 0; i < top; i++){
    if(abs(xcod[i] - newNodeX) <= thres && abs(ycod[i] - newNodeY) <= thres){
      Serial.print("Visited");
      Serial.println();
      isdir[i][oppDir(currentdirection)]=false;
      int x = xcod[i]-xcod[parent];
      int y = ycod[i]-ycod[parent];
      path[i][parent] = sqrt(sq(x)+sq(y));//square root of ((xcod of nodeindex - xcod of parent)^2 + (ycod of nodeindex - ycod of parent)^2)  i.e., distance between two points
      path[parent][i] = sqrt(sq(x)+sq(y));//square root of ((xcod of nodeindex - xcod of parent)^2 + (ycod of nodeindex - ycod of parent)^2)  i.e., distance between two points
      dirFromPrev[parent][i] = inDir;
      dirFromPrev[i][parent] = oppDir(inDir);
      return true;
    }
  }
  Serial.println("Not Visited");
  return false;
}



int turnToDir(int currDir,int finalDir){
  if(currDir == finalDir) return;
  /*Serial.print("Currentdir    ");
  Serial.print(currDir);
  Serial.print("Finaldir    ");
  Serial.print(finalDir);*/

  if(currDir == 0) {
    if(finalDir == 1) 
      turnBack();
    else if(finalDir == 2) 
      turnRight();
    else if(finalDir == 3) 
      turnLeft();
  }
  else if(currDir == 1) {
    if(finalDir == 0) 
      turnBack();
    else if(finalDir == 3) 
      turnRight();
    else if(finalDir == 2) 
      turnLeft();
  }
  else if(currDir == 2) {
    if(finalDir == 3) 
      turnBack();
    else if(finalDir == 1) 
      turnRight();
    else if(finalDir == 0) 
      turnLeft();
  }
  else if(currDir == 3) {
    if(finalDir == 2) 
      turnBack();
    else if(finalDir == 0) 
      turnRight();
    else if(finalDir == 1) 
      turnLeft();
  }
  currentdirection = finalDir;
}




int  nextDir(int currDir, int turn ) {
  if(currDir==0&&turn==0)
    return 0;
  if(currDir==0&&turn==1)
    return 3;
  if(currDir==0&&turn==2)
    return 2;
  if(currDir==0&&turn==3)
    return 1;
  if(currDir==1&&turn==0)
    return 1;
  if(currDir==1&&turn==1)
    return 2;
  if(currDir==1&&turn==2)
    return 3;
  if(currDir==1&&turn==3)
    return 0;
  if(currDir==2&&turn==0)
    return 2;
  if(currDir==2&&turn==1)
    return 0;
  if(currDir==2&&turn==2)
    return 1;
  if(currDir==2&&turn==3)
    return 3;
  if(currDir==3&&turn==0)
    return 3;
  if(currDir==3&&turn==1)
    return 1;
  if(currDir==3&&turn==2)
    return 0;
  if(currDir==3&&turn==3)
    return 2;    
}
int updatePosition() {
  if(currentdirection==0)presentx++;
  else if(currentdirection==1)presentx--;
  else if(currentdirection==2)presenty++;
  else presenty--;
}

int oppDir(int currDir) {
  if(currDir == 0)return 1;
  if(currDir == 1)return 0;
  if(currDir == 2)return 3;
  if(currDir == 3)return 2;
}
int shortpath(int s){
  if(parent1[s] == -1)
    return;
    //sPath[parent1] = s;
    shortpath(parent1[s]);
    sPath[indexSpath] = s;
    indexSpath++;
}
int dijkstra(){
  int s=0,i=0,dist[100];
  for(int i=0;i<nodeIndex;i++){
    dist[i] = MAX;
  }
  parent1[0] = -1;
  dist[s]=0;
  int visit[100];
  for(int i=0;i<nodeIndex;i++){
    visit[i] = 0;
  }
  while(s != finalNode){
    visit[s]=1;
    int maxdis = MAX;
    int indx = -1;
    for(int j=0;j<nodeIndex;j++){
      if(dist[j]>dist[s]+path[s][j]){
        dist[j]=dist[s]+path[s][j];
        parent1[j]=s;
      }
      if(MAX > dist[j]&&visit[j]==0){
        maxdis = dist[j];
        indx = j;
      }
    }
    s=indx;
  }
  shortpath(finalNode);
}
int take(int nodeA,int nodeB){
  turnToDir(currentdirection,dirFromPrev[nodeB][nodeA]);
  while(detectnode()==false) {
    straight();
    delay(20);
  }
  delay(100);
  stopSignal();
}
void final_shortest_path(){
  currentdirection = 0;
  for(int i = 1; i < indexSpath; i++){
    take(sPath[i-1],sPath[i]);
  }
}