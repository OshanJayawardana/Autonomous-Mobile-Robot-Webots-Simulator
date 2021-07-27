#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Brake.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <tuple>
#include <iostream>
#include <cstdlib>
#include <vector>
#include<string> 
#include <webots/LED.hpp>
#include <string>


#define TIME_STEP 64
#define MAX_SPEED 6.28
using namespace webots;
using namespace std;

////////////////////////////////////////////////////////////////////////////////////
//Global Variables
DistanceSensor* ir[8]; //ir_panle
DistanceSensor* ts[2]; //two irs to detect junctions
DistanceSensor* ds[2]; //Two distance sensors to detect wall.
DistanceSensor* fds;
Brake* left_brk;
Brake* right_brk;
PositionSensor* pos_left;
PositionSensor* pos_right;
LED* led;
Motor* leftMotor;
Motor* rightMotor;
Motor* sensorMotor;
Motor* sliderMotor;

//initial pid values
double p = 0;
double i = 0;
double d = 0;
double lastError = 0;
//....................................................
//global variables for maze
bool mazeIn=false;
int quad;
int rad;
bool boxFound=false;
bool colorChecked=false;

bool even=true; //path check parameter

bool found=false;
bool checked=false;
//....................................................

//global variables for pillar counting
bool reverse = false;
int pc=1;
int count = 0;
int ps=0;
int cs=0;
int pj=0;
//.......................................................

//global variables for gate sync
bool safety=true; //safety lock for gate sync. turn off only when code is finalized

bool gatePrev=false;
bool gateCur=false;
bool go=false;
//.....................................................
double dc = 0; //damping coeficient
//......................................................
//junction identifying parameters
bool turn_command = false;
vector<double> pos_lst;
int junc = -1;
vector<int> direct{100,0, 2, 2}; //direct = [1, 0];
vector<string> state{"starting","startingPath","wallFollow","straighPath","enterMaze"};
int pillarLoc=10;
int gate1Loc=pillarLoc+2;
int gate2Loc=pillarLoc+3;
int direct_count = 3;
bool canUpdateStates = true; //variable that enables updating the state vector(directions)
////////////////////////////////////////////////////////

double last_left_speed;
double last_right_speed;
double leftSpeed;
double rightSpeed;

////////////////////////////////////////////////////////
bool leftWall;
bool rightWall;

////////////////////////////////////////////////////////
double leftDsValue;
double rightDsValue;

//////////////////////////////////////////////////////

double junValues[8];

// predefined functions
////////////////////////////////////////////////////////////////////////////////////

// pid controll
void pido() {
    // initializing the pid coefficients
    // 0.14 0.001 0.0001
    float kp = 0.14;
    float ki = 0.001;
    float kd = 0.0001;

    // initializing the array to store ir sensor values
    double ir_values[8];
    double ir_sum = 0;

    for (int i = 0; i < 8; i++) {
        double ir_val = ir[i]->getValue();
        ir_values[i] = ir_val;
        ir_sum += ir_val;
    }

    // standard deviation of the ir array
    double sd = 0;

    for (int i = 0; i < 8; i++) {
        sd += (ir_values[i] - ir_sum / 8)* (ir_values[i] - ir_sum / 8);
    }

    sd = pow(sd / 8, 0.5);

    for (int i = 0; i < 8; i++) {
        ir_values[i] = (ir_values[i] - ir_sum / 8) / (sd + 0.0001);
    }
    // now the ir readings are normalized to a mean of 0 and standard deviation of 1

    // variable for storing the position
    double pos = 0;

    for (int i = 0; i < 4; i++) {
        pos += ir_values[i] * (-i + 4) + ir_values[7 - i] * (-4 + i);
    }

    double error = 0.0 - pos;
    p = error;
    i = i + p;
    d = error - lastError;
    lastError = error;
    double motor_speed = kp * p + ki * i + kd * d;
    leftSpeed = 0.5 * MAX_SPEED - motor_speed;
    rightSpeed = 0.5 * MAX_SPEED + motor_speed;
};
//..........................................................................

// Aruna check this
void pid() {
    // initializing the PID coefficients
    //float kp = 0.14;
    //float ki = 0.001;
    // 0.0001
    float kp = 0.1;
    float ki = 0.001;
    float kd = 0.005;

    // initializing the array to store IR sensor values
    double irValues[8];
    double irSum = 0;

    for (int j = 0; j < 8; j++) {
        double irVal = ir[j]->getValue();
        irValues[j] = irVal;
        junValues[j] = irVal;
        irSum += irVal;
    }
    

    //for (double item : junValues)
        //std::cout << item << ", ";
    //cout << endl;

    // standard deviation of the ir array
    double sd = 0;

    for (int j = 0; j < 8; j++) {
        sd += (irValues[j] - irSum / 8) * (irValues[j] - irSum / 8);
    }

    sd = pow(sd / 8, 0.5);

    for (int j = 0; j < 8; j++) {
        irValues[j] = (irValues[j] - irSum / 8) / (sd + 0.0001);
    }
    //for (double item : irValues)
        //std::cout << item << ", ";
    //cout << endl;
    // now the ir readings are normalized to a mean of 0 and standard deviation of 1

    // variable for storing the position
    double pos = 0;

    for (int j = 0; j < 4; j++) {
        pos += irValues[j] * (-j + 4) + irValues[7 - j] * (-4 + j);
    }

    double error = 0.0 - pos;
    p = error;
    i = i + p;

    if (i > 200) {
        i = 200;
    }
    else if (i < -200) {
        i = -200;
    }

    d = error - lastError;
    lastError = error;
    
    cout << "p: " << p << endl;
    cout << "i: " << i << endl;
    cout << "d: " << d << endl;
    
    double motorSpeed = kp * p + ki * i + kd * d;
    cout << "motor speed: " << motorSpeed << endl;

    leftSpeed = 0.5 * MAX_SPEED - motorSpeed;
    rightSpeed = 0.5 * MAX_SPEED + motorSpeed;
    //std::cout <<"left"<< leftSpeed<< std::endl;
};
//..........................................................................

//Wall following
void wallFollowing() {
    std::cout << "wall following"<<std::endl;
    if (leftWall) {
        //cout << "left wall" << endl;
        if (leftDsValue > 400) {
            //cout << "turn left" << endl;
            leftSpeed = -MAX_SPEED * 0.5;
            rightSpeed = MAX_SPEED * 0.5;
        }
        else if (leftDsValue < 300) {
            //cout << "turn right" << endl;
            rightSpeed = -MAX_SPEED * 0.5;
            leftSpeed = MAX_SPEED * 0.5;
        }
        else {
            leftSpeed = MAX_SPEED * 0.5;
            rightSpeed = MAX_SPEED * 0.5;
        }
    }

    else if (rightWall) {
        //cout << "right wall" << endl;
        if (rightDsValue > 400) {
            //cout << "turn right" << endl;
            rightSpeed = -MAX_SPEED * 0.5;
            leftSpeed = MAX_SPEED * 0.5;
        }
        else if (rightDsValue < 300) {
            //cout << "turn left" << endl;
            leftSpeed = -MAX_SPEED * 0.5;
            rightSpeed = MAX_SPEED * 0.5;
        }
        else {
            leftSpeed = MAX_SPEED * 0.5;
            rightSpeed = MAX_SPEED * 0.5;
        }
    }

}

//No line
void noLine() {
    //cout << "no line" << endl;
    leftSpeed = MAX_SPEED * 0.5;
    rightSpeed = MAX_SPEED * 0.5;
}

void wall() {
    leftDsValue = ds[0]->getValue();
    rightDsValue = ds[1]->getValue();

    //cout << "left ds: " << leftDsValue << endl;
    //cout << "right ds: " << rightDsValue << endl;

    leftWall = leftDsValue < 1000;
    rightWall = rightDsValue < 1000;

    ///////////////////////////////
    //for checking whether there is a line
    bool cond = false;
    for (int j = 0; j < 8; j++) {
        if (junValues[j] < 60000) {
            cond = true;
            //cout << "hey" << endl;
        }
    }

    //cout << "cond" << cond << endl;

    //condition for wall following
    if ((leftWall or rightWall) && !cond) {
        wallFollowing();
    }
    else if (!cond) {
        noLine();
    }


    //cout << leftSpeed << endl;
    //cout << rightSpeed << endl;
}

//junction identificationh
int juncFind() {
    
    double ir_left = ts[0]->getValue();
    double ir_right = ts[1]->getValue();
    //std::cout << "ir val "<< ir_left  <<std::endl;
    //250 was the previous threshold
    bool left = ir_left < 60000;
    bool right = ir_right < 60000;
    if (left && !right) {
        junc = 0;
    }
    else if (left && right) {
        junc = 1;
    }
    else if (!left && right) {
        junc = 2;
    }
    else {
        junc = -1;
    }
    return junc;
};
//....................................................

//set motors
void setMotors() {
    //setting motor speeds
    leftMotor->setVelocity(leftSpeed);
    rightMotor->setVelocity(rightSpeed);

    //setting brakes
    left_brk->setDampingConstant(dc);
    right_brk->setDampingConstant(dc);

    //storing the speed for next loop
    last_left_speed = leftSpeed;
    last_right_speed = rightSpeed;
};
//....................................................

//sharpTurns
void sharpTurn(int turn) {
    double hardLength;
    if (turn == 0) {
        hardLength = 28.0;
        std::cout << "turning left"<<std::endl;
        leftSpeed = 0 * MAX_SPEED;
        rightSpeed = 0.5 * MAX_SPEED;
    }
    else if (turn == 1) {
        hardLength = 8.0;
        std::cout << "going forward"<<std::endl;
        leftSpeed = 0.5 * MAX_SPEED;
        rightSpeed = 0.5 * MAX_SPEED;
    }
    else if (turn == 2) {
        hardLength = 28.0;
        std::cout << "turning right"<<std::endl;
        leftSpeed = 0.5 * MAX_SPEED;
        rightSpeed = 0 * MAX_SPEED;
    }
    else if (turn == -1){
        hardLength = 34.0;
        std::cout << "turning back"<<std::endl;
        leftSpeed = -0.5 * MAX_SPEED;
        rightSpeed = 0.5 * MAX_SPEED;
    }
    else if (turn == 20){
        hardLength = 19.0;
        std::cout << "ramp right"<<std::endl;
        leftSpeed = 0.5 * MAX_SPEED;
        rightSpeed = -0.5 * MAX_SPEED;
    }
    else if (turn == 41){
        hardLength = 8.0;
        std::cout << "quad 4 forward"<<std::endl;
        leftSpeed = 0.5 * MAX_SPEED;
        rightSpeed = 0.5 * MAX_SPEED;
    }
    else if (turn == 100){
        hardLength = 20.0;
        std::cout << "start forward"<<std::endl;
        leftSpeed = 0.5 * MAX_SPEED;
        rightSpeed = 0.5 * MAX_SPEED;
    }
    else {
        cout << "wrong input";
    }
    
    
    double pos_val = pos_right->getValue();
    if (turn == 2) {
        double pos_val = pos_left->getValue();
    }
    if (abs(pos_val) > 0) {
        pos_lst.push_back(pos_val);
        std::cout << "encorder"<< abs(pos_lst.begin() - pos_lst.end())<< std::endl;
        if (abs(pos_lst.begin() - pos_lst.end()) > hardLength) {
            junc = -1;
            pos_lst={};
            turn_command = false;
            direct_count += 1;
            if (safety){go=false;}
            canUpdateStates = true;
            //canUpdateLoc=true;
            //remove this at final stage. This only for safety
            //depends on final robot speed
        }
    }
};
//....................................................

//braking
void brakes() {
    //cout << "brake";
    double speed = last_right_speed;
    if (last_left_speed > last_right_speed) {
        double speed = last_left_speed;
    }
    if ( speed < 0.5 * MAX_SPEED) {
        leftSpeed = 0;
        rightSpeed = 0;
        turn_command = true;
        dc = 0;
    }
    else {
        leftSpeed = speed - 1;
        rightSpeed = speed - 1;
        dc += 0;
    }
};
////////////////////////////////////////////////////////////////////////////////////

void pillarCnt() {

  led->set(0);
   
  //counting pillars
  for (int j = 0; j < 2; j++) {
    if ((ds[j]->getValue())>100 && (ds[j]->getValue())<250){
      pj=j;
      cs=1;
      if (ps==0 && state[direct_count]=="pillar") {
          count += 1; 
          led->set(1);
          ps=cs;   
          //std::cout << "No. of pillars: " << count << std::endl; 
      } else {
          led->set(0);
        }     
    } else { 
      if (pj==j) {
        ps=0;
        cs=0;
      }
      }

    if (ts[j]->getValue() < 400) {
      if (count%2 == 1) {  //wrong path
        reverse = true; 
        pc = 0;
      }
      
    }
   }

}
//////////////////////////////////////////////////////////////////////////////////////
void gatesync(){
    const double value = fds->getValue();
    //std::cout << "Sensor value is : " << value << std::endl;
    if (value <= 750){
      gatePrev=gateCur;
      gateCur=true;
    }
    else{
      gatePrev=gateCur;
      gateCur=false;  
    }
    if (gateCur==false && gatePrev==true){
      go=true;
    }
    else if (gateCur==true && gatePrev==false){
      go=false;
    }
    
    
    //std::cout << "gatePrev = " << gatePrev << " gateCur = " << gateCur  <<std::endl;
    if ( go && (state[direct_count]=="gate1" or state[direct_count]=="gate2")){
      std::cout << "go" <<std::endl;  
    }
    else if(state[direct_count]=="gate1" or state[direct_count]=="gate2"){
      std::cout << "stop" <<std::endl;
      leftSpeed=0;
      rightSpeed=0;
      //rightMotor->setPosition(0.0);
      //leftMotor->setPosition(0.0);
    }
    //else{
      //pid();
      //rightMotor->setPosition(INFINITY);
      //leftMotor->setPosition(INFINITY);
      //rightMotor->setVelocity(10.0);
      //leftMotor->setVelocity(10.0);
    //}
}
///////////////////////////////////////////////////////////////////////////////////////////


int quadUpdate(int cur, int add){
    cur+=add;
    if (cur>4){
        cur=cur%4;
    }
    return cur;
}

int radUpdate(int cur,int add){
    cur+=add;
    if (cur<1){
        cur=4;
    }
    if (cur>4){
      cur=cur%4;
    }
    return cur;
}
///////////////////////////////////////////////////////////////////////////////////////////
void mazeLoc(){
    if (true){
        //...............................................quad and rad Updating
        //Initial quad
        if (state[direct_count-1]=="enterMaze"){
            quad=1;
        }
        //....................................................
        //circle to (circle or rad)
        if (state[direct_count-1]=="circlePath" && direct[direct_count-1]==1){
            quad = quadUpdate(quad, 1);
        }
        else if (state[direct_count-1]=="circlePath"){
            if (direct[direct_count-1]==2){
                rad=quad;
            }
            else if (direct[direct_count-1]==0){
                rad=radUpdate(quad,-1);
            }
        }
        //............................................
        //rad to circle
        if (state[direct_count-1]=="radiusOut"){
            if (direct[direct_count-1]==2){
                quad = quadUpdate(rad, 1);
            }
            else if (direct[direct_count-1]==0){
                quad=rad;
            }
        }
        //.................................................
        //rad to rad
        if (state[direct_count-1]=="radiusIn"){
            rad=radUpdate(rad,direct[direct_count-1]+1);
        }
    }
}
///////////////////////////////////////////////////////////////////////////////////////////
void maze(){
    //maze entrance
    if (state[direct_count]=="enterMaze" && canUpdateStates){
          mazeIn=true;
          vector<int> mazeStates{0,2,1,2,2, 1,0,2};
          direct.insert(direct.begin()+direct_count,mazeStates.begin(),mazeStates.end());
          vector<string> stateNames{"circlePath","radiusIn","radiusOut","circlePath","radiusIn","radiusOut","circlePath"};
          state.insert(state.begin()+direct_count+1,stateNames.begin(),stateNames.end());
          
      }
    //maze location finder
    if (canUpdateStates && mazeIn){
        mazeLoc();
    }
    
    //box check
    const double value = fds->getValue();
    std::cout << "fds"<< value << std::endl;
    if (value<500 && mazeIn && state[direct_count]=="radiusOut" && !colorChecked) {
        boxFound=true;
        colorChecked=true;
        junc=9;
        //direct_count=direct.size();
        //direct.insert(direct.begin()+direct_count,-1);
        checked=true;
        //canUpdateStates=true;
        direct[direct_count]=-1;
        
    }
    else if (value<500 && mazeIn && state[direct_count]=="radiusIn" && !boxFound){
        boxFound=true;
        //direct_count=direct.size();
        //direct_count+=1;
        junc=9;
        found=true;
        direct[direct_count]=-1;
        //canUpdateStates=true;
        //direct.insert(direct.begin()+direct_count,-1);
    }
    
    if (boxFound && colorChecked && canUpdateStates && checked){//exit path
        direct_count=direct.size();
        if (rad==1){
            vector<int> mazeStates{0,0,2};
            direct.insert(direct.end(),mazeStates.begin(),mazeStates.end());
            vector<string> stateNames{"radiusIn","radiusOut","circlePath","ramp"};
            state.insert(state.end(),stateNames.begin(),stateNames.end());
        }
        if (rad==2){
            vector<int> mazeStates{2,2,0};
            direct.insert(direct.end(),mazeStates.begin(),mazeStates.end());
            vector<string> stateNames{"radiusIn","radiusOut","circlePath","ramp"};
            state.insert(state.end(),stateNames.begin(),stateNames.end());
        }
        if (rad==3){
            vector<int> mazeStates{2,0,2};
            direct.insert(direct.end(),mazeStates.begin(),mazeStates.end());
            vector<string> stateNames{"radiusIn","radiusOut","circlePath","ramp"};
            state.insert(state.end(),stateNames.begin(),stateNames.end());
        }
        if (rad==4){
            vector<int> mazeStates{0,2,0};
            direct.insert(direct.end(),mazeStates.begin(),mazeStates.end());
            vector<string> stateNames{"radiusIn","radiusOut","circlePath","ramp"};
            state.insert(state.end(),stateNames.begin(),stateNames.end());
        }
        
        if (even){
            vector<int> mazeStates{0,0,1,1,100};
            direct.insert(direct.end(),mazeStates.begin(),mazeStates.end());
            vector<string> stateNames{"pillar","counted","gate1","gate2","complete"};
            state.insert(state.end(),stateNames.begin(),stateNames.end());
        }
        else{
            vector<int> mazeStates{2,2,1,1,100};
            direct.insert(direct.end(),mazeStates.begin(),mazeStates.end());
            vector<string> stateNames{"pillar","counted","gate1","gate2","complete"};
            state.insert(state.end(),stateNames.begin(),stateNames.end());
        }
        checked=false;
        
        
    }
    else if (boxFound && canUpdateStates && found){
        direct_count=direct.size() ;
        if (rad==1 or rad==4){
            vector<int> mazeStates{2,41,2,2,2};
            direct.insert(direct.end(),mazeStates.begin(),mazeStates.end());
            vector<string> stateNames{"radiusOut","circlePath","circlePath","radiusIn","radiusOut"};
            state.insert(state.end(),stateNames.begin(),stateNames.end());
        }
        else {
            vector<int> mazeStates{2,2,2,2};
            direct.insert(direct.end(),mazeStates.begin(),mazeStates.end());
            vector<string> stateNames{"radiusOut","circlePath","radiusIn","radiusOut"};
            state.insert(state.end(),stateNames.begin(),stateNames.end());
        }
        found = false;
        
        
    }
    
}


///////////////////////////////////////////////////////////////////////////////////////////
void correct(){
    if (state[direct_count]=="counted" && canUpdateStates){
          if (reverse && !even){
              vector<int> mazeStates{-1,0,1,0};
              direct.insert(direct.begin()+direct_count,mazeStates.begin(),mazeStates.end());
              vector<string> stateNames{"reverse","reverse","reverse","reverse"};
              state.insert(state.begin()+direct_count+1,stateNames.begin(),stateNames.end());
              //canUpdateStates = false;
          }
          else if(reverse && even){
              vector<int> mazeStates{-1,2,1,2};
              direct.insert(direct.begin()+direct_count,mazeStates.begin(),mazeStates.end());
              vector<string> stateNames{"reverse","reverse","reverse","reverse"};
              state.insert(state.begin()+direct_count+1,stateNames.begin(),stateNames.end());
          }
      }    
}
///////////////////////////////////////////////////////////////////////////////////////////
void start(){
    if (state[direct_count]=="starting"){
        if (junc!=-1){
            junc=-1;
            leftSpeed = 0.5 * MAX_SPEED;
            rightSpeed = 0.5 * MAX_SPEED;
        }
    }

}
void stop(){
    if (state[direct_count]=="complete"){
        
        if (junc!=-1){
            junc=-1;
            leftSpeed = 0;
            rightSpeed = 0;
        }
    }

}

int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  //Initialization
   ////////////////////////////////////////////////////////////////////////////////////
   //ir_panle
  for (int i = 0; i < 8; i++) {
      ir[i] = robot->getDistanceSensor("ir" + to_string(i));
      ir[i]->enable(TIME_STEP);
  }
  //....................................................

  //Two distance sensors to detect walls
  for (int i = 0; i < 2; i++) {
      ds[i] = robot->getDistanceSensor("ds" + to_string(i));
      ds[i]->enable(TIME_STEP);
  }
  //....................................................

  //two irs to detect junctions
  // for (int i = 0; i < 2; i++) {
      // ts[i] = robot->getDistanceSensor("ts" + to_string(i));
      // ts[i]->enable(TIME_STEP);
  // }
  //....................................................
  
  // fds = robot->getDistanceSensor("fds");
  // fds->enable(TIME_STEP);

  //brakes
  left_brk = robot->getBrake("brake_left");
  right_brk = robot->getBrake("brake_right");
  left_brk->setDampingConstant(0);
  right_brk->setDampingConstant(0);
  //....................................................

  //encorders
  // pos_left = robot->getPositionSensor("pos_left");
  // pos_right = robot->getPositionSensor("pos_right");
  // pos_left->enable(TIME_STEP);
  // pos_right->enable(TIME_STEP);
  // //....................................................

  //led
  // led = robot->getLED("led");
  //....................................................

  //motors
  leftMotor = robot->getMotor("left wheel motor");
  rightMotor = robot->getMotor("right wheel motor");
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0.1 * MAX_SPEED);
  rightMotor->setVelocity(0.1 * MAX_SPEED);
  //....................................................
  // sensorMotor = robot->getMotor("sensor_motor");
  // sliderMotor = robot->getMotor("slider_motor");
  // sensorMotor->setPosition(INFINITY);
  // sliderMotor->setPosition(INFINITY);
  // sensorMotor->setVelocity(0);
  // sliderMotor->setVelocity(0);

  // Main loop:
  while (robot->step(TIME_STEP) != -1) {
      //std::cout<<"junc"<<junc<<std::endl;
      //turning code
      //......................................................
      // if (junc != -1 && turn_command) {
          // sharpTurn(direct[direct_count]);
          // std::cout << "Motor state = turn"<< std::endl;
       // }
      // //for brakes
      // else if (junc != -1) {
          // brakes();
          // std::cout<< "Motor state = brake" << std::endl;
      // }
      // //for line following pid
      // else {
          //dc = 0;
          //std::cout << "Motor state = line follow"<< std::endl;
          pid();
          //std::cout <<"pid_left"<< leftSpeed<< std::endl;
          //wall();
          //std::cout <<"wall_left"<< leftSpeed<< std::endl;
          //junc = juncFind();
      //}
      //......................................................
      //start();
      // maze();
      // pillarCnt();
      // correct();
      // gatesync();
      // stop();
      //std::cout <<"final_left"<< leftSpeed<< std::endl;
      setMotors();
      std::cout << "leftspeed: " << leftSpeed << std::endl; 
      std::cout << "rightspeed: " << rightSpeed << std::endl; 
      std::cout << "############################################################" << std::endl;
      //canUpdateStates=false;
      
      // cout << "quad = "<< quad << " rad = " << rad<< endl;
      
      // cout << "task state = "<< direct_count << " : " << state[direct_count]<< endl;
      // std::cout << "No. of pillars: " << count << std::endl; 
      
      // for(int i=0; i<direct.size(); ++i){
        // if (i==direct_count){
              // std::cout << '*';
        // }
        // std::cout << direct[i] <<" ";
      // }
      // std::cout<< " " << std::endl; 
      
      
      
      // for(int i=0; i<state.size(); ++i){
        // if (i==direct_count){
              // std::cout << '*';
        // }
        // std::cout << state[i] <<" ";
      // }
      // std::cout<< " " << std::endl; 
      
      
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
