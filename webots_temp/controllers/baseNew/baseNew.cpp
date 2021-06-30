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


#define TIME_STEP 64
#define MAX_SPEED 6.28
using namespace webots;
using namespace std;

////////////////////////////////////////////////////////////////////////////////////
//Global Variables
DistanceSensor* ir[8]; //ir_panle
DistanceSensor* ts[2]; //two irs to detect junctions
DistanceSensor* ds[2]; //Two distance sensors to detect wall.
Brake* left_brk;
Brake* right_brk;
PositionSensor* pos_left;
PositionSensor* pos_right;
LED* led;
Motor* leftMotor;
Motor* rightMotor;

//initial pid values
double p = 0;
double i = 0;
double d = 0;
double lastError = 0;
//....................................................

double dc = 0; //damping coeficient

//junction identifying parameters
bool turn_command = false;
vector<double> pos_lst;
int junc = -1;
int direct[]={0,2,2,2,1,0,1, 2, 0, 2, 2, 1, 1, 1, 2, 2, 1, 0}; //direct = [1, 0];
int direct_count = 0;
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

void pid() {
    // initializing the PID coefficients
    //float kp = 0.14;
    //float ki = 0.001;
    float kp = 0.14;
    float ki = 0.001;
    float kd = 0.0001;

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

    if (i > 100) {
        i = 100;
    }
    else if (i < -100) {
        i = -100;
    }

    d = error - lastError;
    lastError = error;
    double motorSpeed = kp * p + ki * i + kd * d;
    //cout << "motor speed: " << motorSpeed << endl;

    leftSpeed = 0.5 * MAX_SPEED - motorSpeed;
    rightSpeed = 0.5 * MAX_SPEED + motorSpeed;
};
//..........................................................................

//Wall following
void wallFollowing() {
    if (leftWall) {
        cout << "left wall" << endl;
        if (leftDsValue > 400) {
            cout << "turn left" << endl;
            leftSpeed = -MAX_SPEED * 0.5;
            rightSpeed = MAX_SPEED * 0.5;
        }
        else if (leftDsValue < 300) {
            cout << "turn right" << endl;
            rightSpeed = -MAX_SPEED * 0.5;
            leftSpeed = MAX_SPEED * 0.5;
        }
        else {
            leftSpeed = MAX_SPEED * 0.5;
            rightSpeed = MAX_SPEED * 0.5;
        }
    }

    else if (rightWall) {
        cout << "right wall" << endl;
        if (rightDsValue > 400) {
            cout << "turn right" << endl;
            rightSpeed = -MAX_SPEED * 0.5;
            leftSpeed = MAX_SPEED * 0.5;
        }
        else if (rightDsValue < 300) {
            cout << "turn left" << endl;
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
    cout << "no line" << endl;
    leftSpeed = MAX_SPEED * 0.5;
    rightSpeed = MAX_SPEED * 0.5;
}

void wall() {
    leftDsValue = ds[0]->getValue();
    rightDsValue = ds[1]->getValue();

    cout << "left ds: " << leftDsValue << endl;
    cout << "right ds: " << rightDsValue << endl;

    leftWall = leftDsValue < 1000;
    rightWall = rightDsValue < 1000;

    ///////////////////////////////
    //for checking whether there is a line
    bool cond = false;
    for (int j = 0; j < 8; j++) {
        if (junValues[j] < 400) {
            cond = true;
            //cout << "hey" << endl;
        }
    }

    cout << "cond" << cond << endl;

    //condition for wall following
    if ((leftWall or rightWall) && !cond) {
        wallFollowing();
    }
    else if (!cond) {
        noLine();
    }


    cout << leftSpeed << endl;
    cout << rightSpeed << endl;
}

//junction identificationh
int juncFind() {
    double ir_left = ts[0]->getValue();
    double ir_right = ts[1]->getValue();
    bool left = ir_left < 250;
    bool right = ir_right < 250;
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
        hardLength = 35.0;
        std::cout << "turning left"<<std::endl;
        leftSpeed = 0 * MAX_SPEED;
        rightSpeed = 0.5 * MAX_SPEED;
    }
    else if (turn == 1) {
        hardLength = 5.0;
        std::cout << "going forward"<<std::endl;
        leftSpeed = 0.5 * MAX_SPEED;
        rightSpeed = 0.5 * MAX_SPEED;
    }
    else if (turn == 2) {
        hardLength = 35.0;
        std::cout << "turning right"<<std::endl;
        leftSpeed = 0.5 * MAX_SPEED;
        rightSpeed = 0 * MAX_SPEED;
    }
    else {
        cout << "wrong input. enter a value from 0-2";
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
  for (int i = 0; i < 2; i++) {
      ts[i] = robot->getDistanceSensor("ts" + to_string(i));
      ts[i]->enable(TIME_STEP);
  }
  //....................................................

  //brakes
  left_brk = robot->getBrake("brake_left");
  right_brk = robot->getBrake("brake_right");
  left_brk->setDampingConstant(0);
  right_brk->setDampingConstant(0);
  //....................................................

  //encorders
  pos_left = robot->getPositionSensor("pos_left");
  pos_right = robot->getPositionSensor("pos_right");
  pos_left->enable(TIME_STEP);
  pos_right->enable(TIME_STEP);
  //....................................................

  //led
  led = robot->getLED("led");
  //....................................................

  //motors
  leftMotor = robot->getMotor("left wheel motor");
  rightMotor = robot->getMotor("right wheel motor");
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0.1 * MAX_SPEED);
  rightMotor->setVelocity(0.1 * MAX_SPEED);
  //....................................................


  // Main loop:
  while (robot->step(TIME_STEP) != -1) {
      std::cout<<"junc"<<junc<<std::endl;
      //turning code
      if (junc != -1 && turn_command) {
          sharpTurn(direct[direct_count]);
          std::cout << "turn"<< std::endl;
       }
      //for brakes
      else if (junc != -1) {
          brakes();
          std::cout<< "brake" << std::endl;
      }
      //for line following pid
      else {
          dc = 0;
          std::cout << "pid"<< std::endl;
          pid();
          wall();
          junc = juncFind();
      }
      setMotors();
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
