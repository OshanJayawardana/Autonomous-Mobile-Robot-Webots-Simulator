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

// predefined functions
////////////////////////////////////////////////////////////////////////////////////

// pid controll
void pid() {
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
    double p = error;
    double i = i + p;
    double d = error - last_error;
    double last_error = error;
    double motor_speed = kp * p + ki * i + kd * d;
    double left_speed = 0.5 * MAX_SPEED - motor_speed;
    double right_speed = 0.5 * MAX_SPEED + motor_speed;
};
//..........................................................................

//junction identificationh
int juncFind() {
    int junc = -1;
    double ir_left = ts[0]->getValue();
    double ir_right = ts[1]->getValue();
    bool left = ir_left < 250;
    bool right = ir_right < 250;
    if (left && !right) {
        int junc = 0;
    }
    else if (left && right) {
        int junc = 1;
    }
    else if (!left && right) {
        int junc = 2;
    }
    else {
        int junc = -1;
    }
    return junc;
};
//....................................................

//set motors
void setMotors() {
    //setting motor speeds
    leftMotor->setVelocity(left_speed);
    rightMotor->setVelocity(right_speed);

    //setting brakes
    left_brk->setDampingConstant(dc);
    right_brk->setDampingConstant(dc);

    //storing the speed for next loop
    double last_left_speed = left_speed;
    double last_right_speed = right_speed;
};
//....................................................

//sharpTurns
void sharpTurn(int turn) {
    double pos_val = pos_right->getValue();
    if (turn == 2) {
        double pos_val = pos_left->getValue();
    }
    if (abs(pos_val) > 0) {
        pos_lst.push_back(pos_val);
        cout << abs(pos_lst.begin() - pos_lst.end());
        if (abs(pos_lst.begin() - pos_lst.end()) > 5.0) {
            junc = -1;
            vector<double> pos_lst;
            turn_command = 0;
            direct_count += 1;
        }
    }
    if (turn == 0) {
        cout << "turning left";
        double left_speed = 0 * MAX_SPEED;
        double right_speed = 0.5 * MAX_SPEED;
    }
    else if (turn == 1) {
        cout << "going forward";
        double left_speed = 0.5 * MAX_SPEED;
        double right_speed = 0.5 * MAX_SPEED;
    }
    else if (turn == 2) {
        cout << "turning right";
        double left_speed = 0.5 * MAX_SPEED;
        double right_speed = 0 * MAX_SPEED;
    }
    else {
        cout << "wrong input. enter a value from 0-2";
    }
};
//....................................................

//braking
void brakes() {
    cout << "brake";
    double speed = last_right_speed;
    if (last_left_speed > last_right_speed) {
        double speed = last_left_speed;
    }
    if ( speed < 0.5 * MAX_SPEED) {
        double left_speed = 0;
        double right_speed = 0;
        turn_command = 1;
        dc = 0;
    }
    else {
        double left_speed = speed - 1;
        double right_speed = speed - 1;
        dc += 0;
    }
};
////////////////////////////////////////////////////////////////////////////////////

//global variables
DistanceSensor* ir[8]; //ir_panle
DistanceSensor* ts[2]; //two irs to detect junctions
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
double last_error = 0;
//....................................................

double dc = 0; //damping coeficient

//junction identifying parameters
bool turn_command = 0;
vector<double> pos_lst;
int junc = -1;
vector<int> direct{2, 1, 2, 2, 2, 0, 2, 2, 2, 2, 1, 0}; //direct = [1, 0];
int direct_count = 0;
////////////////////////////////////////////////////////

double last_left_speed = 0;
double last_right_speed = 0;
double left_speed;
double right_speed;


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

  //two irs to detect junctions
  for (int i = 0; i < 8; i++) {
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
  leftMotor = robot->getMotor('left wheel motor');
  rightMotor = robot->getMotor('right wheel motor');
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0.1 * MAX_SPEED);
  rightMotor->setVelocity(0.1 * MAX_SPEED);
  //....................................................


  // Main loop:
  while (robot->step(TIME_STEP) != -1) {
      //turning code
      if (junc != -1 && turn_command) {
          //led->set(0);
          sharpTurn(direct.at(direct_count));
       }
      else if (junc != 1) {
          cout << ts[0]->getValue();
          cout << ts[1]->getValue();
          //led->set(0);
          brakes();
      }
      else {
          //led->set(0)
          double dc = 0;
          pid();
          junc = juncFind();
      }
      setMotors();
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
