#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/LED.hpp>
#include <iostream>

#define TIME_STEP 64
#define MAX_SPEED 6.28

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

//global variables
int reverse = 0;
int pc=1;
int count = 0;
int ps=0;
int cs=0;
int pj=0;

Motor *wheels[4];
DistanceSensor *ds[2];
DistanceSensor *ts[2];
LED *led;

void pillarCnt() {

  led->set(0);
   
  //counting pillars
  for (int j = 0; j < 2; j++) {
    if ((ds[j]->getValue())>850.0 && (ds[j]->getValue())<900.0){
      pj=j;
      cs=1;
      if (ps==0) {
          count += 1; 
          led->set(1);
          ps=cs;   
          std::cout << "No. of pillars: " << count << std::endl; 
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
        reverse = 1; 
        pc = 0;
      }
    }
   }

}

// entry point of the controller
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();
  
  //wheels
  char wheelsNames[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
  for (int i = 0; i < 4; i++) {
    wheels[i] = robot->getMotor(wheelsNames[i]);
    wheels[i]->setPosition(INFINITY);
    wheels[i]->setVelocity(0.4*MAX_SPEED);
  }
     
  //ds
  char dsNames[2][10] = {"ds_right", "ds_left"};
  for (int i = 0; i < 2; i++) {
    ds[i] = robot->getDistanceSensor(dsNames[i]);
    ds[i]->enable(TIME_STEP);
  }
  
  //ts
  char tsNames[2][4] = {"ts0", "ts1"};
  for (int i = 0; i < 2; i++) {
    ts[i] = robot->getDistanceSensor(tsNames[i]);
    ts[i]->enable(TIME_STEP);
  }
  
  led = robot->getLED("led2");
  
  while (robot->step(TIME_STEP) != -1) {
    pillarCnt();
    //return 0;
  }
  delete robot;
  return 0;  // EXIT_SUCCESS
}


      

      
      