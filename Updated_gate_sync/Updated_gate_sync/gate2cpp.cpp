#include <webots/Robot.hpp>
#include <iostream>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <cmath>
#include <ctime>


#define TIME_STEP 32

using namespace webots;

int main(){
  Robot *robot = new Robot();
  Motor *motor = robot->getMotor("gate_motor");
  std::time_t initial_time = time(NULL);
  std::cout << "Initial time : " << initial_time << std::endl;
  while(robot->step(32)!=-1){
    std::time_t current_time;
    current_time = time(NULL);
    std::time_t temp_time = current_time - initial_time;
    std::cout << temp_time << std::endl;
    if(3<=temp_time && temp_time<=13){
      std::cout<<"gate 2 opened"<< std::endl;
      motor->setPosition(-M_PI/2);
    }
    else if(13<temp_time && temp_time<20){
      std::cout<<"gate 2 closed" << std::endl;
      motor->setPosition(0);
    }
    if (temp_time>=20){
      initial_time = current_time;
    }
  }
  delete robot;
  return 0;
  
}