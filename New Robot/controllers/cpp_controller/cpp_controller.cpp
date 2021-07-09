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
  Motor *motor = robot->getMotor("slider_motor");
  Motor *motor3 = robot->getMotor("left_arm_motor");
  //Motor *motor4 = robot->getMotor("sensor_motor");
  std::time_t initial_time = time(NULL);
  std::cout << "Initial time : " << initial_time << std::endl;
  while(robot->step(32)!=-1){
    std::time_t current_time;
    current_time = time(NULL);
    std::time_t temp_time = current_time - initial_time;
    std::cout << temp_time << std::endl;
    //motor4->setPosition(0);
    if(0<=temp_time && temp_time<=1){
      motor3->setPosition(M_PI/6);
      motor3->setVelocity(3);
    }
    else if(1<=temp_time && temp_time<=2){
      //motor2->setPosition(M_PI/24);
      //motor2->setVelocity(3);
      motor3->setPosition(-M_PI/24);
      motor3->setVelocity(3);
    }
    else if(2<temp_time && temp_time<=5){
      motor->setVelocity(3);
      motor->setPosition(0.03);
    }
    else if(5<temp_time && temp_time<=10){
      //motor4->setPosition(-M_PI/2);
    }
    else if(10<temp_time && temp_time<=15){
      //motor4->setPosition(0);
    }
    else if(15<temp_time && temp_time<18){
      motor->setVelocity(2);
      motor->setPosition(0);
    }
    else if(18<=temp_time && temp_time<20){
      //motor2->setPosition(-M_PI/6);
      //motor2->setVelocity(3);
      motor3->setPosition(M_PI/6);
      motor3->setVelocity(3);
    }
    if (temp_time>=20){
      initial_time = current_time;
    }
  }
  delete robot;
  return 0;
  
}