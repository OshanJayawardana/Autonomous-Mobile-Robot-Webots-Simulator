#include <webots/Robot.hpp>
#include <iostream>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <cmath>

#define TIME_STEP 32

using namespace webots;

int main(){
  Robot *robot = new Robot();
  Motor *motorR = robot->getMotor("MOTOR1");
  Motor *motorL = robot->getMotor("MOTOR2");
  
  DistanceSensor *sensor = robot->getDistanceSensor("sensor1");
  sensor->enable(TIME_STEP);
  
  while(robot->step(32)!=-1){
    
    const double value = sensor->getValue();
    std::cout << "Sensor value is : " << value << std::endl;
    
    if (value <= 100){
      std::cout << "yes";
      motorR->setVelocity(0.0);
      motorL->setVelocity(0.0);
      motorR->setPosition(0.0);
      motorL->setPosition(0.0);
    }
    else{
      motorR->setPosition(INFINITY);
      motorL->setPosition(INFINITY);
      motorR->setVelocity(5.0);
      motorL->setVelocity(5.0);
    }
    
  }
  delete robot;
  return 0;
  
}