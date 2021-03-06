#include <webots/Robot.hpp>
#include <iostream>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <cmath>
#include <ctime>
#include <chrono>
#include <thread>


#define TIME_STEP 32

using namespace webots;
using namespace std::this_thread;
using namespace std::chrono;

void expose_sharpir();
void detect_color();
void pickup();

Robot *robot = new Robot();
Motor *motor = robot->getMotor("slider_motor");
//Motor *motor2 = robot->getMotor("right_arm_motor");
Motor *motor3 = robot->getMotor("left_arm_motor");
Motor *motor4 = robot->getMotor("sensor_motor");
Motor *motorwl = robot->getMotor("right wheel motor");
Motor *motorwr = robot->getMotor("left wheel motor");
DistanceSensor *sensor = robot->getDistanceSensor("fds");

int main(){
  sensor->enable(TIME_STEP);
  bool detected = false;
  while(robot->step(32)!=-1){
    const double value = sensor->getValue();
    std::cout << "Sensor value is : " << value << std::endl;
    if (detected){
      pickup();
    }
    if (value > 1300) {
      detected = true;
      motorwl->setVelocity(0);
      motorwr->setVelocity(0);
      motorwl->setPosition(0);
      motorwr->setPosition(0);
      continue;
    }
    else if(!(detected)) {
      //expose_sharpir();
      motorwl->setVelocity(3);
      motorwr->setVelocity(3);
      motorwl->setPosition(INFINITY);
      motorwr->setPosition(INFINITY);
    }
  }
}
/*
int main(){
  sensor->enable(TIME_STEP);
  std::time_t initial_time = time(NULL);
  std::cout << "Initial time : " << initial_time << std::endl;
  while(robot->step(32)!=-1){
    detect_color();
    std::time_t current_time;
    current_time = time(NULL);
    std::time_t temp_time = current_time - initial_time;
    std::cout << temp_time << std::endl;
    //motor4->setPosition(0);
    motorwl->setVelocity(0);
    motorwr->setVelocity(0);
    std::cout << "before lifting" << std::endl;
    std::cout << "after lifting" << std::endl;
    
    if (temp_time<2){
        expose_sharpir();
        }
    else if(2<=temp_time && temp_time<=5){
      //motor2->setPosition(-M_PI/6);
      //motor2->setVelocity(3);
      motorwl->setVelocity(1);
      motorwr->setVelocity(1);
      motorwl->setPosition(M_PI/2);
      motorwr->setPosition(M_PI/2);
      motor3->setPosition(M_PI/6);
      motor3->setVelocity(3);
    }
    else if(5<=temp_time && temp_time<=7){
      //motor2->setPosition(M_PI/24);
      //motor2->setVelocity(3);
      motor3->setPosition(-M_PI/24);
      motor3->setVelocity(3);
    }
    else if(7<temp_time && temp_time<=9){
      motor->setVelocity(2);
      motor->setPosition(0.09);
    }
    else if(9<temp_time && temp_time<=12){
      motor4->setPosition(-M_PI/2);
    }
    else if(12<temp_time && temp_time<=15){
      motor4->setPosition(0);
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
*/

void expose_sharpir(){
  motor->setVelocity(1);
  motor->setPosition(0.09);
  motor4->setPosition(-M_PI/2);
}
  
void detect_color(){
  const double value = sensor->getValue();
  std::cout << "sensor value : " << value << std::endl;
}

void pickup(){
  std::time_t initial_time = time(NULL);
  while(robot->step(32)!=-1){
    std::cout << "Initial time : " << initial_time << std::endl;
    std::time_t current_time;
    current_time = time(NULL);
    std::time_t temp_time = current_time - initial_time;
    std::cout << "temp_time :" << temp_time << std::endl;
    if(0<=temp_time && temp_time<=3){
      motor->setVelocity(0.1);
      motor->setPosition(0.09);
      motor3->setPosition(M_PI/24);
      motor3->setVelocity(3.0);
    }
    else if(3<temp_time && temp_time<5){
      motorwl->setVelocity(1.0);
      motorwr->setVelocity(1.0);
      motorwl->setPosition(INFINITY);
      motorwr->setPosition(INFINITY);
    }
    else if(5<=temp_time && temp_time<=7){
      motorwl->setVelocity(0.0);
      motorwr->setVelocity(0.0);
      motor->setVelocity(0.1);
      motor->setPosition(0.0);
    }
    else if(7<temp_time && temp_time<=9){
      motor3->setVelocity(1.0);
      motor3->setPosition(-M_PI/24);
      
    }
    else if(9<temp_time && temp_time<=12){
      motor->setVelocity(0.1);
      motor->setPosition(0.09);
    }
    else if(12<temp_time && temp_time<=16){
      motor4->setPosition(-M_PI/2);
    }
    else if(16<temp_time && temp_time<=18){
      motor4->setPosition(0);
    }
    else if(18<temp_time && temp_time<=20){
      motor->setVelocity(0.1);
      motor->setPosition(0.0);
    }
    else if(20<temp_time && temp_time<=21){
      motor3->setPosition(M_PI/24);
      motor3->setVelocity(3.0);
    }
    if(temp_time > 20){
      break;
    }
  }
  /*std::time_t initial_time = time(NULL);
  std::cout << "Initial time : " << initial_time << std::endl;
  std::time_t current_time;
  current_time = time(NULL);
  std::time_t temp_time = current_time - initial_time;
  std::cout << temp_time << std::endl;
  if(0<=temp_time && temp_time<=3){
    motor->setVelocity(0.1);
    motor->setPosition(0.09);
  }
  if(temp_time > 20){
    initial_time = current_time;
  }*/
  return;
}