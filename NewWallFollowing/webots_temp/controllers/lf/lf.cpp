#include <webots/Robot.hpp>

// Added a new include file
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <iostream>

#define TIME_STEP 64

#define MAX_SPEED 6.28
// All the webots classes are defined in the "webots" namespace
using namespace webots;

int main(int argc, char **argv) {
 Robot *robot = new Robot();

 // get the motor devices
 Motor *leftMotor = robot->getMotor("left wheel motor");
 Motor *rightMotor = robot->getMotor("right wheel motor");
 // set the target position of the motors
 leftMotor->setPosition(INFINITY);
 rightMotor->setPosition(INFINITY);
 leftMotor->setVelocity(0.0);
 rightMotor->setVelocity(0.0);
 
 DistanceSensor *ir[8];
 char ir_Names[8][4] = {
   "ir0", "ir1", "ir2", "ir3",
   "ir4", "ir5", "ir6", "ir7"
  };
 
  for (int i = 0; i < 8; i++) {
    ir[i] = robot->getDistanceSensor(ir_Names[i]);
    ir[i]->enable(TIME_STEP);
  }
  
 double irv[8];
 double kp=0.1;
 double ki=0.1;
 double kd=0;
 int p,d,i,lp;
 
 while (robot->step(TIME_STEP) != -1){
  double sum=0;
  for (int i = 0; i < 8 ; i++){
   irv[i] = ir[i]->getValue();
   sum = sum + irv[i];
  }
  //std::cout << irv[0] << std::endl;
  //std::cout << sum << std::endl;
  //double lv = irv[0];
  //double rv = irv[7];
  
  for (int i = 0; i < 8 ; i++){
   irv[i] = irv[i]/sum;
  }
  
  int v;
  v = (7*irv[0] + 6*irv[1] + 5*irv[2] + 4*irv[3] + 3*irv[4] + 2*irv[5] + 1*irv[6] + 0*irv[7]);
  
  //double lv = irv[0];
  //double rv = irv[7];
  
  
  p=v-3.5;
  d=p-lp;
  i=i+p;
  lp=p;
  int power_difference = p*kp + i*ki + d*kd;
  //std::cout << power_difference << std::endl;
  if (power_difference > MAX_SPEED/2)
    power_difference=MAX_SPEED/2;
  if (power_difference < -MAX_SPEED/2)
    power_difference= -MAX_SPEED/2;
    
  
  double left_speed = MAX_SPEED*0.5 + power_difference;
  double right_speed = MAX_SPEED*0.5 - power_difference;
  
  //if (lv>rv && 6<lv && lv<15){
    //left_speed= -0.125*MAX_SPEED;
  //}
  //else if (rv>lv && 6<rv && rv<15){
    //right_speed= -0.125*MAX_SPEED;
  //}
  
  leftMotor->setVelocity(left_speed);
  rightMotor->setVelocity(right_speed);
}
 delete robot;

 return 0;
}