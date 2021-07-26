#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <iostream>
#include <webots/Camera.hpp>
#include <algorithm>
#include <initializer_list>

#define TIME_STEP 64
#define MAX_SPEED 6.28

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

Motor *wheels[4];
DistanceSensor *ds[2];
Camera *cm;

int r;
int g;
int b;
int width;
int height;

int colorDetect() {

  const unsigned char *image = cm->getImage();
  width = cm->getWidth();
  height = cm->getHeight();
  
  r = cm->imageGetRed(image, width, (int)(width/2), (int)(height/2));
  g = cm->imageGetGreen(image, width, (int)(width/2), (int)(height/2));
  b = cm->imageGetBlue(image, width, (int)(width/2), (int)(height/2));

  int m = std::max({r, g, b});

  if (m==r) {
    return 1;
  }
  if (m==g) {
    return 2;
  }
  if (m==b) {
    return 3;
  }
  
}

// entry point of the controller
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();
  
  //camera
  cm = robot->getCamera("camera");
  cm->enable(TIME_STEP);
  
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
  
  while (robot->step(TIME_STEP) != -1) {

    std::cout << "Value: " << colorDetect() << std::endl;

  }
  delete robot;
  return 0;  // EXIT_SUCCESS
}