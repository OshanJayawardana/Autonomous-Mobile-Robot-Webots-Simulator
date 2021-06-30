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
//global variables
DistanceSensor* ir[8]; //Ir panel
DistanceSensor* ds[2]; //Two distance sensors to detect wall.
Motor* frontLeftMotor;
Motor* backLeftMotor;
Motor* frontRightMotor;
Motor* backRightMotor;

//initial PID values
double p = 0;
double i = 0;
double d = 0;
double lastError = 0;

////////////////////////////////////////////////////////

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

// PID control
void pid() {
    // initializing the PID coefficients
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

    for (double item : junValues)
    std::cout << item << ", ";
    cout << endl;

    // standard deviation of the ir array
    double sd = 0;

    for (int j = 0; j < 8; j++) {
        sd += (irValues[j] - irSum / 8)* (irValues[j] - irSum / 8);
    }

    sd = pow(sd / 8, 0.5);

    for (int j = 0; j < 8; j++) {
        irValues[j] = (irValues[j] - irSum / 8) / (sd + 0.0001);
    }
    for (double item : irValues)
    std::cout << item << ", ";
    cout << endl;
    // now the ir readings are normalized to a mean of 0 and standard deviation of 1

    // variable for storing the position
    double pos = 0;

    for (int j = 0; j < 4; j++) {
        pos += irValues[j] * (-j + 4) + irValues[7 - j] * (-4 + j);
    }

    double error = 0.0 - pos;
    p = error;
    i = i + p;

    if (i > 100){
        i = 100;
    }
    else if(i < -100){
        i = -100;
    }

    d = error - lastError;
    lastError = error;
    double motorSpeed = kp * p + ki * i + kd * d;
    cout << "motor speed: " << motorSpeed << endl;

    leftSpeed = -0.5 * MAX_SPEED + motorSpeed;
    rightSpeed = -0.5 * MAX_SPEED - motorSpeed;
};
//..........................................................................

//Wall following
void wallFollowing(){
    if (leftWall){
        cout << "left wall" << endl;
        if (leftDsValue > 400){
            cout << "turn left" <<endl;
            leftSpeed = MAX_SPEED * 0.5;
            rightSpeed = -MAX_SPEED * 0.5;
        }
        else if(leftDsValue < 300){
            cout << "turn right" << endl;
            rightSpeed = MAX_SPEED * 0.5;
            leftSpeed = -MAX_SPEED * 0.5;
        }
        else{
            leftSpeed = -MAX_SPEED * 0.5;
            rightSpeed = -MAX_SPEED * 0.5;
        }
    }

    else if (rightWall){
        cout << "right wall" << endl;
        if (rightDsValue > 400){
            cout << "turn right" <<endl;
            rightSpeed = MAX_SPEED * 0.5;
            leftSpeed = -MAX_SPEED * 0.5;
        }
        else if(rightDsValue < 300){
            cout << "turn left" << endl;
            leftSpeed = MAX_SPEED * 0.5;
            rightSpeed = -MAX_SPEED * 0.5;
        }
        else{
            leftSpeed = -MAX_SPEED * 0.5;
            rightSpeed = -MAX_SPEED * 0.5;
        }
    }

}

//No line
void noLine(){
    cout << "no line" << endl;
    leftSpeed = -MAX_SPEED * 0.5;
    rightSpeed = -MAX_SPEED * 0.5;
}

//wallCheck
void wall() {
    leftDsValue = ds[0]->getValue();
    rightDsValue = ds[1]->getValue();

    cout << "left ds: " << leftDsValue << endl;
    cout << "right ds: " << rightDsValue << endl;

    leftWall = leftDsValue < 1000;
    rightWall = rightDsValue < 1000;

    ///////////////////////////////
    //for checking whether there is a line
    bool cond = true;
    for (int j = 0; j < 8; j++) {
        if (junValues[j] > 400) {
            cond = false;
            //cout << "hey" << endl;
        }
    }

    //cout << cond << endl;

    //condition for wall following
    if (leftWall or rightWall) {
        wallFollowing();
    }
    else if (cond) {
        noLine();
    }


    cout << leftSpeed << endl;
    cout << rightSpeed << endl;
}

//set motors
void setMotors(){
    frontLeftMotor->setVelocity(leftSpeed);
    backLeftMotor->setVelocity(leftSpeed);
    frontRightMotor->setVelocity(rightSpeed);
    backRightMotor->setVelocity(rightSpeed);
}


////////////////////////////////////////////////////////////////////////////////////



int main(int argc, char **argv) {
    // create the Robot instance.
    Robot *robot = new Robot();

    //Initialization
    ////////////////////////////////////////////////////////////////////////////////////
    //IR panel
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

    //motors
    frontLeftMotor = robot->getMotor("front_left_motor");
    frontRightMotor = robot->getMotor("front_right_motor");
    backLeftMotor = robot->getMotor("back_left_motor");
    backRightMotor = robot->getMotor("back_right_motor");

    frontLeftMotor->setPosition(INFINITY);
    frontRightMotor->setPosition(INFINITY);
    backLeftMotor->setPosition(INFINITY);
    backRightMotor->setPosition(INFINITY);

    frontLeftMotor->setVelocity(0.0);
    frontRightMotor->setVelocity(0.0);
    backLeftMotor->setVelocity(0.0);
    backRightMotor->setVelocity(0.0);
    //....................................................


    // Main loop:
    while (robot->step(TIME_STEP) != -1) {
        
        pid();
        wall();
        setMotors();

    };

    // Enter here exit cleanup code.

    delete robot;
    return 0;
}