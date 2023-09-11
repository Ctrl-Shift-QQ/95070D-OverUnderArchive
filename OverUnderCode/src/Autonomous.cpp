#include "vex.h"
#include <iostream>
#include "Autonomous.h"

/********** Functions for Auton **********/

static int getSign(double input){
  if (input > 0){
    return 1;
  }
  else if (input < 0){
    return -1;
  }
  else{
    return 0;
  }
}

void driveForward(double factor, double tolerance, double minimumSpeed, double target){
  
  double error = target;
  double total = 0;
  LeftFront.setPosition(0, turns);
  
  while (fabs(error) > tolerance){
    error = target - LeftFront.position(turns) * 3.25 * M_PI/2;
    total = error * factor;
    if (fabs(total) < minimumSpeed){
      LeftFront.spin(forward, getSign(total) * minimumSpeed, percent);
      LeftBack.spin(forward, getSign(total) * minimumSpeed, percent);
      LeftStack.spin(forward, getSign(total) * minimumSpeed, percent);
      RightFront.spin(forward, getSign(total) * minimumSpeed, percent);
      RightBack.spin(forward, getSign(total) * minimumSpeed, percent);
      RightStack.spin(forward, getSign(total) * minimumSpeed, percent);
    }
    else {
      LeftFront.spin(forward, error * factor, percent);
      LeftBack.spin(forward, error * factor, percent);
      LeftStack.spin(forward, error * factor, percent);
      RightFront.spin(forward, error * factor, percent);
      RightBack.spin(forward, error * factor, percent);
      RightStack.spin(forward, error * factor, percent);
    }
    wait(15, msec);
  }
  LeftFront.stop(brake);
  LeftBack.stop(brake);
  LeftStack.stop(brake);
  RightFront.stop(brake);
  RightBack.stop(brake);
  RightStack.stop(brake);

  wait(100, msec);
}

void driveReverse(double factor, double tolerance, double minimumSpeed, double target){
  
  double error = target;
  double total = 0;
  LeftFront.setPosition(0, turns);
  
  while (fabs(error) > tolerance){
    error = target + LeftFront.position(turns) * 3.25 * M_PI/2;
    total = error * factor;
    if (fabs(total) < minimumSpeed){
      LeftFront.spin(reverse, getSign(total) * minimumSpeed, percent);
      LeftBack.spin(reverse, getSign(total) * minimumSpeed, percent);
      LeftStack.spin(reverse, getSign(total) * minimumSpeed, percent);
      RightFront.spin(reverse, getSign(total) * minimumSpeed, percent);
      RightBack.spin(reverse, getSign(total) * minimumSpeed, percent);
      RightStack.spin(reverse, getSign(total) * minimumSpeed, percent);
    }
    else {
      LeftFront.spin(reverse, error * factor, percent);
      LeftBack.spin(reverse, error * factor, percent);
      LeftStack.spin(reverse, error * factor, percent);
      RightFront.spin(reverse, error * factor, percent);
      RightBack.spin(reverse, error * factor, percent);
      RightStack.spin(reverse, error * factor, percent);
    }
    wait(15, msec);
  }
  LeftFront.stop(brake);
  LeftBack.stop(brake);
  LeftStack.stop(brake);
  RightFront.stop(brake);
  RightBack.stop(brake);
  RightStack.stop(brake);

  wait(100, msec);
}

void turnClockwise(double factor, double tolerance, double minimumSpeed, double target){
  
  double error = target;
  double total = 0;
  Inertial.setRotation(0, degrees);
  
  while (fabs(error) > tolerance){
    error = target - Inertial.rotation(degrees);
    total = error * factor;
    if (fabs(total) < minimumSpeed){
      LeftFront.spin(forward, getSign(total) * minimumSpeed, percent);
      LeftBack.spin(forward, getSign(total) * minimumSpeed, percent);
      LeftStack.spin(forward, getSign(total) * minimumSpeed, percent);
      RightFront.spin(reverse, getSign(total) * minimumSpeed, percent);
      RightBack.spin(reverse, getSign(total) * minimumSpeed, percent);
      RightStack.spin(reverse, getSign(total) * minimumSpeed, percent);
    }
    else {
      LeftFront.spin(forward, error * factor, percent);
      LeftBack.spin(forward, error * factor, percent);
      LeftStack.spin(forward, error * factor, percent);
      RightFront.spin(reverse, error * factor, percent);
      RightBack.spin(reverse, error * factor, percent);
      RightStack.spin(reverse, error * factor, percent);
    }
    wait(15, msec);
  }
  LeftFront.stop(brake);
  LeftBack.stop(brake);
  LeftStack.stop(brake);
  RightFront.stop(brake);
  RightBack.stop(brake);
  RightStack.stop(brake);

  wait(100, msec);
}

static void turnCounterClockwise(double factor, double tolerance, double minimumSpeed, double target){
  
  double error = target;
  double total = 0;
  Inertial.setRotation(0, degrees);
  
  while (fabs(error) > tolerance){
    error = target + Inertial.rotation(degrees);
    total = error * factor;
    if (fabs(total) < minimumSpeed){
      LeftFront.spin(reverse, getSign(total) * minimumSpeed, percent);
      LeftBack.spin(reverse, getSign(total) * minimumSpeed, percent);
      LeftStack.spin(reverse, getSign(total) * minimumSpeed, percent);
      RightFront.spin(forward, getSign(total) * minimumSpeed, percent);
      RightBack.spin(forward, getSign(total) * minimumSpeed, percent);
      RightStack.spin(forward, getSign(total) * minimumSpeed, percent);
    }
    else {
      LeftFront.spin(reverse, error * factor, percent);
      LeftBack.spin(reverse, error * factor, percent);
      LeftStack.spin(reverse, error * factor, percent);
      RightFront.spin(forward, error * factor, percent);
      RightBack.spin(forward, error * factor, percent);
      RightStack.spin(forward, error * factor, percent);
    }
    wait(15, msec);
  }
  LeftFront.stop(brake);
  LeftBack.stop(brake);
  LeftStack.stop(brake);
  RightFront.stop(brake);
  RightBack.stop(brake);
  RightStack.stop(brake);

  wait(100, msec);
}

static void intake(double waitTime){
  //DigitalOutA.set(true);
  Intake.spin(forward, 350, rpm);
  wait(waitTime, sec);
}

static void outake(double waitTime){
  Intake.spin(reverse, 350, rpm);
  //wait(100, msec);
  //DigitalOutA.set(true);
  wait(waitTime, sec);
}


static Auton currentAuton = AutonNone;

static void autonSelector(){
  int runningSelector = true;
  while (runningSelector){
    if (currentAuton == AutonLeftSafe){
      Controller1.Screen.print("Auton Selected, Safe Left");
    }
    if (currentAuton == AutonLeftRisky){
      Controller1.Screen.print("Auton Selected, Risky Left");
    }
    if (currentAuton == AutonRightSafe){
      Controller1.Screen.print("Auton Selected, Safe Right");
    }
    if (currentAuton == AutonRightRisky){
      Controller1.Screen.print("Auton Selected, Risky Right");
    }
    if (Controller1.ButtonLeft.pressing()){
      if (currentAuton == AutonLeftSafe){
        currentAuton = AutonRightRisky;
      }
      else{
        currentAuton = static_cast<Auton> (static_cast<int> (currentAuton) - 1);
      }
      Controller1.Screen.clearScreen();
    }
    if (Controller1.ButtonRight.pressing()){
      if (currentAuton == AutonRightRisky){
        currentAuton = AutonLeftSafe;
      }
      else{
        currentAuton = static_cast<Auton> (static_cast<int> (currentAuton) + 1);
      }
      Controller1.Screen.clearScreen();
    }
    if (Controller1.ButtonA.pressing()){
      runningSelector = false;
    }
  }
  Controller1.rumble(rumblePulse);
}

/********** Autons **********/

void runAutonLeftSafe(){

}

void runAutonLeftRisky(){
  
}

void runAutonRightSafe(){ 
  //Score Match Load
  driveForward(0.5, 0.5, 25, 52);
  turnClockwise(0.3, 0.25, 10, 90);
  outake(0.5);
  driveForward(1, 0.25, 50, 10);
  Intake.stop();
}

void runAutonRightRisky(){
  //Score Match Load
  driveForward(1, 0.5, 30, 52);
  turnClockwise(0.4, 0.25, 10, 90);
  outake(0.4);
  driveForward(1, 0.25, 50, 10);

  //Pick Up Triball One and Score It
  driveReverse(1, 0.5, 10, 10);
  turnCounterClockwise(0.4, 0.25, 10, 130);
  intake(0.3);
  driveForward(1, 0.5, 30, 18);
  turnClockwise(0.4, 0.25, 10, 130);
  driveForward(1, 0.5, 30, 8);
  outake(0.4);
  driveForward(1, 0.5, 50, 10);
  Intake.stop();

  //Pick Up Triball Two and Score It
  

  //Pick Up Triball Three and Score It
  
}

/********** Pre Auton **********/

void callibrate(double seconds){
  Inertial.calibrate();
  wait(seconds, sec);
}

void preAuton(){
  callibrate(5);
  DigitalOutA.set(false);
  autonSelector();
}

/********** Auton Function **********/

void autonomous(){
  switch (currentAuton){
    case AutonNone: {
      break;
    }
    case AutonLeftSafe: {
      runAutonLeftSafe();
      break;
    }
    case AutonLeftRisky: {
      runAutonLeftRisky();
      break;
    }
    case AutonRightSafe: {
      runAutonRightSafe();
      break;
    }
    case AutonRightRisky: {
      runAutonRightRisky();
      break;
    }
    default: {
      break;
    }
  }
}

void testAuton(Auton testedAuton){
  switch (testedAuton){
    case AutonNone: {
      break;
    }
    case AutonLeftSafe: {
      runAutonLeftSafe();
      break;
    }
    case AutonLeftRisky: {
      runAutonLeftRisky();
      break;
    }
    case AutonRightSafe: {
      runAutonRightSafe();
      break;
    }
    case AutonRightRisky: {
      runAutonRightRisky();
      break;
    }
    default: {
      break;
    }
  }
}