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

static void driveForward(double factor, double tolerance, double minimumSpeed, double target){
  
  double error = target;
  double total = 0;
  LeftStack.setPosition(0, turns);
  
  while (fabs(error) > tolerance){
    error = target - LeftStack.position(turns) * 3.25 * M_PI/2;
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
}

static void driveReverse(double factor, double tolerance, double minimumSpeed, double target){
  
  double error = -target;
  double total = 0;
  LeftStack.setPosition(0, turns);
  
  while (fabs(error) > tolerance){
    error = -target + LeftStack.position(turns) * 3.25 * M_PI/2;
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
}

static void turnClockwise(double factor, double tolerance, double minimumSpeed, double target){
  
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
}

static void turnCounterClockwise(double factor, double tolerance, double minimumSpeed, double target){
  
  double error = -target;
  double total = 0;
  Inertial.setRotation(0, degrees);
  
  while (fabs(error) > tolerance){
    error = -target + Inertial.rotation(degrees);
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
}

static void intake(){
  //DigitalOutA.set(true);
  Intake.spin(forward, 350, rpm);
  wait(50, msec);
}

static void outake(){
  Intake.spin(reverse, 350, rpm);
  //wait(100, msec);
  //DigitalOutA.set(true);
  wait(50, msec);
}


static Auton currentAuton = AutonNone;

static void autonSelector(){
  int runningSelector = true;
  while (runningSelector == true){
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
  driveForward(1, 0.25, 25, 52);
  turnClockwise(1, 0.5, 25, 90);
  outake();
  driveForward(1, 0.25, 10, 10);
}

void runAutonRightRisky(){
  //Score Match Load
  driveForward(1.5, 0.25, 40, 58);
  turnClockwise(1, 0.3, 25, 90);
  outake();
  driveForward(1, 0.25, 10, 10);

  //Pick Up Triball One and Score It
  turnCounterClockwise(1, 0.5, 25, 180);
  intake();
  driveForward(1, 0.25, 40, 24);
  turnCounterClockwise(1, 0.5, 25, 180);
  driveForward(1, 0.25, 40, 20);
  outake();
  driveForward(1, 0.25, 10, 10);

  //Pick Up Triball Two and Score It
  turnClockwise(1, 0.25, 25, 180);
  intake();
  driveForward(1, 0.25, 40, 48);
  turnClockwise(1, 0.5, 25, 180);
  driveForward(1, 0.25, 40, 44);
  outake();
  driveForward(1, 0.25, 10, 10);

  //Pick Up Triball Three and Score It
  turnCounterClockwise(1, 0.5, 25, 30);
  intake();
  driveForward(1, 0.25, 40, 50);
  turnCounterClockwise(1, 0.5, 25, 180);
  driveForward(1, 0.25, 40, 46);
  outake();
  driveForward(1, 0.25, 10, 10);
}

/********** Pre Auton **********/

void preAuton(){
  Inertial.calibrate(5000);
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