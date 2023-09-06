#include "vex.h"
#include <iostream>

/* Uh oh */

int auton = 0;

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

static void intake(double intakeVelocity){
  DigitalOutA.set(true);
  Intake.spin(forward, intakeVelocity, rpm);
  wait(50, msec);
}

static void outake(double intakeVelocity){
  Intake.spin(reverse, intakeVelocity, rpm);
  wait(100, msec);
  DigitalOutA.set(true);
  wait(50, msec);
}

static void autonSelector(){
  int auton = 0;
  int runningSelector = true;
  while (runningSelector == true){
    if (auton == 1){
      Controller1.Screen.print("Auton 1: Safe Right");
    }
    if (auton == 2){
      Controller1.Screen.print("Auton 2: High Scoring Right");
    }
    if (auton == 3){
      Controller1.Screen.print("Auton 3: Safe Left");
    }
    if (auton == 4){
      Controller1.Screen.print("Auton 4: High Scoring Left");
    }
    if (Controller1.ButtonLeft.pressing()){
      if (auton == 1){
        auton = 4;
      }
      else{
        auton--;
      }
      Controller1.Screen.clearScreen();
    }
    if (Controller1.ButtonRight.pressing()){
      if (auton == 4){
        auton = 1;
      }
      else{
        auton++;
      }
      Controller1.Screen.clearScreen();
    }
  }
}

/********** Autons **********/

void autonOne(){
  //Place Preload into Goal
  driveForward(1, 0.25, 25, 48 );
  turnClockwise(1, 0.3, 25, 90);
  outake(350);
  driveForward(1, 0.25, 10, 10);
}

void autonTwo(){
  
}

void autonThree(){
  
}

void autonFour(){
  
}

/********** Pre Auton **********/

void preAuton(){
  Inertial.calibrate(5000);
  DigitalOutA.set(false);
  autonSelector();
}

/********** Auton Function **********/

void autonomous(){
  if (auton == 1){
    autonOne();
  }
  if (auton == 2){
    autonTwo();
  }
  if (auton == 3){
    autonThree();
  }
  if (auton == 4){
    autonFour();
  }
}
