/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       jake                                                      */
/*    Created:      Tue Jun 06 2023                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// LeftBack             motor         3               
// LeftFront            motor         1               
// LeftStack            motor         2               
// RightFront           motor         8               
// RightBack            motor         10              
// RightStack           motor         9               
// Controller1          controller                    
// Inertial             inertial      20              
// Intake               motor         4               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <iostream>
#include "DriverControl.h"

using namespace vex;

int getSign(double input){
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

void accurateDrive(double factor, double tolerance, double minimumSpeed, double target){
  double error = target;
  double total = 0;
  LeftStack.setPosition(0, turns);
  while (fabs(error) > tolerance){
    error = target - LeftStack.position(turns)*3.25*M_PI/2;
    total = error*factor;
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
  }
  LeftFront.stop(brake);
  LeftBack.stop(brake);
  LeftStack.stop(brake);
  RightFront.stop(brake);
  RightBack.stop(brake);
  RightStack.stop(brake);
}

void clockwiseTurn(double factor, double tolerance, double minimumSpeed, double target){
  double error = target;
  double total = 0;
  Inertial.setRotation(0, degrees);
  while (fabs(error) > tolerance){
    error = target - Inertial.rotation(degrees);
    total = error*factor;
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
  }
  LeftFront.stop(brake);
  LeftBack.stop(brake);
  LeftStack.stop(brake);
  RightFront.stop(brake);
  RightBack.stop(brake);
  RightStack.stop(brake);
}

void counterClockwiseTurn(double factor, double tolerance, double minimumSpeed, double target){
  double error = target;
  double total = 0;
  Inertial.setRotation(0, degrees);
  while (fabs(error) > tolerance){
    error = target - Inertial.rotation(degrees);
    total = error*factor;
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
  }
  LeftFront.stop(brake);
  LeftBack.stop(brake);
  LeftStack.stop(brake);
  RightFront.stop(brake);
  RightBack.stop(brake);
  RightStack.stop(brake);
}

void runIntake(){
  //Clockwise = 1, Counter-clockwise = 2
  int intakeDirection = 0;
  if (Controller1.ButtonA.pressing()){
    if (intakeDirection == 1){
      intakeDirection = 0;
      Intake.stop();
    }
    else{
      intakeDirection = 1;
      Intake.spin(forward);
      vex::task::sleep(250);
    }
  }
  if (Controller1.ButtonB.pressing()){
    if (intakeDirection == 2){
      intakeDirection = 0;
      Intake.stop();
    }
    else{
      intakeDirection = 2;
      Intake.spin(reverse);
      vex::task::sleep(250);
    }
  }
}
int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
}
