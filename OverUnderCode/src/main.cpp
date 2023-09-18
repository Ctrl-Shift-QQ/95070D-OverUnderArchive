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
// LeftFront            motor         16              
// LeftBack             motor         19              
// LeftStack            motor         18              
// RightFront           motor         1               
// RightBack            motor         14              
// RightStack           motor         13              
// Controller1          controller                    
// Inertial             inertial      11              
// Intake               motor         17              
// DigitalOutA          digital_out   A               
// Catapult             motor         2               
// LimitSwitch          limit         B               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <iostream>
#include "DriverControl.h"
#include "Autonomous.h"

using namespace vex;

competition Competition;

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  //Competition.drivercontrol(driverControl);
  //Competition.autonomous(autonomous);

  Controller1.Screen.clearScreen();
  calibrate(3);

  if (Controller1.ButtonR2.pressing()){
    Controller1.Screen.print("Running Driver Control");
    wait(1, sec);
    driverControl();
  }
  else{
    Controller1.Screen.print("Running Auton");
    wait(1, sec);
    testAuton(AutonRightRisky);
  }
  
  //preAuton();
}

