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
// LeftFront            motor         8               
// LeftBack             motor         9               
// LeftStack            motor         10              
// RightFront           motor         1               
// RightBack            motor         2               
// RightStack           motor         3               
// Controller1          controller                    
// Inertial             inertial      6               
// Intake               motor         7               
// DigitalOutA          digital_out   A               
// Catapult             motor         4               
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
  Competition.drivercontrol(driverControl);
  Competition.autonomous(autonomous);
  
  testAuton(AutonRightSafe);

  //preAuton();
}

