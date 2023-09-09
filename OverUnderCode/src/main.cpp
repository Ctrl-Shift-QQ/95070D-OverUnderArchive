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
// LeftFront            motor         20              
// LeftBack             motor         19              
// LeftStack            motor         18              
// RightFront           motor         11              
// RightBack            motor         12              
// RightStack           motor         13              
// Controller1          controller                    
// Inertial             inertial      15              
// Intake               motor         17              
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

