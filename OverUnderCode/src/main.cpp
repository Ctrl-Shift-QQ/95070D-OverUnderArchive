/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Jake                                                      */
/*    Created:      Tue Jun 06 2023                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <iostream>
#include "DriverControl.h"
#include "Autonomous.h"

using namespace vex;

// A global instance of competition
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

