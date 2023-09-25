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

int main(){
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // Competition.drivercontrol(driverControl);
  // Competition.autonomous(autonomous);

  tempCheck(130);
  calibrate(3);

  testAuton(AutonLeftSafe);

  //preAuton();
}
