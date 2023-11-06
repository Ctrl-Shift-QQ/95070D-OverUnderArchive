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
  // Competition.drivercontrol(driverControl);
  // Competition.autonomous(autonomous);

  // preAuton();

  // while(true){
  //   wait(20, msec);
  // }
  calibrateInertial(3);
  testAuton(AutonRightAWP);
}

