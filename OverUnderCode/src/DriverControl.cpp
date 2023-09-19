#include "vex.h"
#include <iostream>

/********** Functions for Driver Control **********/

static void runIntake(){

  Intake.setVelocity(75, percent);

  if (Controller1.ButtonA.pressing()){
    Intake.spin(forward);
  }
  if (Controller1.ButtonB.pressing()){
    Intake.spin(reverse);
  }
  if (Controller1.ButtonUp.pressing()){
    Intake.stop();
  }
}

static void runCatapult(){

  Catapult.setVelocity(85, percent);

  if (Controller1.ButtonL2.pressing()){
    Catapult.spin(forward);
  }
  else{
    Catapult.stop();
  }
  
  // bool firingCata = false;

  // if (!LimitSwitch.pressing()){ 
  //   Catapult.spin(forward);
  // }
  // else if (Controller1.ButtonL1.pressing()){
  //   firingCata = true;
  // }
  // else{
  //   Catapult.stop();
  // }

  // if (LimitSwitch.pressing() && firingCata){
  //   Catapult.spin(forward);
  // }
  // else{
  //   firingCata = false;
  // }
}

static void tankDrive(){
  LeftFront.spin(forward, Controller1.Axis3.position(), percent);
  LeftBack.spin(forward, Controller1.Axis3.position(), percent);
  LeftStack.spin(forward, Controller1.Axis3.position(), percent);
  RightFront.spin(forward, Controller1.Axis2.position(), percent);
  RightBack.spin(forward, Controller1.Axis2.position(), percent);
  RightStack.spin(forward, Controller1.Axis2.position(), percent);
}

/********** Driver Control Function **********/
void driverControl(){
  while (true){
    runIntake();
    tankDrive();
    runCatapult();

    wait(20, msec);
  }
}