#include "vex.h"
#include <iostream>

/********** Functions for Driver Control **********/

static void runIntake(){

  Intake.setVelocity(60, percent);

  if (Controller1.ButtonA.pressing()){
    Intake.spin(forward);
    //DigitalOutA.set(true);
  }
  if (Controller1.ButtonB.pressing()){
    Intake.spin(reverse);
    //wait(100, msec);
    //DigitalOutA.set(false);
  }
  if (Controller1.ButtonUp.pressing()){
    Intake.stop();
  }
}

static void runCataput(){

  Catapult.setVelocity(100, percent);
  if (Controller1.ButtonL1.pressing()){
  
  }
  if (Controller1.ButtonL2.pressing()){
    Catapult.spin(forward);
  }
}

static void tankDrive(){
  LeftFront.spin(forward, Controller1.Axis3.position(), pct);
  LeftStack.spin(forward, Controller1.Axis3.position(), pct);
  LeftBack.spin(forward, Controller1.Axis3.position(), pct);
  RightFront.spin(forward, Controller1.Axis2.position(), pct);
  RightStack.spin(forward, Controller1.Axis2.position(), pct);
  RightStack.spin(forward, Controller1.Axis2.position(), pct);
}

/********** Driver Control Function **********/

void driverControl(){
  while (true){
    runIntake();
    tankDrive();
    

    wait(20, msec);
  }
}