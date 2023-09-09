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

static void runCatapult(){

  Catapult.setVelocity(75, percent);

  if (Controller1.ButtonL2.pressing()){
    Catapult.spin(forward);
  }
  
  bool firingCata = false;

  if (!LimitSwitch.pressing()){ 
    Catapult.spin(forward);
  }
  else if (Controller1.ButtonL1.pressing()){
    firingCata = true;
  }
  else{
    Catapult.stop();
  }

  if (LimitSwitch.pressing() && firingCata){
    Catapult.spin(forward);
  }
  else{
    firingCata = false;
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
    runCatapult();

    wait(20, msec);
  }
}