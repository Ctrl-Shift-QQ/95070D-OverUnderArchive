#include "vex.h"
#include <iostream>

void runIntake(double intakeVelocity){
  Intake.setVelocity(intakeVelocity, rpm);
  
  if (Controller1.ButtonA.pressing()){
    Intake.spin(forward);
    DigitalOutA.set(true);
  }
  if (Controller1.ButtonB.pressing()){
    Intake.spin(reverse);
    wait(100, msec);
    DigitalOutA.set(false);
  }
  if (Controller1.ButtonUp.pressing()){
    Intake.stop();
  }
}

void tankDrive(){
  LeftFront.spin(forward, Controller1.Axis2.position(), pct);
  LeftStack.spin(forward, Controller1.Axis2.position(), pct);
  LeftBack.spin(forward, Controller1.Axis2.position(), pct);
  RightFront.spin(forward, Controller1.Axis3.position(), pct);
  RightStack.spin(forward, Controller1.Axis3.position(), pct);
  RightStack.spin(forward, Controller1.Axis3.position(), pct);
}