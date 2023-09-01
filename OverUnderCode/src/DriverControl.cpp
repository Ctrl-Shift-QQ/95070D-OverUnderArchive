#include "vex.h"
#include <iostream>

void runIntake(double intakeVelocity){
  Intake.setVelocity(intakeVelocity, rpm);
  
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