#include "vex.h"
#include <iostream>

/********** Functions for Driver Control **********/

static void setSpeeds(){
  Intake.setVelocity(80, percent);
  Catapult.setVelocity(40, percent);
}

static void runIntake(){
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
  if (Controller1.ButtonL2.pressing()){
    Catapult.spin(forward);
  }
  else{
    Catapult.stop();
  }
}

static void pistonToggle(const controller::button &Button, digital_out Piston){
  static bool pistonExtended;
  static bool ButtonPressed;

  if (Button.pressing() && !ButtonPressed){ //Button Pressed
    ButtonPressed = true;
    pistonExtended = !pistonExtended;
    Piston.set(pistonExtended);
  }
  if (!Button.pressing() && ButtonPressed){ //Button Released
    ButtonPressed = false;
  }
}

static void runBlocker(){
  pistonToggle(Controller1.ButtonX, Blocker);
}

static void runWings(){
  pistonToggle(Controller1.ButtonR2, Wings);
}

static void runDrive(){
  LeftFront.spin(forward, Controller1.Axis3.position(), percent);
  LeftMiddle.spin(forward, Controller1.Axis3.position(), percent);
  LeftBack.spin(forward, Controller1.Axis3.position(), percent);
  RightFront.spin(forward, Controller1.Axis2.position(), percent);
  RightMiddle.spin(forward, Controller1.Axis2.position(), percent);
  RightBack.spin(forward, Controller1.Axis2.position(), percent);
}

/********** Driver Control Function **********/
void driverControl(){
  setSpeeds();
  while (true){
    runDrive();
    runIntake();
    runCatapult();
    runBlocker();
    runWings();

    wait(20, msec);
  }
}