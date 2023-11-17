#include "vex.h"
#include <iostream>

/********** Functions for Driver Control **********/

static void setSpeeds(){
  Intake.setVelocity(80, percent);
  Catapult.setVelocity(50, percent);
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

static void runDropIntake(){
  if (Controller1.ButtonY.pressing()){
    IntakePiston.set(true);
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

static void runBlocker(){
  static bool pistonExtended;
  static bool buttonPressed;

  if (Controller1.ButtonX.pressing() && !buttonPressed){ //Button Pressed
    buttonPressed = true;
    pistonExtended = !pistonExtended;
    Blocker.set(pistonExtended);
  }
  if (!Controller1.ButtonX.pressing() && buttonPressed){ //Button Released
    buttonPressed = false;
  }
}

static void runWings(){
  static bool pistonExtended;
  static bool buttonPressed;

  if (Controller1.ButtonR2.pressing() && !buttonPressed){ //Button Pressed
    buttonPressed = true;
    pistonExtended = !pistonExtended;
    Wings.set(pistonExtended);
  }
  if (!Controller1.ButtonR2.pressing() && buttonPressed){ //Button Released
    buttonPressed = false;
  }}

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
    runDropIntake();
    runCatapult();
    runBlocker();
    runWings();

    wait(20, msec);
  }

}