#include "vex.h"
#include "DriverControl.h"
#include <iostream>

/********** Functions for Driver Control **********/

static void setSpeeds(){
  Intake.setVelocity(80, percent);
  Kicker.setVelocity(57, percent);
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

static void runKicker(){
  if (Controller1.ButtonL2.pressing()){
    Kicker.spin(forward);
  }
  else{
    Kicker.stop();
  }
}

static void runBlocker(){
  static bool pistonExtended;
  static bool buttonPressed;

  if (Controller1.ButtonX.pressing() && !buttonPressed){ //Button pressed
    buttonPressed = true;
    pistonExtended = !pistonExtended; //Changes piston direction to opposite
    Blocker.set(pistonExtended);
  }
  if (!Controller1.ButtonX.pressing() && buttonPressed){ //Button released
    buttonPressed = false;
  }
}

static void runBackWings(){
  static bool pistonExtended;
  static bool buttonPressed;

  if (Controller1.ButtonR2.pressing() && !buttonPressed){ //Button pressed
    buttonPressed = true;
    pistonExtended = !pistonExtended; //Changes piston direction to opposite
    BackWings.set(pistonExtended);
  }
  if (!Controller1.ButtonR2.pressing() && buttonPressed){ //Button released
    buttonPressed = false;
  }
}

static void runBalance(){
  static bool pistonExtended;
  static bool buttonPressed;

  if (Controller1.ButtonY.pressing() && !buttonPressed){ //Button pressed
    buttonPressed = true;
    pistonExtended = !pistonExtended; //Changes piston direction to opposite
    Balance.set(pistonExtended);
  }
  if (!Controller1.ButtonY.pressing() && buttonPressed){ //Button released
    buttonPressed = false;
  }
}

static void runDrive(){ //Tank Drive
  LeftFront.spin(forward, Controller1.Axis3.position(), percent); //Base left drive speed off of left joystick
  LeftBack.spin(forward, Controller1.Axis3.position(), percent);
  LeftStack.spin(forward, Controller1.Axis3.position(), percent);
  RightFront.spin(forward, Controller1.Axis2.position(), percent); //Base right drive speed off of Right joystick
  RightBack.spin(forward, Controller1.Axis2.position(), percent);
  RightStack.spin(forward, Controller1.Axis2.position(), percent); 
}

/********** Driver Control Function **********/
void driverControl(){
  setSpeeds();
  while (true){
    runDrive();
    runIntake();
    runKicker();
    runBackWings();
    runBalance();
    runBlocker();

    wait(20, msec); //Delay prevents brain from crashing
  }
}