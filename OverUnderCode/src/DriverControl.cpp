#include "vex.h"
#include "DriverControl.h"
#include <iostream>

/********** Functions for Driver Control **********/

static void setSpeeds(){
  Intake.setVelocity(80, percent);
  Kicker.setVelocity(57, percent);
}

static void runIntake(){ //Button A for intake, Button B for outake, Button Up for stop
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

static void runKicker(){ //Button L2 to run kicker
  if (Controller1.ButtonL2.pressing()){
    Kicker.spin(forward);
  }
  else{
    Kicker.stop();
  }
}

static void runWings(){ //Button R2 to toggle wings
  static bool pistonExtended;
  static bool buttonPressed;

  if (Controller1.ButtonR2.pressing() && !buttonPressed){ //Button pressed
    buttonPressed = true;
    pistonExtended = !pistonExtended; //Changes piston direction to opposite
    LeftWing.set(pistonExtended);
    RightWing.set(pistonExtended);
  }
  if (!Controller1.ButtonR2.pressing() && buttonPressed){ //Button released
    buttonPressed = false;
  }
}

static void runDrive(){ //Tank drive, button X to toggle speed
  static double driveSpeed = 1;
  static bool slowDrive;
  static bool buttonPressed;

  if (Controller1.ButtonX.pressing() && !buttonPressed){ //Button pressed
    buttonPressed = true;
    slowDrive = !slowDrive;
    if (slowDrive){
      driveSpeed = 0.2;
    }
    else{
      driveSpeed = 1;
    }
  }
  if (!Controller1.ButtonX.pressing() && buttonPressed){ //Button released
    buttonPressed = false;
  }

  LeftDrive.spin(forward, driveSpeed * Controller1.Axis3.position(percent), percent); //Base left drive speed off of left joystick
  RightDrive.spin(forward, driveSpeed * Controller1.Axis2.position(percent), percent); //Base right drive speed off of Right joystick
}

/********** Driver Control Function **********/
void driverControl(){
  setSpeeds();
  while (true){
    runDrive();
    runIntake();
    runKicker();
    runWings();

    wait(20, msec); //Delay prevents brain from crashing
  }
}