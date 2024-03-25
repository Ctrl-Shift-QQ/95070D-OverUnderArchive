#include "vex.h"
#include "DriverControl.h"
#include <iostream>

/********** Functions for Driver Control **********/

static void setSpeeds(){
  Intake.setVelocity(80, percent);
  Kicker.setVelocity(40, percent);
}

static void runDrive(){ //Tank drive
  LeftDrive.spin(forward, Controller1.Axis3.position(percent), percent); //Base left drive speed off of left joystick
  RightDrive.spin(forward, Controller1.Axis2.position(percent), percent); //Base right drive speed off of Right joystick
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

static void runHang(){ //Button L1 to toggle hang
  static bool pistonExtended;
  static bool buttonPressed;

  if (Controller1.ButtonL1.pressing() && !buttonPressed){ //Button pressed
    buttonPressed = true;
    pistonExtended = !pistonExtended; //Changes piston direction to opposite
    Hang.set(pistonExtended);
  }
  if (!Controller1.ButtonL1.pressing() && buttonPressed){ //Button released
    buttonPressed = false;
  }
}

static void runFrontWings(){ //Button L2 to toggle front wings 
  static bool pistonExtended;
  static bool buttonPressed;

  if (Controller1.ButtonL2.pressing() && !buttonPressed){ //Button pressed
    buttonPressed = true;
    pistonExtended = !pistonExtended; //Changes piston direction to opposite
    LeftFrontWing.set(pistonExtended);
    RightFrontWing.set(pistonExtended);
  }
  if (!Controller1.ButtonL2.pressing() && buttonPressed){ //Button released
    buttonPressed = false;
  }
}

static void runBackWings(){ //Button R2 to toggle back wings 
  static bool pistonExtended;
  static bool buttonPressed;

  if (Controller1.ButtonR2.pressing() && !buttonPressed){ //Button pressed
    buttonPressed = true;
    pistonExtended = !pistonExtended; //Changes piston direction to opposite
    LeftBackWing.set(pistonExtended);
    RightBackWing.set(pistonExtended);
  }
  if (!Controller1.ButtonR2.pressing() && buttonPressed){ //Button released
    buttonPressed = false;
  }
}

/********** Driver Control Function **********/
void driverControl(){
  setSpeeds();
  while (true){
    runDrive();
    runIntake();
    runFrontWings();
    runBackWings();

    wait(20, msec); //Delay prevents brain from crashing
  }
}