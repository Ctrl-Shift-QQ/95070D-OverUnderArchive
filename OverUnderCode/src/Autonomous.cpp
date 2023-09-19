#include "vex.h"
#include <iostream>
#include "Autonomous.h"

/********** Functions for Auton **********/

void driveForward(double kp, double tolerance, double minimumSpeed, double target){
  
  double leftDriveError = target;
  double rightDriveError = target;
  double leftDriveTotal = 0;
  double rightDriveTotal = 0;

  LeftFront.resetPosition();
  RightFront.resetPosition();
  
  while ((fabs(leftDriveError) + fabs(leftDriveError)) / 2 > tolerance){
    //Left Side
    leftDriveError = target - LeftFront.position(turns) * 3.25 * M_PI / 2;
    leftDriveTotal = leftDriveError * kp;
    if (fabs(leftDriveTotal) < minimumSpeed){
      LeftFront.spin(forward, leftDriveTotal * minimumSpeed, percent);
      LeftBack.spin(forward, leftDriveTotal * minimumSpeed, percent);
      LeftStack.spin(forward, leftDriveTotal * minimumSpeed, percent);
    }
    else {
      LeftFront.spin(forward, leftDriveTotal * kp, percent);
      LeftBack.spin(forward, leftDriveTotal * kp, percent);
      LeftStack.spin(forward, leftDriveTotal * kp, percent);
    }

    //Right Side
    rightDriveError = target - RightFront.position(turns) * 3.25 * M_PI / 2;
    rightDriveTotal = rightDriveError * kp;
    if (fabs(leftDriveTotal) < minimumSpeed){
      RightFront.spin(forward, rightDriveTotal * minimumSpeed, percent);
      RightBack.spin(forward, rightDriveTotal * minimumSpeed, percent);
      RightStack.spin(forward, rightDriveTotal * minimumSpeed, percent);
    }
    else {
      RightFront.spin(forward, rightDriveTotal * kp, percent);
      RightBack.spin(forward, rightDriveTotal * kp, percent);
      RightStack.spin(forward, rightDriveTotal * kp, percent);
    }

    wait(20, msec);
  }
  LeftFront.stop(brake);
  LeftBack.stop(brake);
  LeftStack.stop(brake);
  RightFront.stop(brake);
  RightBack.stop(brake);
  RightStack.stop(brake);

  wait(100, msec);
}

void driveReverse(double kp, double tolerance, double minimumSpeed, double target){
  
  double leftDriveError = target;
  double rightDriveError = target;
  double leftDriveTotal = 0;
  double rightDriveTotal = 0;

  LeftFront.resetPosition();
  RightFront.resetPosition();
  
  while ((fabs(leftDriveError) + fabs(leftDriveError)) / 2 > tolerance){
    //Left Side
    leftDriveError = target + LeftFront.position(turns) * 3.25 * M_PI / 2;
    leftDriveTotal = leftDriveError * kp;

    if (fabs(leftDriveTotal) < minimumSpeed){
      LeftFront.spin(reverse, leftDriveTotal * minimumSpeed, percent);
      LeftBack.spin(reverse, leftDriveTotal * minimumSpeed, percent);
      LeftStack.spin(reverse, leftDriveTotal * minimumSpeed, percent);
    }
    else {
      LeftFront.spin(reverse, leftDriveTotal * kp, percent);
      LeftBack.spin(reverse, leftDriveTotal * kp, percent);
      LeftStack.spin(reverse, leftDriveTotal * kp, percent);
    }
    
    //Right Side
    rightDriveError = target + RightFront.position(turns) * 3.25 * M_PI / 2;
    rightDriveTotal = rightDriveError * kp;

    if (fabs(leftDriveTotal) < minimumSpeed){
      RightFront.spin(reverse, rightDriveTotal * minimumSpeed, percent);
      RightBack.spin(reverse, rightDriveTotal * minimumSpeed, percent);
      RightStack.spin(reverse, rightDriveTotal * minimumSpeed, percent);
    }
    else {
      RightFront.spin(reverse, rightDriveTotal * kp, percent);
      RightBack.spin(reverse, rightDriveTotal * kp, percent);
      RightStack.spin(reverse, rightDriveTotal * kp, percent);
    }

    wait(20, msec);
  }
  LeftFront.stop(brake);
  LeftBack.stop(brake);
  LeftStack.stop(brake);
  RightFront.stop(brake);
  RightBack.stop(brake);
  RightStack.stop(brake);

  wait(100, msec);
}

void turnClockwise(double kp, double kd, double tolerance, double minimumSpeed, double target){
  
  double error = target;
  double previousError = error;
  double total = 0;
  double derivative = 0;

  Inertial.resetRotation();
  
  while (fabs(error) > tolerance){
    error = target - Inertial.rotation(degrees);
    derivative = (error - previousError) * 50;
    total = error * kp - derivative * kd;

    if (fabs(total) < minimumSpeed){
      LeftFront.spin(forward, total * minimumSpeed, percent);
      LeftBack.spin(forward, total * minimumSpeed, percent);
      LeftStack.spin(forward, total * minimumSpeed, percent);
      RightFront.spin(reverse, total * minimumSpeed, percent);
      RightBack.spin(reverse, total * minimumSpeed, percent);
      RightStack.spin(reverse, total * minimumSpeed, percent);
    }
    else {
      LeftFront.spin(forward, error * kp, percent);
      LeftBack.spin(forward, error * kp, percent);
      LeftStack.spin(forward, error * kp, percent);
      RightFront.spin(reverse, error * kp, percent);
      RightBack.spin(reverse, error * kp, percent);
      RightStack.spin(reverse, error * kp, percent);
    }

    previousError = error;
    wait(20, msec);
  }
  LeftFront.stop(brake);
  LeftBack.stop(brake);
  LeftStack.stop(brake);
  RightFront.stop(brake);
  RightBack.stop(brake);
  RightStack.stop(brake);

  wait(100, msec);
}

static void turnCounterClockwise(double kp, double kd, double tolerance, double minimumSpeed, double target){
  
  double error = target;
  double previousError;
  double total = 0;
  double derivative;

  Inertial.resetRotation();
  
  while (fabs(error) > tolerance){
    error = target + Inertial.rotation(degrees);
    derivative = (error - previousError) * 50;
    total = error * kp - derivative * kd;

    if (fabs(total) < minimumSpeed){
      LeftFront.spin(reverse, total * minimumSpeed, percent);
      LeftBack.spin(reverse, total * minimumSpeed, percent);
      LeftStack.spin(reverse, total * minimumSpeed, percent);
      RightFront.spin(forward, total * minimumSpeed, percent);
      RightBack.spin(forward, total * minimumSpeed, percent);
      RightStack.spin(forward, total * minimumSpeed, percent);
    }
    else {
      LeftFront.spin(reverse, error * kp, percent);
      LeftBack.spin(reverse, error * kp, percent);
      LeftStack.spin(reverse, error * kp, percent);
      RightFront.spin(forward, error * kp, percent);
      RightBack.spin(forward, error * kp, percent);
      RightStack.spin(forward, error * kp, percent);
    }

    previousError = error;
    wait(20, msec);
  }
  LeftFront.stop(brake);
  LeftBack.stop(brake);
  LeftStack.stop(brake);
  RightFront.stop(brake);
  RightBack.stop(brake);
  RightStack.stop(brake);

  wait(100, msec);
}

// kp, tolerance, minimumSpeed, target

static void defaultDrive(double target){
  driveForward(0.7, 0.5, 65, target);
}

static void defaultTurn(std::string direction, double target){
  if (direction == "Clockwise"){
    turnClockwise(1, 0.3, 1, 5, target);
  }
  if (direction == "CounterClockwise"){
    turnCounterClockwise(1, 0.3, 1, 15, target);  
  }
}

static void backOut(){
  driveReverse(0.7, 0.5, 60, 10);
}

static void scoreTriball(){
  driveForward(0.7, 0.5, 75, 10);
}

static void intake(){
  Intake.spin(forward, 75, percent);
}

static void outake(double waitTime){
  Intake.spin(reverse, 75, percent);
  wait(waitTime, sec);
}

static void stopIntake(){
  wait(50, msec);
  Intake.stop();
}

static Auton currentAuton = AutonNone;

static void autonSelector(){
  int runningSelector = true;
  while (runningSelector){
    if (currentAuton == AutonLeftSafe){
      Controller1.Screen.print("Auton Selected, Safe Left");
    }
    if (currentAuton == AutonLeftRisky){
      Controller1.Screen.print("Auton Selected, Risky Left");
    }
    if (currentAuton == AutonRightSafe){
      Controller1.Screen.print("Auton Selected, Safe Right");
    }
    if (currentAuton == AutonRightRisky){
      Controller1.Screen.print("Auton Selected, Risky Right");
    }
    if (Controller1.ButtonLeft.pressing()){
      if (currentAuton == AutonLeftSafe){
        currentAuton = AutonRightRisky;
      }
      else{
        currentAuton = static_cast<Auton> (static_cast<int> (currentAuton) - 1);
      }
      Controller1.Screen.clearScreen();
    }
    if (Controller1.ButtonRight.pressing()){
      if (currentAuton == AutonRightRisky){
        currentAuton = AutonLeftSafe;
      }
      else{
        currentAuton = static_cast<Auton> (static_cast<int> (currentAuton) + 1);
      }
      Controller1.Screen.clearScreen();
    }
    if (Controller1.ButtonA.pressing()){
      runningSelector = false;
    }
  }
  Controller1.rumble(rumblePulse);
}

/********** Autons **********/

void runAutonLeftSafe(){

}

void runAutonLeftRisky(){
  
}

void runAutonRightSafe(){ 
  //Score Match Load
  defaultDrive(52);
  defaultTurn("Clockwise", 90);
  outake(0.5);
  scoreTriball();
  backOut();
  defaultTurn("CounterClockwise", 140); //Face Triball 
  intake();
  defaultDrive(26); //Triball Picked Up
  defaultTurn("CounterClockwise", 145);
  defaultDrive(20);
  scoreTriball();
}

void runAutonRightRisky(){
  double currentTime = Brain.Timer.time();

  //Score Match Load
  defaultDrive(42);
  defaultTurn("Clockwise", 90);
  outake(0.25);
  scoreTriball(); //Match Load Scored

  //Middle Triball
  backOut();
  defaultTurn("CounterClockwise", 140); //Face Triball
  intake();
  defaultDrive(12); //Triball Picked Up
  stopIntake();
  defaultTurn("Clockwise", 140);
  outake(0.75);

  //Back Triball
  defaultTurn("Clockwise", 180); //Face Triball
  intake();
  defaultDrive(14); //Triball Picked Up
  stopIntake();
  defaultTurn("CounterClockwise", 180);
  defaultDrive(16);
  outake(0.25);
  defaultTurn("Clockwise", 180);
  Wings.set(true);
  driveReverse(0.7, 0.5, 75, 10); //Middle and Back Triball Scored
  Wings.set(false);

  //Side Triball
  defaultDrive(15);
  defaultTurn("CounterClockwise", 50);
  intake();
  defaultDrive(20); //Triball Picked Up
  stopIntake();
  defaultTurn("Clockwise", 200);
  defaultDrive(12);
  scoreTriball(); //Side Triball Scored

  std::cout << (Brain.Timer.time() - currentTime) / 1000 << " seconds" << std::endl;
}

/********** Pre Auton **********/

void calibrate(double seconds){
  Inertial.calibrate();
  wait(seconds, sec);
}

void preAuton(){
  calibrate(5);
  autonSelector();
}

/********** Auton Function **********/

void autonomous(){
  switch (currentAuton){
    case AutonNone: {
      break;
    }
    case AutonLeftSafe: {
      runAutonLeftSafe();
      break;
    }
    case AutonLeftRisky: {
      runAutonLeftRisky();
      break;
    }
    case AutonRightSafe: {
      runAutonRightSafe();
      break;
    }
    case AutonRightRisky: {
      runAutonRightRisky();
      break;
    }
    default: {
      break;
    }
  }
}

void testAuton(Auton testedAuton){
  switch (testedAuton){
    case AutonNone: {
      break;
    }
    case AutonLeftSafe: {
      runAutonLeftSafe();
      break;
    }
    case AutonLeftRisky: {
      runAutonLeftRisky();
      break;
    }
    case AutonRightSafe: {
      runAutonRightSafe();
      break;
    }
    case AutonRightRisky: {
      runAutonRightRisky();
      break;
    }
    default: {
      break;
    }
  }
}