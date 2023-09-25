#include "vex.h"
#include <iostream>
#include <string>
#include "Autonomous.h"

/********** Functions for Auton **********/

static void driveForward(double kp, double ki, double kd, double tolerance, double minimumSpeed, double target){
  
  double leftDriveError = target;
  double leftIntegral = 0;
  double leftDerivative = 0;
  double previousLeftError = leftDriveError;
  double leftDriveTotal = 0;
  
  double rightDriveError = target;
  double rightIntegral = 0;
  double previousRightError = rightDriveError;
  double rightDerivative = 0;
  double rightDriveTotal = 0;

  LeftFront.resetPosition();
  RightFront.resetPosition();
  
  while ((fabs(leftDriveError) + fabs(leftDriveError)) / 2 > tolerance){
    //Left Side
    leftDriveError = target - LeftFront.position(turns) * 3.25 * M_PI / 2;
    leftDerivative = (previousLeftError - leftDriveError) * 50;
    leftDriveTotal = leftDriveError * kp + leftIntegral * ki - leftDerivative * kd;

    if (fabs(leftDriveTotal) < minimumSpeed){
      LeftFront.spin(forward, minimumSpeed, percent);
      LeftBack.spin(forward, minimumSpeed, percent);
      LeftStack.spin(forward, minimumSpeed, percent);
    }
    else {
      LeftFront.spin(forward, leftDriveTotal, percent);
      LeftBack.spin(forward, leftDriveTotal, percent);
      LeftStack.spin(forward, leftDriveTotal, percent);
    }
    if(leftDriveError < 10){
      leftIntegral += leftDriveError / 50;
    }
    
    //Right Side
    rightDriveError = target - RightFront.position(turns) * 3.25 * M_PI / 2;
    rightDerivative = (previousRightError - rightDriveError) * 50;
    rightDriveTotal = rightDriveError * kp + rightIntegral * ki - rightDerivative * kd;

    if (fabs(leftDriveTotal) < minimumSpeed){
      RightFront.spin(forward, minimumSpeed, percent);
      RightBack.spin(forward, minimumSpeed, percent);
      RightStack.spin(forward, minimumSpeed, percent);
    }
    else {
      RightFront.spin(forward, rightDriveTotal, percent);
      RightBack.spin(forward, rightDriveTotal, percent);
      RightStack.spin(forward, rightDriveTotal, percent);
    }
    if(rightDriveError < 10){
      rightIntegral += rightDriveError / 50;
    }

    //std::cout <<  leftDriveError << std::endl;
    previousLeftError = leftDriveError;
    previousRightError = rightDriveError;
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

static void driveReverse(double kp, double ki, double kd, double tolerance, double minimumSpeed, double target){
  
  double leftDriveError = target;
  double leftIntegral = 0;
  double leftDerivative = 0;
  double previousLeftError = leftDriveError;
  double leftDriveTotal = 0;
  
  double rightDriveError = target;
  double rightIntegral = 0;
  double previousRightError = rightDriveError;
  double rightDerivative = 0;
  double rightDriveTotal = 0;

  LeftFront.resetPosition();
  RightFront.resetPosition();
  
  while ((fabs(leftDriveError) + fabs(leftDriveError)) / 2 > tolerance){
    //Left Side
    leftDriveError = target + LeftFront.position(turns) * 3.25 * M_PI / 2;
    leftDerivative = (previousLeftError - leftDriveError) * 50;
    leftDriveTotal = leftDriveError * kp + leftIntegral * ki - leftDerivative * kd;

    if (fabs(leftDriveTotal) < minimumSpeed){
      LeftFront.spin(reverse, minimumSpeed, percent);
      LeftBack.spin(reverse, minimumSpeed, percent);
      LeftStack.spin(reverse, minimumSpeed, percent);
    }
    else {
      LeftFront.spin(reverse, leftDriveTotal, percent);
      LeftBack.spin(reverse, leftDriveTotal, percent);
      LeftStack.spin(reverse, leftDriveTotal, percent);
    }
    if(leftDriveError < 10){
      leftIntegral += leftDriveError / 50;
    }

    //Right Side
    rightDriveError = target + RightFront.position(turns) * 3.25 * M_PI / 2;
    rightDerivative = (previousRightError - rightDriveError) * 50;
    rightDriveTotal = rightDriveError * kp + rightIntegral * ki - rightDerivative * kd;

    if (fabs(leftDriveTotal) < minimumSpeed){
      RightFront.spin(reverse, minimumSpeed, percent);
      RightBack.spin(reverse, minimumSpeed, percent);
      RightStack.spin(reverse, minimumSpeed, percent);
    }
    else {
      RightFront.spin(reverse, rightDriveTotal, percent);
      RightBack.spin(reverse, rightDriveTotal, percent);
      RightStack.spin(reverse, rightDriveTotal, percent);
    }
    if(rightDriveError < 10){
      rightIntegral += rightDriveError / 50;
    }

    std::cout <<  rightDriveError << std::endl;
    previousLeftError = leftDriveError;
    previousRightError = rightDriveError;
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

static void turnClockwise(double kp, double ki, double kd, double tolerance, double minimumSpeed, double target){
  
  double error = target;
  double integral = 0;
  double derivative = 0;
  double previousError = error;
  double total = 0;

  Inertial.resetRotation();
  
  while (fabs(error) > tolerance){
    error = target - Inertial.rotation(degrees);
    derivative = (previousError - error) * 50;
    total = error * kp + integral * ki - derivative * kd;

    if (fabs(total) < minimumSpeed){
      LeftFront.spin(forward, minimumSpeed, percent);
      LeftBack.spin(forward, minimumSpeed, percent);
      LeftStack.spin(forward, minimumSpeed, percent);
      RightFront.spin(reverse, minimumSpeed, percent);
      RightBack.spin(reverse, minimumSpeed, percent);
      RightStack.spin(reverse, minimumSpeed, percent);
    }
    else {
      LeftFront.spin(forward, total, percent);
      LeftBack.spin(forward, total, percent);
      LeftStack.spin(forward, total, percent);
      RightFront.spin(reverse, total, percent);
      RightBack.spin(reverse, total, percent);
      RightStack.spin(reverse, total, percent);
    }

    if (error < 30){
      integral += error / 50;
    }

    std::cout << error << std::endl;
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

static void turnCounterClockwise(double kp, double ki, double kd, double tolerance, double minimumSpeed, double target){
  
  double error = target;
  double derivative;
  double previousError = error;
  double integral = 0;
  double total = 0;

  Inertial.resetRotation();
  
  while (fabs(error) > tolerance){
    error = target + Inertial.rotation(degrees);
    derivative = (previousError - error) * 50;
    total = error * kp - derivative * kd;

    if (fabs(total) < minimumSpeed){
      LeftFront.spin(reverse, minimumSpeed, percent);
      LeftBack.spin(reverse, minimumSpeed, percent);
      LeftStack.spin(reverse, minimumSpeed, percent);
      RightFront.spin(forward, minimumSpeed, percent);
      RightBack.spin(forward, minimumSpeed, percent);
      RightStack.spin(forward, minimumSpeed, percent);
    }
    else {
      LeftFront.spin(reverse, total, percent);
      LeftBack.spin(reverse, total, percent);
      LeftStack.spin(reverse, total, percent);
      RightFront.spin(forward, total, percent);
      RightBack.spin(forward, total, percent);
      RightStack.spin(forward, total, percent);
    }

    if (error < 30){
      integral += error / 50;
    }

    std::cout << error << std::endl;
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

static void defaultDrive(std::string direction, double target){
  if (direction == "Forward"){
    driveForward(2.5, 0.1, 0.2, 0.5, 40, target);
  }
  if (direction == "Reverse"){
    driveReverse(2.5, 0.1, 0.2, 0.5, 40, target);
  }
  
}

static void defaultTurn(std::string direction, double target){
  if (direction == "Clockwise"){
    turnClockwise(1, 0.04, 0.07, 2, 30, target);
  }
  if (direction == "CounterClockwise"){
    turnCounterClockwise(1, 0.04, 0.07, 2, 30, target);  
  } 
}


static void slowDrive(std::string direction, double target){
  if (direction == "Forward"){
    driveForward(1.5, 0.05, 0.1, 0.5, 25, target);
  }
  if (direction == "Reverse"){
    driveReverse(1.5, 0.05, 0.1, 0.5, 25, target);
  }
  
}

static void slowTurn(std::string direction, double target){
  if (direction == "Clockwise"){
    turnClockwise(0.7, 0.02, 0.03, 2, 15, target);
  }
  if (direction == "CounterClockwise"){
    turnCounterClockwise(0.7, 0.02, 0.03, 2, 15, target);  
  } 
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

static void scoreTriball(){
  driveForward(0.7, 0, 0, 0.5, 75, 10);
}

static void backOut(){
  driveReverse(0.7, 0.05, 0.1, 0.5, 60, 10);
}

/********** Autons **********/

void runAutonLeftSafe(){
  //Score Pre Load
  slowDrive("Forward", 24);
  slowTurn("Clockwise", 45);
  outake(0.5);
  scoreTriball(); //Pre Load Scored
  stopIntake();

  //Pull Out Match Load
  slowDrive("Reverse", 16);
  slowTurn("CounterClockwise", 45);
  slowDrive("Reverse", 17);
  slowTurn("Clockwise", 90);
  slowDrive("Reverse", 2);
  Arm.set(true);
  defaultTurn("CounterClockwise", 125); //Match Load Pulled Out
  Arm.set(false);

  //Touch Bar
  defaultDrive("Reverse", 43);
}

void runAutonLeftRisky(){
  
}

void runAutonRightSafe(){ 
  //Score Pre Load
  slowDrive("Forward", 38);
  slowTurn("Clockwise", 80); //Counter Clockwise Movement from Dropping Intake
  outake(0.5);
  scoreTriball(); //Pre Load Scored
  backOut();

  //Side Triball
  slowTurn("CounterClockwise", 140); //Face Triball 
  intake();
  slowDrive("Forward", 26); //Triball Picked Up
  slowTurn("CounterClockwise", 145);
  slowDrive("Forward", 20);
  scoreTriball();
}

void runAutonRightRisky(){
  double currentTime = Brain.Timer.time();

  //Score Pre Load
  defaultDrive("Forward", 38);
  defaultTurn("Clockwise", 80); //Counter Clockwise Movement from Dropping Intake
  outake(0.25);
  scoreTriball(); //Pre Load Scored

  //Middle Triball
  backOut();
  defaultTurn("CounterClockwise", 120); //Face Triball
  intake();
  defaultDrive("Forward", 13); //Triball Picked Up
  stopIntake();
  defaultTurn("Clockwise", 120);
  outake(0.75);

  //Back Triball
  defaultTurn("Clockwise", 180); //Face Triball
  intake();
  defaultDrive("Forward", 14); //Triball Picked Up
  stopIntake();
  defaultTurn("CounterClockwise", 180);
  defaultDrive("Forward", 12);
  outake(0.25);
  defaultTurn("Clockwise", 180);
  Wings.set(true);
  defaultDrive("Reverse", 10); //Middle and Back Triball Scored
  Wings.set(false);

  //Side Triball
  defaultDrive("Forward", 15);
  defaultTurn("CounterClockwise", 50);
  intake();
  defaultDrive("Forward", 22); //Triball Picked Up
  stopIntake();
  defaultDrive("Reverse", 22);
  defaultTurn("CounterClockwise", 130);
  defaultDrive("Forward", 10);
  outake(0.25);
  scoreTriball(); //Side Triball Scored

  std::cout << (Brain.Timer.time() - currentTime) / 1000 << " seconds" << std::endl;
}

/********** Pre Auton **********/

static Auton currentAuton = AutonNone;

static void autonSelector(){
  bool runningSelector = true;
  while (runningSelector){
    if (currentAuton == AutonNone){
      Controller1.Screen.print("No Auton Selected");
    }
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

void calibrate(double seconds){
  Inertial.calibrate();
  wait(seconds, sec);
}

void tempCheck(double warningTemp){
  double leftDriveTemp = std::max(std::max(LeftFront.temperature(fahrenheit), LeftBack.temperature(fahrenheit)), LeftStack.temperature(fahrenheit));
  double rightDriveTemp = std::max(std::max(RightFront.temperature(fahrenheit), RightBack.temperature(fahrenheit)), RightStack.temperature(fahrenheit));
  double cataTemp = Catapult.temperature(fahrenheit);
  double intakeTemp = Intake.temperature(fahrenheit); 

  if (leftDriveTemp > warningTemp){
    Controller1.Screen.setCursor(7, 2);
    Controller1.Screen.print("LEFT DRIVE HOT!!!");
  }
  if (rightDriveTemp > warningTemp){
    Controller1.Screen.setCursor(7, 3);
    Controller1.Screen.print("RIGHT DRIVE HOT!!!");
  }
  if (cataTemp > warningTemp){
    Controller1.Screen.setCursor(7, 4);
    Controller1.Screen.print("CATAPULT HOT!!!");
  }
  if (intakeTemp > warningTemp){
    Controller1.Screen.setCursor(7, 5);
    Controller1.Screen.print("INTAKE HOT!!!");
  }
}

void preAuton(){
  Controller1.Screen.clearScreen();
  calibrate(5);
  tempCheck(130);
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