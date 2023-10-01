#include "vex.h"
#include "Autonomous.h"
#include <iostream>

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

    previousLeftError = leftDriveError;
    previousRightError = rightDriveError; 
    std::cout << leftDriveError << std::endl; 
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

    previousLeftError = leftDriveError;
    previousRightError = rightDriveError;
    std::cout << leftDriveError << std::endl; 
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

    previousError = error;
    std::cout << error << std::endl;
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

    previousError = error;
    std::cout << error << std::endl;
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
    driveForward(2.5, 0.1, 0.2, 0.5, 80, target);
  }
  if (direction == "Reverse"){
    driveReverse(2.5, 0.1, 0.2, 0.5, 80, target);
  }
}

static void defaultTurn(std::string direction, double target){
  if (direction == "Clockwise"){
    turnClockwise(1, 0.04, 0.1, 2, 30, target);
  }
  if (direction == "CounterClockwise"){
    turnCounterClockwise(1, 0.04, 0.1, 2, 30, target);  
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
    turnClockwise(0.7, 0.02, 0.03, 2, 5, target);
  }
  if (direction == "CounterClockwise"){
    turnCounterClockwise(0.7, 0.02, 0.03, 2, 5, target);  
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
  driveForward(2.5, 0.1, 0.2, 0.5, 85, 10);
}

static void backOut(){
  driveReverse(2.5, 0.1, 0.2, 0.5, 70, 10);
}

/********** Autons **********/

void runAutonLeftAWP(){
  //Score Pre Load
  slowDrive("Forward", 20);
  slowTurn("Clockwise", 45);
  outake(0.5);
  defaultDrive("Forward", 10); //Pre Load Scored
  stopIntake();

  //Pull Out Match Load
  slowDrive("Reverse", 14);
  slowTurn("CounterClockwise", 45);
  slowDrive("Reverse", 14);
  slowTurn("Clockwise", 90);
  slowDrive("Reverse", 6);
  Arm.set(true);
  wait(0.3, sec);
  defaultTurn("CounterClockwise", 90); //Match Load Pulled Out
  Arm.set(false);
  slowDrive("Reverse", 8);
  defaultTurn("CounterClockwise", 30);

  //Touch Bar
  defaultDrive("Reverse", 36);
}

void runAutonLeftSabotage(){

}

void runAutonRightSafe(){ 
  //Score Pre Load
  slowDrive("Forward", 48);
  slowTurn("Clockwise", 90); //Counter Clockwise Movement from Dropping Intake
  outake(0.5);
  scoreTriball(); //Pre Load Scored
  backOut();

  //Side Triball
  slowTurn("Clockwise", 150); //Face Triball
  intake();
  slowDrive("Forward", 28); //Triball Picked Up
  stopIntake();
  slowDrive("Reverse", 28);
  slowTurn("CounterClockwise", 150);
  outake(0.5);
  scoreTriball();
  stopIntake();
}

void runAutonRightFourTB(){
  double currentTime = Brain.Timer.time();

  //Score Pre Load
  defaultDrive("Forward", 40);
  defaultTurn("Clockwise", 80); //Counter Clockise Movement from Dropping Intake
  outake(0.25);
  scoreTriball(); //Pre Load Scored

  //Middle Triball
  backOut();
  defaultTurn("CounterClockwise", 130); //Face Triball
  intake();
  defaultDrive("Forward", 12); //Triball Picked Up
  stopIntake();
  defaultTurn("Clockwise", 130);
  outake(0.75);

  //Back Triball
  defaultTurn("Clockwise", 180); //Face Triball
  intake();
  defaultDrive("Forward", 16); //Triball Picked Up
  stopIntake();
  defaultTurn("CounterClockwise", 180);
  defaultDrive("Forward", 16);
  outake(0.25);
  defaultTurn("Clockwise", 180);
  Wings.set(true);
  defaultDrive("Reverse", 15); //Middle and Back Triball Scored
  Wings.set(false);

  //Side Triball
  defaultDrive("Forward", 20);
  defaultTurn("CounterClockwise", 30);
  intake();
  defaultDrive("Forward", 26); //Triball Picked Up
  stopIntake();
  defaultDrive("Reverse", 20);
  slowTurn("CounterClockwise", 140);
  defaultDrive("Forward", 10);
  outake(0.25);
  scoreTriball(); //Side Triball Scored
  stopIntake();
  
  std::cout << (Brain.Timer.time() - currentTime) / 1000 << " seconds" << std::endl;
}

/********** Pre Auton **********/

static Auton currentAuton = AutonNone;

static void autonSelector(){
  bool runningSelector = true;
  while (runningSelector){
    if (currentAuton == AutonNone){
      Controller1.Screen.setCursor(2, 3);
      Controller1.Screen.print("No Auton Selected");
    }
    if (currentAuton == AutonLeftAWP){
      Controller1.Screen.setCursor(1, 5);
      Controller1.Screen.print("Auton Selected:");
      Controller1.Screen.setCursor(3, 5);
      Controller1.Screen.print("Left-Side AWP");
    }
    if (currentAuton == AutonLeftSabotage){
      Controller1.Screen.setCursor(1, 5);
      Controller1.Screen.print("Auton Selected:");
      Controller1.Screen.setCursor(3, 3);
      Controller1.Screen.print("Left-Side Sabotage");
    }
    if (currentAuton == AutonRightSafe){
      Controller1.Screen.setCursor(1, 5);
      Controller1.Screen.print("Auton Selected:");
      Controller1.Screen.setCursor(3, 5);
      Controller1.Screen.print("Right-Side Safe");
    }
    if (currentAuton == AutonRightFourTB){
      Controller1.Screen.setCursor(1, 5);
      Controller1.Screen.print("Auton Selected");
      Controller1.Screen.setCursor(3, 2);
      Controller1.Screen.print("Right-Side Four Triball");
    }
    if (Controller1.ButtonLeft.pressing()){
      Controller1.Screen.clearScreen();
      if (currentAuton == AutonNone || currentAuton == AutonLeftAWP){
        currentAuton = AutonRightFourTB;
      }
      else{
        currentAuton = static_cast<Auton> (static_cast<int> (currentAuton) - 1);
      }
    }
    if (Controller1.ButtonRight.pressing()){
      Controller1.Screen.clearScreen();
      if (currentAuton == AutonRightFourTB){
        currentAuton = AutonLeftAWP;
      }
      else{
        currentAuton = static_cast<Auton> (static_cast<int> (currentAuton) + 1);
      }
    }
    if (Controller1.ButtonDown.pressing()){
      Controller1.Screen.clearScreen();
      runningSelector = false;
    }

    wait(0.2, sec);
  }
  Controller1.rumble(rumblePulse);
}

void calibrate(double seconds){
  Inertial.calibrate();
  Controller1.Screen.setCursor(2, 6);
  Controller1.Screen.print("CALIBRATING!!!");
  wait(seconds, sec);
  Controller1.Screen.clearScreen();
}

void tempCheck(double warningTemp){
  double leftDriveTemp = std::max(std::max(LeftFront.temperature(celsius), LeftBack.temperature(celsius)), LeftStack.temperature(celsius));
  double rightDriveTemp = std::max(std::max(RightFront.temperature(celsius), RightBack.temperature(celsius)), RightStack.temperature(celsius));
  double cataTemp = Catapult.temperature(celsius);
  double intakeTemp = Intake.temperature(celsius); 

  if (leftDriveTemp > warningTemp){
    Controller1.Screen.setCursor(7, 2);
    Controller1.Screen.print("LEFT DRIVE HOT!!!");
    wait(2, sec);
  }
  if (rightDriveTemp > warningTemp){
    Controller1.Screen.setCursor(7, 2);
    Controller1.Screen.print("RIGHT DRIVE HOT!!!");
    wait(2, sec);

  }
  if (cataTemp > warningTemp){
    Controller1.Screen.setCursor(7, 2);
    Controller1.Screen.print("CATAPULT HOT!!!");
    wait(2, sec);
  }
  if (intakeTemp > warningTemp){
    Controller1.Screen.setCursor(7, 2);
    Controller1.Screen.print("INTAKE HOT!!!");
    wait(2, sec);
  }
}

void preAuton(){
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  Controller1.Screen.clearScreen();
  calibrate(5);
  tempCheck(55);
  autonSelector();
}

/********** Auton Function **********/

void autonomous(){
  switch (currentAuton){
    case AutonNone: {
      break;
    }
    case AutonLeftAWP: {
      runAutonLeftAWP();
      break;
    }
    case AutonLeftSabotage: {
      runAutonLeftSabotage();
      break;
    }
    case AutonRightSafe: {
      runAutonRightSafe();
      break;
    }
    case AutonRightFourTB: {
      runAutonRightFourTB();
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
    case AutonLeftAWP: {
      runAutonLeftAWP();
      break;
    }
    case AutonLeftSabotage: {
      runAutonLeftSabotage();
      break;
    }
    case AutonRightSafe: {
      runAutonRightSafe();
      break;
    }
    case AutonRightFourTB: {
      runAutonRightFourTB();
      break;
    }
    default: {
      break;
    }
  }
}