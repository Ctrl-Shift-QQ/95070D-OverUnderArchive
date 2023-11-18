#include "vex.h"
#include "Autonomous.h"
#include <iostream>

/********** Functions for Auton **********/

static int getSign(double input){
  if (input > 0){
    return 1;
  }
  else if (input < 0){
    return -1;
  }
  else{
    return 0;
  }
}

static void driveWithPID(double kp, double ki, double kd, double tolerance, double minimumSpeed, double target){
  
  double leftDriveError = target;
  double leftIntegral = 0;
  double leftDerivative = 0;
  double previousLeftError = leftDriveError;
  double leftDriveTotal = 0;
  
  double rightDriveError = target;
  double rightIntegral = 0;
  double rightDerivative = 0;
  double previousRightError = rightDriveError;
  double rightDriveTotal = 0;

  LeftBack.resetPosition();
  RightBack.resetPosition();
  
  while ((fabs(leftDriveError) + fabs(rightDriveError)) / 2 > tolerance){
    //Left Side
    leftDriveError = target - LeftBack.position(turns) * 3.25 * M_PI / (5/3);
    leftDerivative = (previousLeftError - leftDriveError) * 50;
    leftDriveTotal = leftDriveError * kp + leftIntegral * ki - leftDerivative * kd;

    if (fabs(leftDriveTotal) < minimumSpeed){
      LeftFront.spin(forward, getSign(leftDriveError) * minimumSpeed, percent);
      LeftMiddle.spin(forward, getSign(leftDriveError) * minimumSpeed, percent);
      LeftBack.spin(forward, getSign(leftDriveError) * minimumSpeed, percent);
    }
    else {
      LeftFront.spin(forward, leftDriveTotal, percent);
      LeftMiddle.spin(forward, leftDriveTotal, percent);
      LeftBack.spin(forward, leftDriveTotal, percent);
    }

    if(fabs(leftDriveError) < 10){
      leftIntegral += leftDriveError / 50;
    }
    
    //Right Side
    rightDriveError = target - RightBack.position(turns) * 3.25 * M_PI / (5/3);
    rightDerivative = (previousRightError - rightDriveError) * 50;
    rightDriveTotal = rightDriveError * kp + rightIntegral * ki - rightDerivative * kd;

    if (fabs(rightDriveTotal) < minimumSpeed){
      RightFront.spin(forward, getSign(rightDriveError) * minimumSpeed, percent);
      RightMiddle.spin(forward, getSign(rightDriveError) * minimumSpeed, percent);
      RightBack.spin(forward, getSign(rightDriveError) * minimumSpeed, percent);
    }
    else {
      RightFront.spin(forward, rightDriveTotal, percent);
      RightMiddle.spin(forward, rightDriveTotal, percent);
      RightBack.spin(forward, rightDriveTotal, percent);
    }

    if(fabs(rightDriveError) < 10){
      rightIntegral += rightDriveError / 50;
    }

    previousLeftError = leftDriveError;
    previousRightError = rightDriveError;    

    wait(20, msec);
  }

  LeftFront.stop(brake);
  LeftMiddle.stop(brake);
  LeftBack.stop(brake);
  RightFront.stop(brake);
  RightMiddle.stop(brake);
  RightBack.stop(brake);

  wait(100, msec);
}

static double shorterTurningPathError(double target){
  
  static int throughZeroDirection;
  double smallerDegree = std::min(target, Inertial.heading());
  double largerDegree = std::max(target, Inertial.heading());

  if(smallerDegree == target){
    throughZeroDirection = 1;
  }
  else{
    throughZeroDirection = -1; 
  }

  if(largerDegree - smallerDegree > 180){
    return throughZeroDirection * (360 - largerDegree + smallerDegree);
  }
  else{
    return target - Inertial.heading();
  }
}

static void turnWithPIDTo(double kp, double ki, double kd, double tolerance, double minimumSpeed, double target){
  
  double error = shorterTurningPathError(target);
  double integral = 0;
  double derivative = 0;
  double previousError = error;
  double total = 0;
  
  while (fabs(error) > tolerance){
    
    error = shorterTurningPathError(target);
    derivative = (previousError - error) * 50;
    total = error * kp + integral * ki - derivative * kd;

    if (fabs(error) < minimumSpeed){
      LeftFront.spin(forward, getSign(error) * minimumSpeed, percent);
      LeftMiddle.spin(forward, getSign(error) * minimumSpeed, percent);
      LeftBack.spin(forward, getSign(error) * minimumSpeed, percent);
      RightFront.spin(reverse, getSign(error) * minimumSpeed, percent);
      RightMiddle.spin(reverse, getSign(error) * minimumSpeed, percent);
      RightBack.spin(reverse, getSign(error) * minimumSpeed, percent);
    }
    else {
      LeftFront.spin(forward, total, percent);
      LeftMiddle.spin(forward, total, percent);
      LeftBack.spin(forward, total, percent);
      RightFront.spin(reverse, total, percent);
      RightMiddle.spin(reverse, total, percent);
      RightBack.spin(reverse, total, percent);
    }

    if (fabs(error) < 20){
      integral += error / 50;
    }

    previousError = error;

    wait(20, msec);
  }

  LeftFront.stop(brake);
  LeftMiddle.stop(brake);
  LeftBack.stop(brake);
  RightFront.stop(brake);
  RightMiddle.stop(brake);
  RightBack.stop(brake);

  wait(100, msec);
}

static void slowDrive(std::string direction, double target){ 
  if (direction == "Forward"){
    driveWithPID(0.5, 0.5, 0.3, 0.5, 35, target);
  }
  if (direction == "Reverse"){
    driveWithPID(0.5, 0.5, 0.3, 0.5, 35, -target);
  }
}

static void defaultDrive(std::string direction, double target){
  if (direction == "Forward"){
    driveWithPID(3, 0.3, 0.05, 0.5, 40, target);
  }
  if (direction == "Reverse"){
    driveWithPID(3, 0.3, 0.05, 0.5, 40, -target);
  }
}

static void turnTo(double target){
  turnWithPIDTo(0.7, 0.1, 0.04, 3, 12, target);
}

static void intake(){
  Intake.spin(forward, 90, percent);
}

static void outake(double waitTime){
  Intake.spin(reverse, 90, percent);
  wait(waitTime, sec);
  Intake.stop();
}

static void frontRam(double target){
  driveWithPID(2.5, 0.1, 0.2, 3, 70, target);
}

static void backRam(double target){
  driveWithPID(2.5, 0.1, 0.2, 3, 70, -target);
}

/********** Autons **********/

void runAutonLeftAWP(){
  IntakePiston.set(true);
  intake();
  slowDrive("Forward", 28);
  turnTo(45);
  slowDrive("Forward", 8);
  outake(0.5);
  slowDrive("Reverse", 6);
  turnTo(225);
  backRam(22); //Pre Load Scored

  slowDrive("Forward", 18);
  turnTo(0);
  slowDrive("Reverse", 10);
  Wings.set(true);
  slowDrive("Reverse", 28);
  turnTo(315); //Match Load Retrieved

  Blocker.set(true);
  slowDrive("Reverse", 56); //Elevation Bar Touched
  Wings.set(false);
}

void runAutonLeftNoAWP(){
  IntakePiston.set(true);
  intake();
  slowDrive("Forward", 28);
  turnTo(45);
  slowDrive("Forward", 8);
  outake(0.5);
  slowDrive("Reverse", 6);
  turnTo(225);
  backRam(22); //Pre Load Scored

  slowDrive("Forward", 18);
  turnTo(0);
  slowDrive("Reverse", 10);
  Wings.set(true);
  slowDrive("Reverse", 28);
  turnTo(315); //Match Load Retrieved

  slowDrive("Reverse", 56); //Below Bar Triball Pushed

  Wings.set(false);
  slowDrive("Forward", 60);
  turnTo(270);
  defaultDrive("Forward", 4); //Match Loading Position
}

void runAutonLeftSabotage(){
  IntakePiston.set(true);
  intake();
  defaultDrive("Forward", 62);
  turnTo(290);
  outake(0.5);
  turnTo(270);
  defaultDrive("Reverse", 16);
  intake();
  turnTo(0);
  defaultDrive("Forward", 6);
  turnTo(90);
  frontRam(29); 
  outake(1); //Middle Triball Popped Over

  backRam(52); //Pre Load Scored

  defaultDrive("Forward", 4);
  turnTo(0);
  defaultDrive("Reverse", 85);
  turnTo(270);
  defaultDrive("Reverse", 44); //Below Bar Triball Pushed

  defaultDrive("Forward", 52); 
  turnTo(225); 
  defaultDrive("Forward", 4); //Match Loading Position
}

void runAutonRightSafe(){
  double startTime = Brain.Timer.time();

  IntakePiston.set(true);
  intake();
  defaultDrive("Forward", 8);
  slowDrive("Reverse", 54);
  turnTo(315);
  slowDrive("Reverse", 20);
  Wings.set(true);
  turnTo(285); //Match Load Retreived
  
  backRam(40); //Pre Load and Match Load Scored 

  Wings.set(false);
  turnTo(270);
  defaultDrive("Forward", 8);
  turnTo(90);
  outake(0.3);
  frontRam(12); //Below Bar Triball Scored

  defaultDrive("Reverse", 18);
  turnTo(15);
  intake();
  defaultDrive("Forward", 80);
  turnTo(315);
  defaultDrive("Reverse", 24);
  turnTo(180);
  defaultDrive("Forward", 16);
  outake(0.3);
  frontRam(14); //Side Triball Scored

  defaultDrive("Reverse", 14); //Back Out

  std::cout << "Time: " << (Brain.Timer.time() - startTime) / 1000 << std::endl;  
}

void runAutonRightFiveTB(){
  double startTime = Brain.Timer.time();

  IntakePiston.set(true);
  intake();
  wait(250, msec);
  slowDrive("Reverse", 58);
  turnTo(315);
  slowDrive("Reverse", 18);
  Wings.set(true);
  turnTo(285); //Match Load Retreived
  
  backRam(32); //Pre Load and Match Load Scored 

  Wings.set(false);
  turnTo(270);
  defaultDrive("Forward", 8);
  turnTo(90);
  outake(0.3);
  frontRam(12); //Below Bar Triball Scored

  defaultDrive("Reverse", 8);
  turnTo(135);
  defaultDrive("Reverse", 20);
  turnTo(45);
  intake();
  frontRam(100);
  turnTo(0);
  Wings.set(true);
  backRam(45); //Middle Triball Scored

  Wings.set(false);
  defaultDrive("Forward", 8);
  turnTo(180);
  outake(0.3);
  frontRam(10); //Back Triball Scored

  defaultDrive("Reverse", 12); //Back Out

  std::cout << "Time: " << (Brain.Timer.time() - startTime) / 1000 << std::endl;  
}

/********** Pre Auton **********/

static Auton currentAuton = AutonNone;

static void autonSelector(){
  bool runningSelector = true;

  int columns[5] = {2, 3, 3, 5, 3};
  std::string autonNames[5] = {"Left-Side Safe AWP", "Left-Side NO AWP", "Left-Side Sabotage", "Right-Side Safe", "Right-Side Five Triball"};
  Auton autons[5] = {AutonLeftAWP, AutonLeftNoAWP, AutonLeftSabotage, AutonRightSafe, AutonRightFiveTB};

  bool buttonLeftPressed;
  bool buttonRightPressed;

  while (runningSelector){
    if (currentAuton == AutonNone){
      Controller1.Screen.setCursor(2, 3);
      Controller1.Screen.print("No Auton Selected");
    }
    else{
      Controller1.Screen.setCursor(1, 5);
      Controller1.Screen.print("Auton Selected:");
    }

    for (int i = 0; i < 5; i++){
      if (currentAuton == autons[i]){
        Controller1.Screen.setCursor(3, columns[i]);
        Controller1.Screen.print(autonNames[i].c_str());
      }
    }

    if (Controller1.ButtonLeft.pressing() && !buttonLeftPressed){
      Controller1.Screen.clearScreen();
      if (currentAuton == AutonNone || currentAuton == AutonLeftAWP){
        currentAuton = AutonRightFiveTB;
      }
      else{
        currentAuton = static_cast<Auton> (static_cast<int> (currentAuton) - 1);
      }

      buttonLeftPressed = true;
    }
    if (!Controller1.ButtonLeft.pressing() && buttonLeftPressed){
      buttonLeftPressed = false;
    }

    if (Controller1.ButtonRight.pressing() && !buttonRightPressed){
      Controller1.Screen.clearScreen();
      if (currentAuton == AutonRightFiveTB){
        currentAuton = AutonLeftAWP;
      }
      else{
        currentAuton = static_cast<Auton> (static_cast<int> (currentAuton) + 1);
      }

      buttonRightPressed = true;
    }
    if (!Controller1.ButtonRight.pressing() && buttonRightPressed){
      buttonRightPressed = false; 
    }

    if (Controller1.ButtonDown.pressing()){
      Controller1.Screen.clearScreen();
      runningSelector = false;
    }

    wait(50, msec);
  }
  Controller1.rumble("-.-.");
}

void calibrateInertial(){
  Controller1.Screen.setCursor(2, 6);
  Inertial.calibrate();
  Controller1.Screen.print("CALIBRATING!!!");
  while(Inertial.isCalibrating()){
    wait(100, msec);
  }
  Controller1.Screen.clearScreen();
  Inertial.resetHeading();
}

void tempCheck(double warningTemp){
  double leftDriveTemp = std::max(std::max(LeftFront.temperature(fahrenheit), LeftMiddle.temperature(fahrenheit)), LeftBack.temperature(fahrenheit));
  double rightDriveTemp = std::max(std::max(RightFront.temperature(fahrenheit), RightMiddle.temperature(fahrenheit)), RightBack.temperature(fahrenheit));
  double cataTemp = Catapult.temperature(fahrenheit);
  double intakeTemp = Intake.temperature(fahrenheit);

  double columns[4] = {3, 3, 4, 6};
  double temperatures[4] = {leftDriveTemp, rightDriveTemp, cataTemp, intakeTemp};
  std::string mechs[4] = {"LEFT DRIVE", "RIGHT DRIVE", "CATAPULT", "INTAKE"};

  for (int i = 0; i < 4; i++){
    if (temperatures[i] > warningTemp){
      Controller1.Screen.setCursor(1, columns[i]);
      Controller1.Screen.print(mechs[i].c_str());
      Controller1.Screen.print(" HOT!!!");
      Controller1.Screen.setCursor(3, 10);
      Controller1.Screen.print(temperatures[i]);
      
      wait(1.5, sec);
      Controller1.Screen.clearScreen();
    }
  }
}

void preAuton(){
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  Controller1.Screen.clearScreen();

  tempCheck(120);
  calibrateInertial();
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
    case AutonLeftNoAWP: {
      runAutonLeftNoAWP();
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
    case AutonRightFiveTB: {
      runAutonRightFiveTB();
      break;
    }
    default: {
      break;
    }
  }
}

void testAuton(Auton testedAuton){
  double startTime = Brain.Timer.time();

  switch (testedAuton){
    case AutonNone: {
      break;
    }
    case AutonLeftAWP: {
      runAutonLeftAWP();
      std::cout << "Time: " << (Brain.Timer.time() - startTime) / 1000 << std::endl;  
      break;
    }
    case AutonLeftNoAWP: {
      runAutonLeftNoAWP();
      std::cout << "Time: " << (Brain.Timer.time() - startTime) / 1000 << std::endl;  
      break;
    }
    case AutonLeftSabotage: {
      runAutonLeftSabotage();
      std::cout << "Time: " << (Brain.Timer.time() - startTime) / 1000 << std::endl;  
      break;
    }
    case AutonRightSafe: {
      runAutonRightSafe();
      std::cout << "Time: " << (Brain.Timer.time() - startTime) / 1000 << std::endl;  
      break;
    }
    case AutonRightFiveTB: {
      runAutonRightFiveTB();
      std::cout << "Time: " << (Brain.Timer.time() - startTime) / 1000 << std::endl;  
      break;
    }
    default: {
      break;
    }
  }
}