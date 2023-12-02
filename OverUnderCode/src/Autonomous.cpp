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

static void driveWithPID(double kp, double ki, double kd, double tolerance, double minimumSpeed, double maxI, double target){
  
  double leftDriveError = target;
  double leftIntegral = 0;
  double leftDerivative;
  double previousLeftError = leftDriveError;
  double leftDriveTotal;
  
  double rightDriveError = target;
  double rightIntegral = 0;
  double rightDerivative;
  double previousRightError = rightDriveError;
  double rightDriveTotal;

  LeftBack.resetPosition();
  RightBack.resetPosition();
  
  while ((fabs(leftDriveError) + fabs(rightDriveError)) / 2 > tolerance){
    //Left Side
    leftDriveError = target - LeftBack.position(turns) * 3.25 * M_PI / (4/3);
    leftDerivative = (previousLeftError - leftDriveError) * 100;
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

    if(fabs(leftDriveError) < maxI){
      leftIntegral += leftDriveError / 100;
    }
    
    //Right Side
    rightDriveError = target - RightBack.position(turns) * 3.25 * M_PI / (4/3);
    rightDerivative = (previousRightError - rightDriveError) * 100;
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

    if(fabs(rightDriveError) < maxI){
      rightIntegral += rightDriveError / 100;
    }

    previousLeftError = leftDriveError;
    previousRightError = rightDriveError;    

    wait(10, msec);
  }

  LeftFront.stop(brake);
  LeftMiddle.stop(brake);
  LeftBack.stop(brake);
  RightFront.stop(brake);
  RightMiddle.stop(brake);
  RightBack.stop(brake);

  wait(100, msec);
}

static double turnError(double target){
  
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

static void turnWithPID(double kp, double ki, double kd, double tolerance, double minimumSpeed, double maxI, double target){
  
  double error = turnError(target);
  double integral = 0;
  double derivative;
  double previousError = error;
  double total;
  
  while (fabs(error) > tolerance){
    error = turnError(target);
    derivative = (previousError - error) * 100;
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

    if (fabs(error) < maxI){
      integral += error / 100;
    }

    previousError = error;

    wait(10, msec);
  }

  LeftFront.stop(brake);
  LeftMiddle.stop(brake);
  LeftBack.stop(brake);
  RightFront.stop(brake);
  RightMiddle.stop(brake);
  RightBack.stop(brake);

  wait(100, msec);
}

static double swingError(Direction turn, double target){
  double smallerDegree = std::min(target, Inertial.heading());
  double largerDegree = std::max(target, Inertial.heading());

  if (turn == Clockwise){
    if (largerDegree == Inertial.heading()){
      return 360 - Inertial.heading() + target;
    }
    else{
      return target - Inertial.heading();
    }
  }
  else if (turn == CounterClockwise){
    if (smallerDegree == Inertial.heading()){
      return 360 - target + Inertial.heading();
    }
    else{
      return Inertial.heading() - target;
    }
  }
  else{
    return 0;
  }
}

static void swingWithPID(Direction drive, Direction turn, double kp, double ki, double kd, double tolerance, double minimumSpeed, double maxI, double percentage, double target){
  double error = target;
  double integral = 0;
  double derivative;
  double previousError = error;
  double total;
  directionType driveDirection;
  double leftDriveFactor;
  double rightDriveFactor;

  if (drive == Forward){
    driveDirection = forward;
  }
  else if (drive == Reverse){
    driveDirection = reverse;
  }

  if (turn == Clockwise){
    leftDriveFactor = 1;
    rightDriveFactor = percentage / 100;
  }
  else if (turn == CounterClockwise){
    leftDriveFactor = percentage / 100;
    rightDriveFactor = 1;
  }

  while (fabs(error) > tolerance){
    error = swingError(turn, target);
    derivative = (previousError - error) * 100;
    total = error * kp + integral * ki - derivative * kd;

    if (fabs(error) < minimumSpeed){
      LeftFront.spin(driveDirection, leftDriveFactor * getSign(error) * minimumSpeed, percent);
      LeftMiddle.spin(driveDirection, leftDriveFactor * getSign(error) * minimumSpeed, percent);
      LeftBack.spin(driveDirection, leftDriveFactor * getSign(error) * minimumSpeed, percent);
      RightFront.spin(driveDirection, rightDriveFactor * getSign(error) * minimumSpeed, percent);
      RightMiddle.spin(driveDirection, rightDriveFactor * getSign(error) * minimumSpeed, percent);
      RightBack.spin(driveDirection, rightDriveFactor * getSign(error) * minimumSpeed, percent);
    }
    else {
      LeftFront.spin(driveDirection, leftDriveFactor * total, percent);
      LeftMiddle.spin(driveDirection, leftDriveFactor * total, percent);
      LeftBack.spin(driveDirection, leftDriveFactor * total, percent);
      RightFront.spin(driveDirection, rightDriveFactor * total, percent);
      RightMiddle.spin(driveDirection, rightDriveFactor * total, percent);
      RightBack.spin(driveDirection, rightDriveFactor * total, percent);

      std::cout << error << "   " << integral << "   " << derivative << "   " << total << std::endl;
    }

    if (fabs(error) < maxI){
      integral += error / 100;
    }

    previousError = error;

    wait(10, msec);
  }

  LeftFront.stop(brake);
  LeftMiddle.stop(brake);
  LeftBack.stop(brake);
  RightFront.stop(brake);
  RightMiddle.stop(brake);
  RightBack.stop(brake);

  wait(100, msec);
}

static void slowDrive(Direction direction, double target){ 
  if (direction == Forward){
    //driveWithPID();
  }
  if (direction == Reverse){
    //driveWithPID();
  }
}

static void defaultDrive(Direction direction, double target){
  if (direction == Forward){
    //driveWithPID();
  }
  if (direction == Reverse){
    //driveWithPID();
  }
}

static void turnTo(double target){
  //turnWithPID();
}

static void swingTo(double target){
  //swingWithPID();
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
  // driveWithPID();
}

static void backRam(double target){
  // driveWithPID();
}

/********** Autons **********/

void runAutonLeftAWP(){
}

void runAutonLeftNoAWP(){
}

void runAutonLeftSabotage(){
}

void runAutonRightSafe(){
}

void runAutonRightSixTB(){ 
}

/********** Pre Auton **********/

static Auton currentAuton = AutonNone;

static void autonSelector(){
  bool runningSelector = true;

  int columns[5] = {2, 3, 3, 5, 2};
  std::string autonNames[5] = {"Left-Side Safe AWP", "Left-Side NO AWP", "Left-Side Sabotage", "Right-Side Safe", "Right-Side Six Triball"};
  Auton autons[5] = {AutonLeftAWP, AutonLeftNoAWP, AutonLeftSabotage, AutonRightSafe, AutonRightSixTB};

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
        currentAuton = AutonRightSixTB;
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
      if (currentAuton == AutonRightSixTB){
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
    case AutonRightSixTB: {
      runAutonRightSixTB();
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
    case AutonRightSixTB: {
      runAutonRightSixTB();
      std::cout << "Time: " << (Brain.Timer.time() - startTime) / 1000 << std::endl;  
      break;
    }
    default: {
      break;
    }
  }
}