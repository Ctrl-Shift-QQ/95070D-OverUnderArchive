#include "vex.h"
#include "Autonomous.h"
#include <iostream>

/********** Functions for Auton **********/

static void driveWithPID(double kp, double ki, double kd, double tolerance, double minimumSpeed, double target){
  
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

  LeftBack.resetPosition();
  RightBack.resetPosition();
  
  while ((fabs(leftDriveError) + fabs(leftDriveError)) / 2 > tolerance){
    //Left Side
    leftDriveError = target - LeftFront.position(turns) * 3.25 * M_PI / (5/3);
    leftDerivative = (previousLeftError - leftDriveError) * 50;
    leftDriveTotal = leftDriveError * kp + leftIntegral * ki - leftDerivative     * kd;

    if (fabs(leftDriveTotal) < minimumSpeed){
      LeftFront.spin(forward, minimumSpeed, percent);
      LeftMiddle.spin(forward, minimumSpeed, percent);
      LeftBack.spin(forward, minimumSpeed, percent);
    }
    else {
      LeftFront.spin(forward, leftDriveTotal, percent);
      LeftMiddle.spin(forward, leftDriveTotal, percent);
      LeftBack.spin(forward, leftDriveTotal, percent);
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
      RightMiddle.spin(forward, minimumSpeed, percent);
      RightBack.spin(forward, minimumSpeed, percent);
    }
    else {
      RightFront.spin(forward, rightDriveTotal, percent);
      RightMiddle.spin(forward, rightDriveTotal, percent);
      RightBack.spin(forward, rightDriveTotal, percent);
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
  LeftMiddle.stop(brake);
  LeftBack.stop(brake);
  RightFront.stop(brake);
  RightMiddle.stop(brake);
  RightBack.stop(brake);

  wait(100, msec);
}


static void turnWithPIDTo(double kp, double ki, double kd, double tolerance, double minimumSpeed, double target){
  
  double error = target;
  double integral = 0;
  double derivative = 0;
  double previousError = error;
  double total = 0;
  
  while (fabs(error) > tolerance){

    if(360 - std::max(target, Inertial.heading()) + std::min(target, Inertial.heading()) > 180){
      error = 360 - std::max(target, Inertial.heading()) + std::min(target, Inertial.heading());
    }
    else{
      error = target - Inertial.heading();
    }

    derivative = (previousError - error) * 50;
    total = error * kp + integral * ki - derivative * kd;

    if (fabs(total) < minimumSpeed){
      LeftFront.spin(forward, minimumSpeed, percent);
      LeftMiddle.spin(forward, minimumSpeed, percent);
      LeftBack.spin(forward, minimumSpeed, percent);
      RightFront.spin(reverse, minimumSpeed, percent);
      RightMiddle.spin(reverse, minimumSpeed, percent);
      RightBack.spin(reverse, minimumSpeed, percent);
    }
    else {
      LeftFront.spin(forward, total, percent);
      LeftMiddle.spin(forward, total, percent);
      LeftBack.spin(forward, total, percent);
      RightFront.spin(reverse, total, percent);
      RightMiddle.spin(reverse, total, percent);
      RightBack.spin(reverse, total, percent);
    }
    if (error < 30){
      integral += error / 50;
    }

    previousError = error;
    std::cout << error << std::endl;
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

static void turnCounterClockwiseTo(double kp, double ki, double kd, double tolerance, double minimumSpeed, double target){
  
  double error = target;
  double derivative;
  double previousError = error;
  double integral = 0;
  double total = 0;
  
  while (fabs(error) > tolerance){
    error = target + Inertial.rotation(degrees);
    derivative = (previousError - error) * 50;
    total = error * kp - derivative * kd;

    if (fabs(total) < minimumSpeed){
      LeftFront.spin(reverse, minimumSpeed, percent);
      LeftMiddle.spin(reverse, minimumSpeed, percent);
      LeftBack.spin(reverse, minimumSpeed, percent);
      RightFront.spin(forward, minimumSpeed, percent);
      RightMiddle.spin(forward, minimumSpeed, percent);
      RightBack.spin(forward, minimumSpeed, percent);
    }
    else {
      LeftFront.spin(reverse, total, percent);
      LeftMiddle.spin(reverse, total, percent);
      LeftBack.spin(reverse, total, percent);
      RightFront.spin(forward, total, percent);
      RightMiddle.spin(forward, total, percent);
      RightBack.spin(forward, total, percent);
    }
    if (error < 30){
      integral += error / 50;
    }

    previousError = error;
    std::cout << error << std::endl;
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

static void defaultDrive(std::string direction, double target){
  if (direction == "Forward"){
    driveWithPID(2.5, 0.1, 0.2, 0.5, 80, target);
  }
  if (direction == "Reverse"){
    driveWithPID(2.5, 0.1, 0.2, 0.5, 80, -target);
  }
}

static void defaultTurn(std::string direction, double target){
  turnWithPIDTo(1, 0.04, 0.1, 2, 30, target);
}


static void slowDrive(std::string direction, double target){ 
  if (direction == "Forward"){
    driveWithPID(1.5, 0.05, 0.1, 0.5, 25, target);
  }
  if (direction == "Reverse"){
    driveWithPID(1.5, 0.05, 0.1, 0.5, 25, -target);
  }
  
}

static void slowTurn(double target){
  turnWithPIDTo(0.7, 0.02, 0.03, 2, 5, target);
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
  driveWithPID(2.5, 0.1, 0.2, 0.5, 85, 10);
}

static void backOut(){
  driveWithPID(2.5, 0.1, 0.2, 0.5, 70, -10);
}

/********** Autons **********/

void runAutonLeftAWP(){
}

void runAutonLeftNoAWP(){
}

void runAutonRightSafe(){ 
}

void runAutonRightFourTB(){
}

void runAutonRightSixTB(){
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
    else{
      Controller1.Screen.setCursor(1, 5);
      Controller1.Screen.print("Auton Selected:");
    }

    if (currentAuton == AutonLeftAWP){
      Controller1.Screen.setCursor(3, 3);
      Controller1.Screen.print("Left-Side Safe AWP");
    }
    if (currentAuton == AutonLeftNoAWP){
      Controller1.Screen.setCursor(3, 5);
      Controller1.Screen.print("Left-Side NO AWP");
    }
    if (currentAuton == AutonRightSafe){
      Controller1.Screen.setCursor(3, 5);
      Controller1.Screen.print("Right-Side Safe");
    }
    if (currentAuton == AutonRightSixTB){
      Controller1.Screen.setCursor(3, 2);
      Controller1.Screen.print("Right-Side Six Triball");
    }

    if (Controller1.ButtonLeft.pressing()){
      Controller1.Screen.clearScreen();
      if (currentAuton == AutonNone || currentAuton == AutonLeftAWP){
        currentAuton = AutonRightSixTB;
      }
      else{
        currentAuton = static_cast<Auton> (static_cast<int> (currentAuton) - 1);
      }
    }
    if (Controller1.ButtonRight.pressing()){
      Controller1.Screen.clearScreen();
      if (currentAuton == AutonRightSixTB){
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
  Controller1.rumble("...");
}

void calibrate(double seconds){
  Inertial.calibrate();
  Controller1.Screen.setCursor(2, 6);
  Controller1.Screen.print("CALIBRATING!!!");
  wait(seconds, sec);
  Controller1.Screen.clearScreen();
}

void tempCheck(double warningTemp){
  double leftDriveTemp = std::max(std::max(LeftFront.temperature(celsius), LeftMiddle.temperature(celsius)), LeftBack.temperature(celsius));
  double rightDriveTemp = std::max(std::max(RightFront.temperature(celsius), RightMiddle.temperature(celsius)), RightBack.temperature(celsius));
  double cataTemp = Catapult.temperature(celsius);
  double intakeTemp = Intake.temperature(celsius); 

  Controller1.Screen.setCursor(2, 8);

  if (leftDriveTemp > warningTemp){
    Controller1.Screen.print("LEFT DRIVE HOT!!!");
    wait(1, sec);
  }
  if (rightDriveTemp > warningTemp){
    Controller1.Screen.print("RIGHT DRIVE HOT!!!");
    wait(1, sec);
  }
  if (cataTemp > warningTemp){
    Controller1.Screen.print("CATAPULT HOT!!!");
    wait(1, sec);
  }
  if (intakeTemp > warningTemp){
    Controller1.Screen.print("INTAKE HOT!!!");
    wait(1, sec);
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
    case AutonLeftNoAWP: {
      runAutonLeftNoAWP();
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
  switch (testedAuton){
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