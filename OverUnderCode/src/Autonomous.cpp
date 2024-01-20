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

  LeftFront.resetPosition();
  RightFront.resetPosition();
  
  while ((fabs(leftDriveError) + fabs(rightDriveError)) / 2 > tolerance){
    //Left Side
    leftDerivative = (previousLeftError - leftDriveError) * 50;
    leftDriveTotal = leftDriveError * kp + leftIntegral * ki - leftDerivative * kd;

    if (fabs(leftDriveTotal) < minimumSpeed){
      LeftFront.spin(forward, getSign(leftDriveError) * minimumSpeed, percent);
      LeftBack.spin(forward, getSign(leftDriveError) * minimumSpeed, percent);
      LeftStack.spin(forward, getSign(leftDriveError) * minimumSpeed, percent);
    }
    else {
      LeftFront.spin(forward, leftDriveTotal, percent);
      LeftBack.spin(forward, leftDriveTotal, percent);
      LeftStack.spin(forward, leftDriveTotal, percent);
    }

    if(fabs(leftDriveError) < maxI){
      leftIntegral += leftDriveError / 50;
    }
    
    //Right Side
    rightDerivative = (previousRightError - rightDriveError) * 50;
    rightDriveTotal = rightDriveError * kp + rightIntegral * ki - rightDerivative * kd;

    if (fabs(rightDriveTotal) < minimumSpeed){
      RightFront.spin(forward, getSign(rightDriveError) * minimumSpeed, percent);
      RightBack.spin(forward, getSign(rightDriveError) * minimumSpeed, percent);
      RightStack.spin(forward, getSign(rightDriveError) * minimumSpeed, percent);
    }
    else {
      RightFront.spin(forward, rightDriveTotal, percent);
      RightBack.spin(forward, rightDriveTotal, percent);
      RightStack.spin(forward, rightDriveTotal, percent);
    }

    if(fabs(rightDriveError) < maxI){
      rightIntegral += rightDriveError / 50;
    }

    previousLeftError = leftDriveError;
    previousRightError = rightDriveError;    

    wait(20, msec);

    leftDriveError = target - LeftFront.position(turns) * 3.25 * M_PI * 4/5;
    rightDriveError = target - RightFront.position(turns) * 3.25 * M_PI * 4/5;
  }

  LeftFront.stop(brake);
  LeftBack.stop(brake);
  LeftStack.stop(brake);
  RightFront.stop(brake);
  RightBack.stop(brake);
  RightStack.stop(brake);

  wait(100, msec);
}

static double turnError(double target){
  
  int throughZeroDirection;
  double smallerDegree = std::min(target, Inertial.heading(degrees));
  double largerDegree = std::max(target, Inertial.heading(degrees));

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
    return target - Inertial.heading(degrees);
  }
}

static void turnWithPID(double kp, double ki, double kd, double tolerance, double minimumSpeed, double maxI, double target){
  double error = turnError(target);
  double integral = 0;
  double derivative;
  double previousError = error;
  double total;
  
  while (fabs(error) > tolerance){
    derivative = (previousError - error) * 100;
    total = error * kp + integral * ki - derivative * kd;

    if (fabs(total) < minimumSpeed){
      LeftFront.spin(forward, getSign(error) * minimumSpeed, percent);
      LeftBack.spin(forward, getSign(error) * minimumSpeed, percent);
      LeftStack.spin(forward, getSign(error) * minimumSpeed, percent);
      RightFront.spin(reverse, getSign(error) * minimumSpeed, percent);
      RightBack.spin(reverse, getSign(error) * minimumSpeed, percent);
      RightStack.spin(reverse, getSign(error) * minimumSpeed, percent);
    }
    else {
      LeftFront.spin(forward, total, percent);
      LeftBack.spin(forward, total, percent);
      LeftStack.spin(forward, total, percent);
      RightFront.spin(reverse, total, percent);
      RightBack.spin(reverse, total, percent);
      RightStack.spin(reverse, total, percent);
    }

    if (fabs(error) < maxI){
      integral += error / 100;
    }

    previousError = error;

    wait(10, msec);

    error = turnError(target);
  }

  LeftFront.stop(brake);
  LeftBack.stop(brake);
  LeftStack.stop(brake);
  RightFront.stop(brake);
  RightBack.stop(brake);
  RightStack.stop(brake);

  wait(100, msec);
}

/********** Tunings **********/

static void crawl(Direction direction, double target){ 
  if (direction == Forward){
    driveWithPID(0, 0, 0, 0.5, 20, 0, target);
  }
  if (direction == Reverse){
    driveWithPID(0, 0, 0, 0.5, 20, 0, -target);
  }
}

static void drive(Direction direction, double target){
  if (direction == Forward){
    driveWithPID(4.5, 0, 0.005, 0.5, 10, 0, target);
  }
  if (direction == Reverse){
    driveWithPID(4.5, 0, 0.005, 0.5, 10, 0, -target);
  }
}

static void ram(Direction direction, double target){
  if (direction == Forward){
    driveWithPID(0, 0, 0, 0.5, 95, 0, target);
  }
  if (direction == Reverse){
    driveWithPID(0, 0, 0, 0.5, 95, 0, -target);  
  }
}

static void turnTo(double target){
  turnWithPID(0.35, 0, 0.015, 2, 10, 0, target);
}

static void intake(){
  Intake.spin(forward, 90, percent);
}

static void outake(double waitTime){
  Intake.spin(reverse, 90, percent);
  wait(waitTime, sec);
  Intake.stop();
}

/********** Autons **********/

void runAutonLeftAWP(){
  intake();
  drive(Forward, 6);
  BackWings.set(true);
  turnTo(330); //Match Load Retrieved

  BackWings.set(false);
  turnTo(0);
  drive(Forward, 10);
  turnTo(45);
  outake(1.5);
  turnTo(225);
  ram(Reverse, 8); //Pre Load Scored

  drive(Forward, 8);
  turnTo(0);
  drive(Reverse, 23);
  turnTo(315);
  Blocker.set(true);
  crawl(Reverse, 35); //Elevation Bar Touched
}

void runAutonLeftNoAWP(){
  turnTo(90);
}

void runAutonLeftSabotage(){
  intake();
  drive(Forward, 36);
  turnTo(285);
  outake(0.5);
  turnTo(90);
  drive(Forward, 10);
  turnTo(0);
  intake();
  drive(Forward, 8);
  wait(0.5, sec);
  turnTo(90);
  drive(Forward, 26); 
  outake(1); //Triball Popped Over

  drive(Reverse, 42);
  turnTo(180);
  drive(Forward, 72);
}

void runAutonRightSafe(){
  double currentTime = Brain.Timer.time();
  intake();
  BackWings.set(true);
  wait(0.3, sec);
  crawl(Reverse, 12);
  turnTo(315);
  ram(Reverse, 20); //Match Load Scored

  drive(Forward, 7);
  BackWings.set(false);
  turnTo(135);
  outake(0.75);
  ram(Forward, 6); //Pre Load Scored

  turnTo(135);
  drive(Reverse, 8);
  turnTo(70);
  intake();
  drive(Forward, 50);
  turnTo(200);
  drive(Forward, 10);
  Intake.setVelocity(40, percent);
  Intake.spin(reverse);
  wait(0.75, sec);
  Intake.stop();
  turnTo(85);
  intake();
  drive(Forward, 14);
  turnTo(45);
  BackWings.set(true);
  ram(Reverse, 36); //Middle and Back Triballs Scored

  drive(Forward, 8);
  BackWings.set(false);
  turnTo(225);
  outake(0.5);
  ram(Forward, 8); //Corner Triball Scored

  std::cout << Brain.Timer.time() - currentTime << std::endl;
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

    if (Controller1.ButtonUp.pressing()){
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
  wait(3, sec);
  Controller1.Screen.clearScreen();
}

void tempCheck(double warningTemp){
  double leftDriveTemp = std::max(std::max(LeftFront.temperature(fahrenheit), LeftBack.temperature(fahrenheit)), LeftStack.temperature(fahrenheit));
  double rightDriveTemp = std::max(std::max(RightFront.temperature(fahrenheit), RightBack.temperature(fahrenheit)), RightStack.temperature(fahrenheit));
  double kickerTemp = Kicker.temperature(fahrenheit);
  double intakeTemp = Intake.temperature(fahrenheit);

  double columns[4] = {4, 3, 6, 6};
  double temperatures[4] = {leftDriveTemp, rightDriveTemp, kickerTemp, intakeTemp};
  std::string mechs[4] = {"LEFT DRIVE", "RIGHT DRIVE", "KICKER", "INTAKE"};

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