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

static double shorterTurningPathError(double target, double error){
  
  double smallerDegree = std::min(target, Inertial.heading());
  double largerDegree = std::max(target, Inertial.heading());
  static int throughZeroDirection;

  if(smallerDegree == target){
    throughZeroDirection = 1;
  }
  else{
    throughZeroDirection = -1; 
  }

  if(360 - largerDegree + smallerDegree < 180){
    return throughZeroDirection * (360 - largerDegree + smallerDegree);
  }
  else{
    return target - Inertial.heading();
  }
}

static void turnWithPIDTo(double kp, double ki, double kd, double tolerance, double minimumSpeed, double target){
  
  double error = target;
  double integral = 0;
  double derivative = 0;
  double previousError = error;
  double total = 0;
  
  while (fabs(error) > tolerance){
    
    error = shorterTurningPathError(target, error);

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
<<<<<<< HEAD
    driveWithPID(2.5, 0.1, 0.2, 0.5, 80, target);
  }
  if (direction == "Reverse"){
    driveWithPID(2.5, 0.1, 0.2, 0.5, 80, -target);
=======
    driveForward(2.5, 0.1, 0.2, 0.5, 70, target);
  }
  if (direction == "Reverse"){
    driveReverse(2.5, 0.1, 0.2, 0.5, 70, target);
>>>>>>> parent of 09b6bcc (Winters)
  }
  
}

static void defaultTurn(std::string direction, double target){
<<<<<<< HEAD
  turnWithPIDTo(1, 0.04, 0.1, 2, 30, target);
=======
  if (direction == "Clockwise"){
    turnClockwise(1, 0.04, 0.07, 2, 15, target);
  }
  if (direction == "CounterClockwise"){
    turnCounterClockwise(1, 0.04, 0.07, 2, 15, target);  
  } 
>>>>>>> parent of 09b6bcc (Winters)
}


static void slowDrive(std::string direction, double target){ 
  if (direction == "Forward"){
    driveWithPID(1.5, 0.05, 0.1, 0.5, 25, target);
  }
  if (direction == "Reverse"){
    driveWithPID(1.5, 0.05, 0.1, 0.5, 25, -target);
  }
  
}

<<<<<<< HEAD
static void slowTurn(double target){
  turnWithPIDTo(0.7, 0.02, 0.03, 2, 5, target);
=======
static void slowTurn(std::string direction, double target){
  if (direction == "Clockwise"){
    turnClockwise(0.7, 0.02, 0.03, 2, 15, target);
  }
  if (direction == "CounterClockwise"){
    turnCounterClockwise(0.7, 0.02, 0.03, 2, 15, target);  
  } 
>>>>>>> parent of 09b6bcc (Winters)
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
<<<<<<< HEAD
  driveWithPID(2.5, 0.1, 0.2, 0.5, 85, 10);
=======
  driveForward(2.5, 0.1, 0.2, 0.5, 85, 12);
>>>>>>> parent of 09b6bcc (Winters)
}

static void backOut(){
  driveWithPID(2.5, 0.1, 0.2, 0.5, 70, -10);
}

/********** Autons **********/

void runAutonLeftAWP(){
<<<<<<< HEAD
}

void runAutonLeftNoAWP(){
}

void runAutonRightSafe(){ 
=======
  //Score Pre Load
  slowDrive("Forward", 20);
  slowTurn("Clockwise", 45);
  outake(0.5);
  scoreTriball(); //Pre Load Scored
  stopIntake();

  //Pull Out Match Load
  slowDrive("Reverse", 14);
  slowTurn("CounterClockwise", 45);
  slowDrive("Reverse", 14);
  slowTurn("Clockwise", 90);
  slowDrive("Reverse", 6);
  Arm.set(true);
  wait(0.3, sec);
  defaultTurn("CounterClockwise", 125); //Match Load Pulled Out
  Arm.set(false);
  slowDrive("Reverse", 8);
  defaultTurn("Clockwise", 45);
  slowDrive("Reverse", 8);
  defaultTurn("CounterClockwise", 45);

  //Touch Bar
  defaultDrive("Reverse", 32);
}

void runAutonLeftSabotage(){
  
}

void runAutonRightSafe(){ 
  //Score Pre Load
  slowDrive("Forward", 40);
  slowTurn("Clockwise", 90); //Counter Clockwise Movement from Dropping Intake
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
>>>>>>> parent of 09b6bcc (Winters)
}

void runAutonRightFourTB(){
}

<<<<<<< HEAD
void runAutonRightSixTB(){
=======
  //Score Pre Load
  defaultDrive("Forward", 38);
  defaultTurn("Clockwise", 80); //Counter Clockwise Movement from Dropping Intake
  outake(0.25);
  scoreTriball(); //Pre Load Scored

  //Middle Triball
  backOut();
  defaultTurn("CounterClockwise", 120); //Face Triball
  intake();
  defaultDrive("Forward", 12); //Triball Picked Up
  stopIntake();
  defaultTurn("Clockwise", 120);
  outake(0.75);

  //Back Triball
  defaultTurn("Clockwise", 180); //Face Triball
  intake();
  defaultDrive("Forward", 14); //Triball Picked Up
  stopIntake();
  defaultTurn("CounterClockwise", 180);
  defaultDrive("Forward", 14);
  outake(0.25);
  defaultTurn("Clockwise", 180);
  Wings.set(true);
  defaultDrive("Reverse", 15); //Middle and Back Triball Scored
  Wings.set(false);

  //Side Triball
  defaultDrive("Forward", 20);
  defaultTurn("CounterClockwise", 50);
  intake();
  defaultDrive("Forward", 24); //Triball Picked Up
  stopIntake();
  defaultDrive("Reverse", 18);
  slowTurn("CounterClockwise", 130);
  defaultDrive("Forward", 10);
  outake(0.25);
  scoreTriball(); //Side Triball Scored
  stopIntake();
  
  std::cout << (Brain.Timer.time() - currentTime) / 1000 << " seconds" << std::endl;
>>>>>>> parent of 09b6bcc (Winters)
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
<<<<<<< HEAD
      if (currentAuton == AutonNone || currentAuton == AutonLeftAWP){
        currentAuton = AutonRightSixTB;
=======
      if (currentAuton == AutonLeftAWP){
        currentAuton = AutonRightFourTB;
>>>>>>> parent of 09b6bcc (Winters)
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
<<<<<<< HEAD
  double leftDriveTemp = std::max(std::max(LeftFront.temperature(celsius), LeftMiddle.temperature(celsius)), LeftBack.temperature(celsius));
  double rightDriveTemp = std::max(std::max(RightFront.temperature(celsius), RightMiddle.temperature(celsius)), RightBack.temperature(celsius));
  double cataTemp = Catapult.temperature(celsius);
  double intakeTemp = Intake.temperature(celsius); 
=======
  double leftDriveTemp = std::max(std::max(LeftFront.temperature(fahrenheit), LeftBack.temperature(fahrenheit)), LeftStack.temperature(fahrenheit));
  double rightDriveTemp = std::max(std::max(RightFront.temperature(fahrenheit), RightBack.temperature(fahrenheit)), RightStack.temperature(fahrenheit));
  double cataTemp = Catapult.temperature(fahrenheit);
  double intakeTemp = Intake.temperature(fahrenheit); 
>>>>>>> parent of 09b6bcc (Winters)

  Controller1.Screen.setCursor(2, 8);

  if (leftDriveTemp > warningTemp){
    Controller1.Screen.print("LEFT DRIVE HOT!!!");
<<<<<<< HEAD
    wait(1, sec);
=======
>>>>>>> parent of 09b6bcc (Winters)
  }
  if (rightDriveTemp > warningTemp){
    Controller1.Screen.print("RIGHT DRIVE HOT!!!");
<<<<<<< HEAD
    wait(1, sec);
=======
>>>>>>> parent of 09b6bcc (Winters)
  }
  if (cataTemp > warningTemp){
    Controller1.Screen.print("CATAPULT HOT!!!");
<<<<<<< HEAD
    wait(1, sec);
=======
>>>>>>> parent of 09b6bcc (Winters)
  }
  if (intakeTemp > warningTemp){
    Controller1.Screen.print("INTAKE HOT!!!");
<<<<<<< HEAD
    wait(1, sec);
=======
>>>>>>> parent of 09b6bcc (Winters)
  }
}

void preAuton(){
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  Controller1.Screen.clearScreen();
  calibrate(3);
  tempCheck(130);
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