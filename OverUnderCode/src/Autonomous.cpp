#include "vex.h"
#include "Autonomous.h"
#include <iostream>

/********** Functions for Auton **********/

static int getSign(double input){
  if (input > 0){ //Returns 1 if positive
    return 1;
  }
  else if (input < 0){ //Returns -1 if negative
    return -1;
  }
  else{
    return 0;
  }
}

static void driveWithPID(double kp, double ki, double kd, double tolerance, double minimumSpeed, double maxI, double target){ //Drives straight
  
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
  
  while ((fabs(leftDriveError) + fabs(rightDriveError)) / 2 > tolerance){ //Runs while not within tolerance
    //Left Side
    leftDerivative = (previousLeftError - leftDriveError) * 50; //Calculate derivative
    leftDriveTotal = leftDriveError * kp + leftIntegral * ki - leftDerivative * kd; //Calculates total output

    if (fabs(leftDriveTotal) < minimumSpeed){ //Runs at minimum speed when calculated output is less
      LeftDrive.spin(forward, getSign(leftDriveError) * minimumSpeed, percent);
    }
    else {
      LeftDrive.spin(forward, leftDriveTotal, percent);
    }

    if(fabs(leftDriveError) < maxI){
      leftIntegral += leftDriveError / 50; //Calculates integral
    }
    
    //Right Side
    rightDerivative = (previousRightError - rightDriveError) * 50; //Calculate derivative
    rightDriveTotal = rightDriveError * kp + rightIntegral * ki - rightDerivative * kd; //Calculates total output

    if (fabs(rightDriveTotal) < minimumSpeed){ //Runs at minimum speed when calculated output is less
      RightDrive.spin(forward, getSign(rightDriveError) * minimumSpeed, percent);
    }
    else {
      RightDrive.spin(forward, rightDriveTotal, percent);
    }

    if(fabs(rightDriveError) < maxI){
      rightIntegral += rightDriveError / 50; //Calculates integral
    }

    previousLeftError = leftDriveError; //Makes current error previous error for next loop
    previousRightError = rightDriveError;    

    wait(20, msec);

    leftDriveError = target - LeftFront.position(turns) * 3.25 * M_PI * 3/4;
    rightDriveError = target - RightFront.position(turns) * 3.25 * M_PI * 3/4;
  }

  LeftDrive.stop(brake);
  RightDrive.stop(brake);
}

static double turnError(double target){ //Calculates error for the shortest path to target heading
  double output;

  if (fabs(target - Inertial.heading(degrees)) < 180){
    output = target - Inertial.heading(degrees);
  }
  else {
    output = (360 - fabs(target - Inertial.heading(degrees)));
    if (target > Inertial.heading(degrees)){
      return -output;
    }
  }

  return output;
}

static void turnWithPID(double kp, double ki, double kd, double tolerance, double minimumSpeed, double maxI, double target){ //Turns in place
  double error = turnError(target);
  double integral = 0;
  double derivative;
  double previousError = error;
  double total;
  
  while ((fabs(error) > tolerance) || derivative > 4){ //Runs while not within tolerance and motors are still spinning quickly (prevents drift)
    derivative = (previousError - error) * 100; //Calculates derivative
    total = error * kp + integral * ki - derivative * kd; //Calculates total output

    if (fabs(total) < minimumSpeed){ //Runs at minimum speed when calculated output is less
      LeftDrive.spin(forward, getSign(error) * minimumSpeed, percent);
      RightDrive.spin(reverse, getSign(error) * minimumSpeed, percent);
    }
    else {
      LeftDrive.spin(forward, total, percent);
      RightDrive.spin(reverse, total, percent);
    }

    if (fabs(error) < maxI){
      integral += error / 100; //Calculates integral
    }

    previousError = error; //Makes current error previous error for next loop

    wait(10, msec);

    error = turnError(target);
  }

  LeftDrive.stop(brake);
  RightDrive.stop(brake);
}

static double swingError(double target, Direction side, Direction direction){ //Calculates error for desired direction and drivetrain side
  double output;
  double currentHeading = Inertial.heading(degrees);
  double smallerDegree = std::min(target, currentHeading);
  double largerDegree = std::max(target, currentHeading);
  Direction rotation;


  if (((side == Left) && (direction == Forward)) || ((side == Right) && (direction == Reverse))){
    rotation = Clockwise;
  }
  else{
    rotation = CounterClockwise;
  }

  if (fabs(target - Inertial.heading(degrees)) > 5){
    if (rotation == Clockwise){
      if (target > Inertial.heading(degrees)){
        output = target - Inertial.heading(degrees);
      }
      else {
        output = 360 - Inertial.heading(degrees) + target;
      }
    }
    else {
      if (target < Inertial.heading(degrees)){
        output = target - Inertial.heading();
      }
      else {
        output = -(360 - target + Inertial.heading(degrees));
      }
    }
  }
  else {
    if (fabs(target - Inertial.heading(degrees)) < 180){
      output = target - Inertial.heading(degrees);
    }
    else {
      output = (360 - fabs(target - Inertial.heading(degrees)));
      if (target > Inertial.heading(degrees)){
        return -output;
      }
    }
  }

  if (side == Right){
    return -output;
  }
  
  return output;
}

static void swingWithPID(Direction side, Direction direction, double kp, double ki, double kd, double tolerance, double minimumSpeed, double maxI, double target){ //Turns one side of drivetrain
  double error = swingError(target, side, direction);
  double integral = 0;
  double derivative;
  double previousError = error;
  double total;
  
  while ((fabs(error) > tolerance) || derivative > 4){ //Runs while not within tolerance and motors are still spinning quickly (prevents drift)
    derivative = (previousError - error) * 100; //Calculates derivative
    total = error * kp + integral * ki - derivative * kd; //Calculates total output

    if (side == Left){
      if (fabs(total) < minimumSpeed){ //Runs at minimum speed when calculated output is less
        LeftDrive.spin(forward, getSign(error) * minimumSpeed, percent);
      }
      else {
        LeftDrive.spin(forward, total, percent);
      }
    }
    else{
      if (fabs(total) < minimumSpeed){ //Runs at minimum speed when calculated output is less
        RightDrive.spin(forward, getSign(error) * minimumSpeed, percent);
      }
      else {
        RightDrive.spin(forward, total, percent);
      }
    }

    if (fabs(error) < maxI){
      integral += error / 100; //Calculates integral
    }

    previousError = error; //Makes current error previous error for next loop

    wait(10, msec);

    error = swingError(target, side, direction);
  }

  LeftDrive.stop(brake);
  RightDrive.stop(brake);
}

/********** Tunings **********/

void crawl(Direction direction, double target){ 
  if (direction == Forward){
    driveWithPID(0, 0, 0, 0.5, 15, 0, target);
  }
  if (direction == Reverse){
    driveWithPID(0, 0, 0, 0.5, 15, 0, -target);
  }
}

void drive(Direction direction, double target){
  if (direction == Forward){
    driveWithPID(4.5, 0, 0.004, 0.5, 15, 0, target);
  }
  if (direction == Reverse){
    driveWithPID(4.5, 0, 0.004, 0.5, 15, 0, -target);
  }
}

void ram(Direction direction, double target){
  if (direction == Forward){
    driveWithPID(0, 0, 0, 2, 70, 0, target);
  }
  if (direction == Reverse){
    driveWithPID(0, 0, 0, 2, 70, 0, -target);
  }
}

void turnTo(double target){
  turnWithPID(0.4, 0.1, 0.00115, 2, 3, 20, target);
}

void swingTo(double target, Direction side, Direction direction){
  swingWithPID(side, direction, 0.6, 0.1, 0.007, 3, 15, 90, target);
}

void intake(){
  Intake.spin(forward, 90, percent);
}

void outake(double waitTime){
  Intake.spin(reverse, 100, percent);
  wait(waitTime, sec);
}

/********** Pre Auton **********/

static Auton currentAuton = AutonNone;

static void autonSelector(){
  bool runningSelector = true;

  int columns[6] = {2, 3, 3, 5, 2, 2};
  std::string autonNames[6] = {"Left-Side Safe AWP", "Left-Side NO AWP", "Left-Side Sabotage", 
                               "Right-Side Quals", "Right-Side Elims Safe", "Right-Side Elims Risky"};
  Auton autons[6] = {AutonLeftAWP, AutonLeftNoAWP, AutonLeftSabotage, AutonRightQuals, AutonRightElimsSafe, AutonRightElimsRisky};

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

    for (int i = 0; i < 6; i++){
      if (currentAuton == autons[i]){ //Displays auton label
        Controller1.Screen.setCursor(3, columns[i]);
        Controller1.Screen.print(autonNames[i].c_str());
      }
    }

    if (Controller1.ButtonLeft.pressing() && !buttonLeftPressed){ //Pressing left button go left on auton list
      Controller1.Screen.clearScreen();
      if (currentAuton == AutonNone || currentAuton == AutonLeftAWP){
        currentAuton = AutonRightElimsRisky;
      }
      else{
        currentAuton = static_cast<Auton> (static_cast<int> (currentAuton) - 1);
      }

      buttonLeftPressed = true;
    }
    if (!Controller1.ButtonLeft.pressing() && buttonLeftPressed){
      buttonLeftPressed = false;
    }

    if (Controller1.ButtonRight.pressing() && !buttonRightPressed){ //Pressing right button go left on auton list
      Controller1.Screen.clearScreen();
      if (currentAuton == AutonRightElimsRisky){
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

    if (Controller1.ButtonUp.pressing()){ //Exits selector when up button is pressed
      Controller1.Screen.clearScreen();
      runningSelector = false;
    }

    wait(50, msec);
  }
  Controller1.rumble("-.-.");
}

static void calibrateInertial(){ //Calibrates inertial sensor for three seconds
  Controller1.Screen.setCursor(2, 6);
  Inertial.calibrate();
  Controller1.Screen.print("CALIBRATING!!!");
  wait(3, sec);
  Controller1.Screen.clearScreen();
}

static void tempCheck(double warningTemp){
  double leftDriveTemp = std::max(std::max(LeftFront.temperature(fahrenheit), //Gets average left drive temperature
                         LeftMiddle.temperature(fahrenheit)), LeftBack.temperature(fahrenheit));
  double rightDriveTemp = std::max(std::max(RightFront.temperature(fahrenheit), //Gets average right drive temperature
                          RightMiddle.temperature(fahrenheit)), RightBack.temperature(fahrenheit));
  double kickerTemp = Kicker.temperature(fahrenheit);
  double intakeTemp = Intake.temperature(fahrenheit);

  double columns[4] = {4, 3, 6, 6}; //Makes the text centered on controller
  double temperatures[4] = {leftDriveTemp, rightDriveTemp, kickerTemp, intakeTemp};
  std::string mechs[4] = {"LEFT DRIVE", "RIGHT DRIVE", "KICKER", "INTAKE"};

  for (int i = 0; i < 4; i++){ //Runs once for each mechanism
    if (temperatures[i] > warningTemp){ //Checks if temperature is exceeding warning temp
      Controller1.Screen.setCursor(1, columns[i]);
      Controller1.Screen.print(mechs[i].c_str());
      Controller1.Screen.print(" HOT!!!"); 
      Controller1.Screen.setCursor(3, 10);
      Controller1.Screen.print(temperatures[i]);
      //Example on controller screen:

      //LEFT DRIVE HOT!!!
      
      //    125

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
    case AutonRightQuals: {
      runAutonRightQuals();
      break;
    }
    case AutonRightElimsSafe: {
      runAutonRightElimsSafe();
      break;
    }
    case AutonRightElimsRisky: {
      runAutonRightElimsRisky();
    }
    default: {
      break;
    }
  }
}

void testAuton(Auton testedAuton){
  double startTime = Brain.Timer.time(); //Records start time
  Controller1.Screen.setCursor(2, 3);

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
    case AutonLeftSabotage: {
      runAutonLeftSabotage();
      break;
    }
    case AutonRightElimsSafe: {
      runAutonRightElimsSafe();
      break;
    }
    case AutonRightElimsRisky: {
      runAutonRightElimsRisky();
    }
    default: {
      break;
    }
  }

  Controller1.Screen.print((Brain.Timer.time() - startTime) / 1000); //Records time spent
}