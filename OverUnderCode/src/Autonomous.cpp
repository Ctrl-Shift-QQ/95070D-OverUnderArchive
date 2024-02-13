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
  double currentHeading = Inertial.heading(degrees);
  double smallerDegree = std::min(target, currentHeading);
  double largerDegree = std::max(target, currentHeading);
  
  if ((largerDegree - smallerDegree) > 180){
    output = (360 - largerDegree + smallerDegree);
    if (smallerDegree == currentHeading){
      return -output;
    }
  }
  else {
    output = target - currentHeading;
  }

  return output;
}

static void turnWithPID(double kp, double ki, double kd, double tolerance, double minimumSpeed, double maxI, double target){
  double error = turnError(target);
  double integral = 0;
  double derivative;
  double previousError = error;
  double total;
  
  while ((fabs(error) > tolerance) || derivative > 3){ //Runs while not within tolerance and motors are still spinning quickly (prevents drift)
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


  if ((side == Left && direction == Forward) || (side == Right && direction == Reverse)){
    rotation = Clockwise;
  }
  else{
    rotation = CounterClockwise;
  }

  if ((largerDegree - smallerDegree) > 180){
    if ((rotation == Clockwise && largerDegree == currentHeading) || (rotation == CounterClockwise && largerDegree == target)){
      output = fabs(360 - largerDegree + smallerDegree);
    }
  }
  else{
    output = largerDegree - smallerDegree;
  }

  if (direction == Reverse){
    return -output;
  }
  
  return output;
}

static void swingWithPID(Direction side, Direction direction, double kp, double ki, double kd, double tolerance, double minimumSpeed, double maxI, double target){
  double error = swingError(target, side, direction);
  double integral = 0;
  double derivative;
  double previousError = error;
  double total;
  
  while ((fabs(error) > tolerance) || derivative > 3){ //Runs while not within tolerance and motors are still spinning quickly (prevents drift)
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

    error = turnError(target);
  }

  LeftDrive.stop(brake);
  RightDrive.stop(brake);
}

/********** Tunings **********/

static void crawl(Direction direction, double target){ 
  if (direction == Forward){
    driveWithPID(0, 0, 0, 0.5, 15, 0, target);
  }
  if (direction == Reverse){
    driveWithPID(0, 0, 0, 0.5, 15, 0, -target);
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
    driveWithPID(0, 0, 0, 1, 70, 0, target);
  }
  if (direction == Reverse){
    driveWithPID(0, 0, 0, 1, 70, 0, -target);  
  }
}

static void turnTo(double target){
  turnWithPID(0.45, -0.2, 0.0085, 3, 5, 20, target);
}

static void swingTo(double target, Direction side, Direction direction){
  swingWithPID(side, direction, 0.7, 0.2, 0.01, 2, 20, 90, target);
}

static void intake(){
  Intake.spin(forward, 90, percent);
}

static void outake(double waitTime){
  Intake.spin(reverse, 90, percent);
  wait(waitTime, sec);
}

/********** Autons **********/

void runAutonLeftAWP(){
  drive(Forward, 7);
  LeftWing.set(true);
  turnTo(330); //Match Load Descored

  LeftWing.set(false);
  turnTo(25);
  drive(Forward, 14);
  wait(100, msec);
  turnTo(45);
  wait(100, msec);
  outake(2);
  Intake.stop();
  wait(100, msec);
  drive(Reverse, 4);
  turnTo(225);
  wait(100, msec);
  ram(Reverse, 10); //Preload Scored

  turnTo(225);
  drive(Forward, 10);
  turnTo(180);
  drive(Forward, 28);
  wait(100, msec);
  turnTo(135);
  outake(0);
  crawl(Forward, 30); //Elevation Bar Touched
  wait(3, sec);
  Intake.stop();
}

void runAutonLeftNoAWP(){
}

void runAutonLeftSabotage(){
}

void runAutonRightSafe(){
  double currentTime = Brain.Timer.time(msec);

  LeftWing.set(true);
  drive(Reverse, 6);
  turnTo(330);
  LeftWing.set(false);
  turnTo(0);
  drive(Reverse, 17);
  turnTo(315);
  ram(Reverse, 3); //Match Load Scored

  turnTo(315);
  drive(Forward, 8);
  turnTo(135);
  outake(0.75);
  ram(Forward, 10); //Preload Scored  

  turnTo(135);
  ram(Reverse, 8);
  turnTo(67.5);
  intake();
  drive(Forward, 48);
  turnTo(190);
  drive(Forward, 8);
  Intake.setVelocity(40, percent);
  Intake.spin(reverse);
  wait(1, sec);
  turnTo(85);
  intake();
  drive(Forward, 16);
  wait(100, msec);
  turnTo(45);
  LeftWing.set(true);
  RightWing.set(true);
  ram(Reverse, 20); //Middle, Back, and Corner Triballs Scored

  drive(Forward, 6);
  LeftWing.set(false);
  RightWing.set(false);
  turnTo(225);
  ram(Forward, 10);
  ram(Reverse, 10);
  turnTo(150);
  drive(Reverse, 30); 


  std::cout << (Brain.Timer.time(msec) - currentTime) / 1000 << std::endl;
}

void runAutonRightSixTB(){ 
}

/********** Pre Auton **********/

static Auton currentAuton = AutonNone;

static void autonSelector(){
  bool runningSelector = true;

  int columns[5] = {2, 3, 3, 5, 2};
  std::string autonNames[5] = {"Left-Side Safe AWP", "Left-Side NO AWP", "Left-Side Sabotage", 
                               "Right-Side Safe", "Right-Side Six Triball"};
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
      if (currentAuton == autons[i]){ //Displays auton label
        Controller1.Screen.setCursor(3, columns[i]);
        Controller1.Screen.print(autonNames[i].c_str());
      }
    }

    if (Controller1.ButtonLeft.pressing() && !buttonLeftPressed){ //Pressing left button go left on auton list
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

    if (Controller1.ButtonRight.pressing() && !buttonRightPressed){ //Pressing right button go left on auton list
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

    if (Controller1.ButtonUp.pressing()){ //Exits selector when up button is pressed
      Controller1.Screen.clearScreen();
      runningSelector = false;
    }

    wait(50, msec);
  }
  Controller1.rumble("-.-.");
}

void calibrateInertial(){ //Calibrates inertial sensor for three seconds
  Controller1.Screen.setCursor(2, 6);
  Inertial.calibrate();
  Controller1.Screen.print("CALIBRATING!!!");
  wait(3, sec);
  Controller1.Screen.clearScreen();
}

void tempCheck(double warningTemp){
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
  double startTime = Brain.Timer.time(); //Records start time

  switch (testedAuton){
    case AutonNone: {
      break;
    }
    case AutonLeftAWP: {
      runAutonLeftAWP();
      std::cout << "Time: " << (Brain.Timer.time() - startTime) / 1000 << std::endl; //Records time spent
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