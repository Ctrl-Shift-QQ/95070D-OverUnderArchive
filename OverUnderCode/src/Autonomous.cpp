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
      leftIntegral += leftDriveError / 50; //Calculates integral
    }
    
    //Right Side
    rightDerivative = (previousRightError - rightDriveError) * 50; //Calculate derivative
    rightDriveTotal = rightDriveError * kp + rightIntegral * ki - rightDerivative * kd; //Calculates total output

    if (fabs(rightDriveTotal) < minimumSpeed){ //Runs at minimum speed when calculated output is less
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
      rightIntegral += rightDriveError / 50; //Calculates integral
    }

    previousLeftError = leftDriveError; //Makes current error previous error for next loop
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
}

static double turnError(double target){ //Calculates error for the shortest path to target heading
  
  double output;
  double smallerDegree = std::min(target, Inertial.heading(degrees));
  double largerDegree = std::max(target, Inertial.heading(degrees));


  if (largerDegree - smallerDegree > 180){
    output = (360 - largerDegree + smallerDegree);
    if (smallerDegree = Inertial.heading()){
      return -output;
    }
  }
  else {
    output = target - Inertial.heading(degrees);
    if (smallerDegree = target){
      return -output;
    }
  }
  
  return output;
}

static void turnWithPID(double kp, double ki, double kd, double tolerance, double minimumSpeed, double maxI, double target){
  double error = turnError(target);
  double integral = 0;
  double derivative;
  double previousError = error;
  double total;
  
  while ((fabs(error) > tolerance) || fabs(RightFront.velocity(percent) > 1)){ //Runs while not within tolerance and motors are still spinning quickly (prevents drift)
    derivative = (previousError - error) * 100; //Calculates derivative
    total = error * kp + integral * ki - derivative * kd; //Calculates total output

    if (fabs(total) < minimumSpeed){ //Runs at minimum speed when calculated output is less
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
      integral += error / 100; //Calculates integral
    }

    previousError = error; //Makes current error previous error for next loop

    wait(10, msec);

    error = turnError(target);
  }

  LeftFront.stop(brake);
  LeftBack.stop(brake);
  LeftStack.stop(brake);
  RightFront.stop(brake);
  RightBack.stop(brake);
  RightStack.stop(brake);
}

static double swingError(double target, Direction direction, Direction side){ //Calculates error for desired direction and drivetrain side
  
  double output;
  double smallerDegree = std::min(target, Inertial.heading(degrees));
  double largerDegree = std::max(target, Inertial.heading(degrees));

  if ((largerDegree == target && direction == Clockwise) || 
    (largerDegree == Inertial.heading() && direction == CounterClockwise)){
    output = fabs(largerDegree - smallerDegree);
  }
  else{
    output = (360 - largerDegree + smallerDegree);
  }
  
  if (direction == CounterClockwise){
    output = -output;
  }

  if (side == Left){
    return -output;
  }
  
  return output;
}

static void swingWithPID(Direction direction, Direction side, double kp, double ki, double kd, double tolerance, double minimumSpeed, double maxI, double target){
  double error = swingError(target, direction, side);
  double integral = 0;
  double derivative;
  double previousError = error;
  double total;
  
  while ((fabs(error) > tolerance) || fabs(RightFront.velocity(percent) > 1)){ //Runs while not within tolerance and motors are still spinning quickly (prevents drift)
    derivative = (previousError - error) * 100; //Calculates derivative
    total = error * kp + integral * ki - derivative * kd; //Calculates total output

    if (side == Left){
      if (fabs(total) < minimumSpeed){ //Runs at minimum speed when calculated output is less
        LeftFront.spin(forward, getSign(error) * minimumSpeed, percent);
        LeftBack.spin(forward, getSign(error) * minimumSpeed, percent);
        LeftStack.spin(forward, getSign(error) * minimumSpeed, percent);
      }
      else {
        LeftFront.spin(forward, total, percent);
        LeftBack.spin(forward, total, percent);
        LeftStack.spin(forward, total, percent);
      }
    }
    else{
      if (fabs(total) < minimumSpeed){ //Runs at minimum speed when calculated output is less
        RightFront.spin(reverse, getSign(error) * minimumSpeed, percent);
        RightBack.spin(reverse, getSign(error) * minimumSpeed, percent);
        RightStack.spin(reverse, getSign(error) * minimumSpeed, percent);
      }
      else {
        RightFront.spin(reverse, total, percent);
        RightBack.spin(reverse, total, percent);
        RightStack.spin(reverse, total, percent);
      }
    }

    if (fabs(error) < maxI){
      integral += error / 100; //Calculates integral
    }

    previousError = error; //Makes current error previous error for next loop

    wait(10, msec);

    error = turnError(target);
  }

  LeftFront.stop(brake);
  LeftBack.stop(brake);
  LeftStack.stop(brake);
  RightFront.stop(brake);
  RightBack.stop(brake);
  RightStack.stop(brake);
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
    driveWithPID(0, 0, 0, 1, 70, 0, target);
  }
  if (direction == Reverse){
    driveWithPID(0, 0, 0, 1, 70, 0, -target);  
  }
}

static void turnTo(double target){
  turnWithPID(0.6, -0.06, 0.0575, 2, 0, 20, target);
}

static void swingTo(double target, Direction direction, Direction side){
  swingWithPID(direction, side, 0, 0, 0, 2, 0, 0, target);
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
  BackWings.set(true);
  turnTo(320); //Match Load Retrieved

  wait(100, msec);
  BackWings.set(false);
  turnTo(0);
  wait(100, msec);
  drive(Forward, 6);
  turnTo(20);
  wait(100, msec);
  drive(Forward, 5);
  turnTo(45);
  wait(100, msec);
  drive(Forward, 4);
  outake(2);
  drive(Reverse, 2);
  Intake.stop();
  turnTo(225);
  wait(100, msec);
  ram(Reverse, 10); //Pre Load Scored

  wait(100, msec);
  turnTo(225);
  wait(100, msec);
  drive(Forward, 2);
  turnTo(0);
  wait(100, msec);
  drive(Reverse, 28.5);
  turnTo(135);
  wait(400, msec);
  turnTo(135);
  wait(400, msec);
  turnTo(135);
  outake(0);
  crawl(Forward, 27); //Elevation Bar Touched
}

void runAutonLeftNoAWP(){
  drive(Forward, 7);
  BackWings.set(true);
  turnTo(320); //Match Load Retrieved

  wait(100, msec);
  BackWings.set(false);
  turnTo(0);
  wait(100, msec);
  drive(Forward, 6);
  turnTo(20);
  wait(100, msec);
  drive(Forward, 5);
  turnTo(45);
  wait(100, msec);
  drive(Forward, 4);
  outake(2);
  drive(Reverse, 2);
  Intake.stop();
  turnTo(225);
  wait(100, msec);
  ram(Reverse, 10); //Pre Load Scored

  wait(100, msec);
  turnTo(225);
  wait(100, msec);
  drive(Forward, 2);
  turnTo(0);
  wait(100, msec);
  drive(Reverse, 28.5);
  turnTo(135);
  wait(400, msec);
  turnTo(135);
  wait(400, msec);
  turnTo(135);
  outake(0);
  drive(Forward, 27);
  ram(Reverse, 29); //Match Load Bar Touched
}

void runAutonLeftSabotage(){
  intake();
  drive(Forward, 37);
  wait(250, msec);
  turnTo(290);
  outake(1.5);
  turnTo(270);
  Intake.stop();
  BackWings.set(true);
  wait(500, msec);
  ram(Reverse, 30); //Triballs Messed Up

  BackWings.set(false);
  turnTo(270);
  drive(Forward, 22);
  turnTo(90);
  BackWings.set(true);
  ram(Reverse, 15); //Preload Scored

  drive(Forward, 3);
  BackWings.set(false);
  turnTo(180);
  drive(Forward, 38);
  wait(100, msec);
  turnTo(225);
  drive(Forward, 14); //Ready To Match Load
}

void runAutonRightSafe(){
  // BackWings.set(true);
  // wait(0.2, sec);
  ram(Reverse, 13);
  // wait(750, msec);
  // BackWings.set(false);
  turnTo(135);
  wait(50, msec);
  outake(1.2);
  ram(Forward, 8); //Pre Load Scored

  turnTo(135);
  ram(Reverse, 8);
  turnTo(62);
  intake();
  drive(Forward, 52);
  turnTo(180);
  drive(Forward, 12);
  turnTo(195);
  Intake.setVelocity(30, percent);
  Intake.spin(reverse);
  wait(0.75, sec);
  Intake.stop();
  turnTo(70);
  intake();
  drive(Forward, 16);
  wait(100, msec);
  turnTo(45);
  turnTo(45);
  wait(40, msec);
  BackWings.set(true);
  ram(Reverse, 23); //Middle and Back Triballs Scored

  turnTo(45);
  drive(Forward, 4);
  BackWings.set(false);
  wait(10, msec);
  turnTo(225);
  outake(0.5);
  Intake.stop();
  ram(Forward, 12); //Corner Triball Scored

  drive(Reverse, 10);
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
                         LeftBack.temperature(fahrenheit)), LeftStack.temperature(fahrenheit));
  double rightDriveTemp = std::max(std::max(RightFront.temperature(fahrenheit), //Gets average right drive temperature
                          RightBack.temperature(fahrenheit)), RightStack.temperature(fahrenheit));
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