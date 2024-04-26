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

static double driveError(double target, double wheelDiameter, double gearRatio){ //Gear ratio in driving over driven
  return target - (LeftDrive.position(turns) + RightFront.position(turns)) / 2 * wheelDiameter * M_PI * gearRatio; //Calculates error in inches
}

void driveWithPID(double target, double kp, double ki, double kd, double tolerance, double minimumSpeed, double maxI){ //Drives straight
  double error = target;
  double integral = 0;
  double derivative;
  double previousError = error;
  double total;

  LeftFront.resetPosition();
  RightFront.resetPosition();
  
  while (fabs(error) > tolerance){ //Runs while not within tolerance
    //Left Side
    derivative = (previousError - error) * 50; //Calculate derivative
    total = error * kp + integral * ki - derivative * kd; //Calculates total output

    if (fabs(total) < minimumSpeed){ //Runs at minimum speed when calculated output is less
      LeftDrive.spin(forward, getSign(error) * minimumSpeed, percent);
      RightDrive.spin(forward, getSign(error) * minimumSpeed, percent);
    }
    else {
      LeftDrive.spin(forward, total, percent);
      RightDrive.spin(forward, total, percent);
    }

    if(fabs(error) < maxI){
      integral += error / 50; //Calculates integral
    }

    wait(20, msec);
    
    error = driveError(target, 3.25, 0.75); //Calculates error in inches
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

void turnWithPID(double target, double kp, double ki, double kd, double tolerance, double exitSpeed, double minimumSpeed, double maxI){ //Turns in place
  double error = turnError(target);
  double integral = 0;
  double derivative;
  double previousError = error;
  double total;

  while ((fabs(error) > tolerance) || fabs(derivative) > exitSpeed){ //Runs while not within tolerance and motors are still spinning quickly (prevents drift)
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

static double swingError(double target, Direction leadSide, Direction direction){ //Calculates error for desired direction and drivetrain side
  double output;

  output = target - Inertial.rotation(degrees);
  
  if (leadSide == Right){
    return -output;
  }

  return output;
}

void swingWithPID(double target, Direction leadSide, Direction direction, double percentage, double kp, double ki, double kd, double tolerance, double exitSpeed, double minimumSpeed, double maxI){ //Turns one side of drivetrain
  double error = swingError(target, leadSide, direction);
  double turnDirection;
  double integral = 0;
  double derivative;
  double previousError = error;
  double total;
  Inertial.setRotation(Inertial.heading(degrees), degrees);

  if ((leadSide == Left && direction == Forward) || (leadSide == Right && direction == Reverse)){
    turnDirection = Clockwise;
  }
  else {
    turnDirection = CounterClockwise;
  }

  //Finds the actual target
  if (turnDirection == Clockwise && target < Inertial.heading(degrees)){
    target += 360;
  }
  else if (turnDirection == CounterClockwise && target > Inertial.heading(degrees)){
    target -= 360;
  }
  
  error = swingError(target, leadSide, direction);

  while ((fabs(error) > tolerance) || fabs(derivative) > exitSpeed){ //Runs while not within tolerance and motors are still spinning quickly (prevents drift)
    derivative = (previousError - error) * 100; //Calculates derivative
    total = error * kp + integral * ki - derivative * kd; //Calculates total output

    if (leadSide == Left){
      if (fabs(total) < minimumSpeed){ //Runs at minimum speed when calculated output is less
        LeftDrive.spin(forward, getSign(error) * minimumSpeed, percent);
        RightDrive.spin(forward, percentage / 100 * getSign(error) * minimumSpeed, percent);
      }
      else if (fabs(total) > 100){
        LeftDrive.spin(forward, getSign(error) * 100, percent);
        RightDrive.spin(forward, getSign(error) * percentage, percent);
      }
      else {
        LeftDrive.spin(forward, total, percent);
        RightDrive.spin(forward, percentage / 100 * total, percent);
      }
    }
    else{
      if (fabs(total) < minimumSpeed){ //Runs at minimum speed when calculated output is less
        LeftDrive.spin(forward, percentage / 100 * getSign(error) * minimumSpeed, percent);
        RightDrive.spin(forward, getSign(error) * minimumSpeed, percent);
      }
      else if (fabs(total > 100)){
       LeftDrive.spin(forward, getSign(error) * percentage, percent);
        RightDrive.spin(forward, getSign(error) * 100, percent);
      }
      else {
        LeftDrive.spin(forward, percentage / 100 * total, percent);
        RightDrive.spin(forward, total, percent);
      }
    }

    if (fabs(error) < maxI){
      integral += error / 100; //Calculates integral
    }
  
    previousError = error; //Makes current error previous error for next loop

    wait(10, msec);

    error = swingError(target, leadSide, direction);
  }

  LeftDrive.stop(brake);
  RightDrive.stop(brake);
}

/********** Tunings **********/

void crawl(Direction direction, double target){ 
  if (direction == Forward){
    driveWithPID(target, 0, 0, 0, 0.5, 15, 0);
  }
  if (direction == Reverse){
    driveWithPID(-target, 0, 0, 0, 0.5, 15, 0);
  }
}

void drive(Direction direction, double target){
  if (direction == Forward){
    driveWithPID(target, 4.5, 3, 0.005, 0.5, 10, 20);
  }
  if (direction == Reverse){
    driveWithPID(-target, 4.5, 3, 0.005, 0.5, 10, 20);
  }
}

void ram(Direction direction, double target){
  if (direction == Forward){
    driveWithPID(target, 0, 0, 0, 2, 70, 0);
  }
  if (direction == Reverse){
    driveWithPID(-target, 0, 0, 0, 2, 70, 0);
  }
}

void turnTo(double target){
  turnWithPID(target, 0.45, 0.3, 0.007, 3, 150, 5, 10);
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

  int columns[] = {5, 6, 5, 2, 4, 3};
  std::string autonNames[] = {"Left-Side Quals", "Left-Side Elims", 
                               "Right-Side Quals", "Right-Side Elims Safe", "Right-Side Six Ball", "Right-Side Mid Rush"};
  Auton autons[] = {AutonLeftQuals, AutonLeftElims, AutonRightQuals, AutonRightElimsSafe, AutonRightElimsSix, AutonRightElimsRush};

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
      if (currentAuton == AutonNone || currentAuton == AutonLeftQuals){
        currentAuton = AutonRightElimsRush;
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
      if (currentAuton == AutonRightElimsRush){
        currentAuton = AutonLeftQuals;
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
  double startTime = Brain.Timer.time(msec);
  double columns[] = {4, 3, 6}; //Makes the text centered on controller
  int temperatures[] = {};
  std::string mechs[] = {"LEFT DRIVE", "RIGHT DRIVE", "INTAKE"};

  while (Brain.Timer.time(msec) - startTime < 1000){
    temperatures[0] = std::max(std::max(LeftFront.temperature(celsius), //Gets highest left drive temperature
                      LeftMiddle.temperature(celsius)), LeftBack.temperature(celsius));
    temperatures[1] = std::max(std::max(RightFront.temperature(celsius), //Gets highest right drive temperature
                      RightMiddle.temperature(celsius)), RightBack.temperature(celsius));
    temperatures[2] = Intake.temperature(celsius);

    wait(100, msec);
  }

  for (int i = 0; i < 3; i++){ //Runs once for each mechanism
    if (temperatures[i] > warningTemp){ //Checks if temperature is exceeding warning satemp
      Controller1.Screen.setCursor(1, columns[i]);
      Controller1.Screen.print(mechs[i].c_str());
      Controller1.Screen.print(" HOT!!!"); 
      Controller1.Screen.setCursor(3, 7);
      Controller1.Screen.print(temperatures[i]);
      Controller1.Screen.print(" degrees");

      Controller1.rumble("..");

      //Example on controller screen:

      //LEFT DRIVE HOT!!!
      
      // 125 degrees

      wait(1.5, sec);
      Controller1.Screen.clearScreen();
    }
  }
}

void preAuton(){
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  Controller1.Screen.clearScreen();

  // tempCheck(0);
  calibrateInertial();
  autonSelector();
}

/********** Auton Function **********/

void autonomous(){
  double startTime = Brain.Timer.time(); //Records start time
  Controller1.Screen.setCursor(2, 6);

  switch (currentAuton){
    case AutonNone: {
      break;
    }
    case AutonLeftQuals: {
      runAutonLeftQuals();
      break;
    }
    case AutonLeftElims: {
      runAutonLeftElims();
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
    case AutonRightElimsSix: {
      runAutonRightElimsSix();
      break;
    }
    case AutonRightElimsRush: {
      runAutonRightElimsRush();
      break;
    }
    default: {
      break;
    }
  }

  Controller1.Screen.print((Brain.Timer.time() - startTime) / 1000); //Records time spent
  Controller1.Screen.print(" Seconds");
}