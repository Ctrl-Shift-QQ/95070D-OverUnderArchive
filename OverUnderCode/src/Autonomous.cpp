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

static void driveWithPID(double target, double kp, double ki, double kd, double correctiveFactor, double tolerance, double minimumSpeed, double maxI){ //Drives straight
  double initialOrientation = Inertial.heading(degrees);
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
      LeftDrive.spin(forward, getSign(error) * minimumSpeed + (Inertial.heading(degrees) - initialOrientation) * correctiveFactor, percent); //If orientation is off, drivetrain speeds change accordingly
      RightDrive.spin(forward, getSign(error) * minimumSpeed - (Inertial.heading(degrees) - initialOrientation) * correctiveFactor, percent);
    }
    else {
      LeftDrive.spin(forward, total + (Inertial.heading(degrees) - initialOrientation) * correctiveFactor, percent);
      RightDrive.spin(forward, total - (Inertial.heading(degrees) - initialOrientation) * correctiveFactor, percent);
    }

    if(fabs(error) < maxI){
      integral += error / 50; //Calculates integral
    }

    wait(20, msec);

    error = target - (LeftDrive.position(turns) + RightFront.position(turns)) / 2 * 3.25 * M_PI * 3/4; //Calculates error in inches
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

static void turnWithPID(double target, double kp, double ki, double kd, double exitSpeed, double tolerance, double minimumSpeed, double maxI){ //Turns in place
  double error = turnError(target);
  double integral = 0;
  double derivative;
  double previousError = error;
  double total;

  if (fabs(target - Inertial.heading(degrees)) < 180){ //Calculates the error for fastest path to the desired heading
    error = target - Inertial.heading(degrees);
  }
  else {
    error = (360 - fabs(target - Inertial.heading(degrees)));
    if (target > Inertial.heading(degrees)){
      error = -error;
    }
  }
  
  while ((fabs(error) > tolerance) || (LeftDrive.velocity(percent) + RightDrive.velocity(percent)) / 2  > exitSpeed){ //Runs while not within tolerance and motors are still spinning quickly (prevents drift)
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

static void swingWithPID(double target, Direction side, Direction direction, double percentage, double kp, double ki, double kd, double tolerance, double minimumSpeed, double maxI){ //Turns one side of drivetrain
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

    error = swingError(target, side, direction);
  }

  LeftDrive.stop(brake);
  RightDrive.stop(brake);
}

/********** Tunings **********/

void crawl(Direction direction, double target){ 
  if (direction == Forward){
    driveWithPID(target, 0, 0, 0, 0, 0.5, 15, 0);
  }
  if (direction == Reverse){
    driveWithPID(-target, 0, 0, 0, 0, 0.5, 15, 0);
  }
}

void drive(Direction direction, double target){
  if (direction == Forward){
    driveWithPID(target, 4.5, 0.1, 0.005, 0.3, 0.5, 15, 10);
  }
  if (direction == Reverse){
    driveWithPID(-target, 4.5, 0.1, 0.005, 0.3, 0.5, 15, 10);
  }
}

void ram(Direction direction, double target){
  if (direction == Forward){
    driveWithPID(target, 0, 0, 0, 0, 2, 70, 0);
  }
  if (direction == Reverse){
    driveWithPID(-target, 0, 0, 0, 0, 2, 70, 0);
  }
}

void turnTo(double target){
  turnWithPID(target, 0.45, 0, 0.0008, 1, 2, 7, 20);
}

void swingTo(double target, Direction side, Direction direction, double percentage){
  swingWithPID(target, side, direction, percentage, 0, 0, 0, 0, 0, 0);
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

  int columns[] = {2, 3, 3, 5, 2};
  std::string autonNames[] = {"Left-Side Safe AWP", "Left-Side NO AWP", "Left-Side Sabotage", 
                               "Right-Side Quals", "Right-Side Goal Rush"};
  Auton autons[] = {AutonLeftAWP, AutonLeftNoAWP, AutonLeftSabotage, AutonRightQuals, AutonRightElimsRush};

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
  double startTime = Brain.Timer.time(msec);
  double columns[] = {4, 3, 6, 6}; //Makes the text centered on controller
  double temperatures[] = {};
  std::string mechs[] = {"LEFT DRIVE", "RIGHT DRIVE", "KICKER", "INTAKE"};

  while (Brain.Timer.time(msec) - startTime < 1000){
    temperatures[0] = std::max(std::max(LeftFront.temperature(fahrenheit), //Gets highest left drive temperature
                      LeftMiddle.temperature(fahrenheit)), LeftBack.temperature(fahrenheit));
    temperatures[1] = std::max(std::max(RightFront.temperature(fahrenheit), //Gets highest right drive temperature
                      RightMiddle.temperature(fahrenheit)), RightBack.temperature(fahrenheit));
    temperatures[2] = Kicker.temperature(fahrenheit);
    temperatures[3] = Kicker.temperature(fahrenheit);
  }

  for (int i = 0; i < 4; i++){ //Runs once for each mechanism
    if (temperatures[i] > warningTemp){ //Checks if temperature is exceeding warning satemp
      Controller1.Screen.setCursor(1, columns[i]);
      Controller1.Screen.print(mechs[i].c_str());
      Controller1.Screen.print(" HOT!!!"); 
      Controller1.Screen.setCursor(3, 10);
      Controller1.Screen.print(temperatures[i]);
      Controller1.rumble("..");

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

  calibrateInertial();
  autonSelector();
  tempCheck(120);
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
    case AutonRightElimsRush: {
      runAutonRightElimsRush();
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
    case AutonRightQuals: {
      runAutonRightQuals();
      break;
    }
    case AutonRightElimsRush: {
      runAutonRightElimsRush();
    }
    default: {
      break;
    }
  }

  Controller1.Screen.print((Brain.Timer.time() - startTime) / 1000); //Records time spent
  Controller1.Screen.print(" Seconds");
}