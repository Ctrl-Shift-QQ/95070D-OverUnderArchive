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

    if (fabs(error) < 30){
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
    driveWithPID(3, 0.3, 0.05, 0.5, 40, target);
  }
  if (direction == "Reverse"){
    driveWithPID(3, 0.3, 0.05, 0.5, 40, -target);
  }
}

static void defaultTurn(double target){
  turnWithPIDTo(1, 0.2, 0.1, 2, 15, target);
}


static void slowDrive(std::string direction, double target){ 
  if (direction == "Forward"){
    driveWithPID(0.5, 0.5, 0.3, 0.5, 35, target);
  }
  if (direction == "Reverse"){
    driveWithPID(0.5, 0.5, 0.3, 0.5, 35, -target);
  }
  
}

static void slowTurn(double target){
  turnWithPIDTo(0.7, 0.1, 0.07, 2, 10, target);
}

static void intake(){
  Intake.spin(forward, 90, percent);
}

static void outake(double waitTime){
  Intake.spin(reverse, 90, percent);
  wait(waitTime, sec);
  Intake.stop();
}

static void frontRam(){
  driveWithPID(2.5, 0.1, 0.2, 3, 60, 10);
}

static void backRam(){
  driveWithPID(2.5, 0.1, 0.2, 3, 60, -10);
}

static void wingsOut (){
  wait(50, msec);
  Wings.set(true);
}

static void wingsIn (){
  wait(50, msec);
  Wings.set(false);
}

void launchTriball(double secondsFireCata){
  double currentTime = Brain.Timer.time(seconds);
  while((Brain.Timer.time(seconds) - currentTime) < secondsFireCata){
    Catapult.spin(forward);
    LeftFront.spin(forward);
    RightFront.spin(forward);
  }
  Catapult.stop();
  LeftFront.stop();
  RightFront.stop();
}

/********** Autons **********/

void runAutonLeftAWP(){
  IntakePiston.set(true);
  slowDrive("Forward", 36);
  slowTurn(45);
  slowDrive("Forward", 10);
  outake(0.5);
  frontRam();
  slowDrive("Reverse", 14);
  slowTurn(0);
  slowDrive("Reverse", 40);
  Wings.set(true);
  slowTurn(45);
  Blocker.set(true);
  slowDrive("Reverse", 50);
}

void runAutonLeftNoAWP(){
}

void runAutonRightAWP(){
}

void runAutonRightSixTB(){
  IntakePiston.set(true);
  intake();
  defaultDrive("Forward", 8);
  slowDrive("Reverse", 56);
  slowTurn(315);
  slowDrive("Reverse", 20);
  Wings.set(true);
  defaultTurn(290); //Match Load Retreived
  
  slowDrive("Reverse", 24);
  Wings.set(false);
  slowTurn(270);  
  backRam(); //Pre Load and Match Load Scored 

  defaultDrive("Forward", 8);
  defaultTurn(90);
  outake(0.3);
  frontRam(); //Below Bar Triball Scored

  defaultDrive("Reverse", 15);
  defaultTurn(25);
  intake();
  defaultDrive("Forward", 80);
  defaultTurn(315);
  defaultDrive("Reverse", 28);
  defaultTurn(180);
  defaultDrive("Forward", 25);
  outake(0.3);
  frontRam(); //Side Triball Scored

  defaultDrive("Reverse", 12);
}

void runAutonSkillsSafe(){
  //Turning is based on the intakes posistion
  slowDrive("Reverse" , 28);
  //launchTriball(3); // Launch all of the tribals to the other side of the field
  defaultTurn(325);
  defaultDrive("Reverse",27); //Score the 2 allance tribals into the goal
  defaultDrive("Forward",30);
  defaultTurn(240);
  defaultDrive("Forward",3);
  launchTriball(5);
}

void runAutonSkillsRiskyLeft(){
  //V2
  /*
  defaultTurn(45);
  defaultTurn(90);
  defaultTurn(135);
  defaultTurn(180);
  defaultTurn(225);
  defaultTurn(270);
  defaultTurn(315);
  defaultTurn(0);
  */
  //Set up the motors
  Catapult.setVelocity(60,percent);  
   //Score the 2 allance tribals into the goal
  defaultDrive("Reverse",30);
  defaultTurn(320);
  defaultDrive("Reverse",27);
  defaultDrive("Forward",24);
  //Launch all of the tribals to the other side of the field
  defaultTurn(245);
  slowDrive("Forward",2);
  launchTriball(2);
  Catapult.stop();
  //Drive to the other side of the field
  defaultTurn(200);
  defaultDrive("Reverse",38);
  defaultTurn(210);
  defaultDrive("Reverse",120);
  // Move to 1st shoving position
  defaultTurn(315);
  wingsOut();
  defaultDrive("Reverse",48);
  defaultTurn(40);
  wingsIn();
  defaultDrive("Reverse",40);
  wingsOut();
  defaultTurn(315);
  defaultDrive("Reverse",30);
  defaultTurn(225);
  //The first push
  defaultDrive("Reverse",40);
  wingsIn();
  defaultDrive("Forward",40);
  //Go to second position
  defaultTurn(315);
  wingsOut();
  defaultDrive("Reverse",30);
  defaultTurn(225);
  //Second Push
  defaultDrive("Reverse",40);
  wingsIn();
  defaultDrive("Forward",40);
  //Go to tried position
  defaultTurn(315);
  defaultDrive("Reverse",50);
  defaultTurn(200);
  //Thrid push
  wingsOut();
  defaultDrive("Reverse",50);
  wingsIn();
  defaultDrive("Forward",20);
  //Test Go to close side push
  defaultTurn(150);
  defaultDrive("Reverse",132);
  //Get ready to push left side
  defaultTurn(225);
  defaultDrive("Reverse",6);
  defaultTurn(270);
  defaultDrive("Reverse",50);
  defaultTurn(315);
  //fourth push 
  defaultDrive("Reverse",30);
  defaultDrive("forward",30);

  //Get ready to push right side
  defaultTurn(45);
  defaultDrive("Reverse",44);
  defaultTurn(320);
  defaultDrive("Reverse",156);
  defaultTurn(220);
  defaultDrive("Reverse",6);
  defaultTurn(175);
  defaultDrive("Reverse",50);
  defaultTurn(135);
  //fifth push 
  defaultDrive("Reverse",30);
  defaultDrive("forward",30);
  //escape
  defaultTurn(45);
  defaultDrive("Reverse",30);
  /*
  fourth push right
  defaultTurn(315);
  defaultDrive("Revrese",36)
  defaultDrive("Reverse",6);
  defaultTurn(170);
  defaultDrive("Reverse",50);
  defaultTurn(135);
  defaultDrive("Reverse",30);
  defaultDrive("forward",30);
  //Get ready to push left
  defaultTurn(45);
  defaultDrive("Reverse",30);
  //Get ready to push left side
  defaultTurn(225);
  defaultDrive("Reverse",6);
  defaultTurn(270);
  defaultDrive("Reverse",50);
  defaultTurn(315);
  //fifth push 
  defaultDrive("Reverse",30);
  defaultDrive("forward",30);
  //escape
  defaultTurn(45);
  defaultDrive("Reverse",30);
  */
  
/*
  Catapult.setVelocity(50,percent);  
   //Score the 2 allance tribals into the goal
  defaultDrive("Reverse" , 30);
  defaultTurn(320);
  defaultDrive("Reverse",27);
  defaultDrive("Forward",24);
  //Launch all of the tribals to the other side of the field
  defaultTurn(245);
  slowDrive("Forward",2);
  //slowRightChange(27);
  Catapult.stop();
  //Drive to the other side of the field
  defaultTurn(200);
  defaultDrive("Reverse",38);
  defaultTurn(210);
  defaultDrive("Reverse",120);
  // Move to 1st shoving position
  defaultTurn(315);
  wingsOut();
  defaultDrive("Reverse",48);
  defaultTurn(40);
  wingsIn();
  defaultDrive("Reverse",40);
  defaultTurn(315);
  defaultDrive("Reverse",30);
  defaultTurn(225);
  //The first push
  wingsOut();
  defaultDrive("Reverse",36);
  wingsIn();
  defaultDrive("Forward",36);
  //Go to second position
  defaultTurn(315);
  wingsOut();
  defaultDrive("Reverse",30);
  defaultTurn(225);
  //Second Push
  defaultDrive("Reverse",36);
  wingsIn();
  defaultDrive("Forward",36);
  //Go to tried position
  defaultTurn(315);
  defaultDrive("Reverse",36);
  defaultTurn(195);
  //Thrid push
  wingsOut();
  defaultDrive("Reverse",45);
  wingsIn();
  defaultDrive("Forward",20);
*/ 
}

void runAutonSkillsRiskyRight(){
  //V2
  /*
  defaultTurn(45);
  defaultTurn(90);
  defaultTurn(135);
  defaultTurn(180);
  defaultTurn(225);
  defaultTurn(270);
  defaultTurn(315);
  defaultTurn(0);
  */
  //Set up the motors
  Catapult.setVelocity(60,percent);  
   //Score the 2 allance tribals into the goal
  defaultDrive("Reverse",30);
  defaultTurn(320);
  defaultDrive("Reverse",27);
  defaultDrive("Forward",24);
  //Launch all of the tribals to the other side of the field
  defaultTurn(245);
  slowDrive("Forward",2);
  launchTriball(2);
  Catapult.stop();
  //Drive to the other side of the field
  defaultTurn(200);
  defaultDrive("Reverse",38);
  defaultTurn(210);
  defaultDrive("Reverse",120);
  // Move to 1st shoving position
  defaultTurn(315);
  wingsOut();
  defaultDrive("Reverse",48);
  defaultTurn(40);
  wingsIn();
  defaultDrive("Reverse",40);
  wingsOut();
  defaultTurn(315);
  defaultDrive("Reverse",30);
  defaultTurn(225);
  //The first push
  defaultDrive("Reverse",40);
  wingsIn();
  defaultDrive("Forward",40);
  //Go to second position
  defaultTurn(315);
  wingsOut();
  defaultDrive("Reverse",30);
  defaultTurn(225);
  //Second Push
  defaultDrive("Reverse",40);
  wingsIn();
  defaultDrive("Forward",40);
  //Go to tried position
  defaultTurn(315);
  defaultDrive("Reverse",50);
  defaultTurn(200);
  //Thrid push
  wingsOut();
  defaultDrive("Reverse",50);
  wingsIn();
  defaultDrive("Forward",20);
  /*
  //Test Go to close side push
  defaultTurn(150);
  defaultDrive("Reverse",132);
  //Get ready to push left side
  defaultTurn(225);
  defaultDrive("Reverse",6);
  defaultTurn(270);
  defaultDrive("Reverse",50);
  defaultTurn(315);
  //fourth push 
  defaultDrive("Reverse",30);
  defaultDrive("forward",30);

  //Get ready to push right side
  defaultTurn(45);
  defaultDrive("Reverse",44);
  defaultTurn(320);
  defaultDrive("Reverse",156);
  defaultTurn(220);
  defaultDrive("Reverse",6);
  defaultTurn(175);
  defaultDrive("Reverse",50);
  defaultTurn(135);
  //fifth push 
  defaultDrive("Reverse",30);
  defaultDrive("forward",30);
  //escape
  defaultTurn(45);
  defaultDrive("Reverse",30);
  */
  
  //fourth push right
  defaultTurn(315);
  defaultDrive("Revrese",36);
  defaultDrive("Reverse",6);
  defaultTurn(170);
  defaultDrive("Reverse",50);
  defaultTurn(135);
  defaultDrive("Reverse",30);
  defaultDrive("forward",30);
  //Get ready to push left
  defaultTurn(45);
  defaultDrive("Reverse",30);
  //Get ready to push left side
  defaultTurn(225);
  defaultDrive("Reverse",6);
  defaultTurn(270);
  defaultDrive("Reverse",50);
  defaultTurn(315);
  //fifth push 
  defaultDrive("Reverse",30);
  defaultDrive("forward",30);
  //escape
  defaultTurn(45);
  defaultDrive("Reverse",30);
  

/*
  Catapult.setVelocity(50,percent);  
   //Score the 2 allance tribals into the goal
  defaultDrive("Reverse" , 30);
  defaultTurn(320);
  defaultDrive("Reverse",27);
  defaultDrive("Forward",24);
  //Launch all of the tribals to the other side of the field
  defaultTurn(245);
  slowDrive("Forward",2);
  //slowRightChange(27);
  Catapult.stop();
  //Drive to the other side of the field
  defaultTurn(200);
  defaultDrive("Reverse",38);
  defaultTurn(210);
  defaultDrive("Reverse",120);
  // Move to 1st shoving position
  defaultTurn(315);
  wingsOut();
  defaultDrive("Reverse",48);
  defaultTurn(40);
  wingsIn();
  defaultDrive("Reverse",40);
  defaultTurn(315);
  defaultDrive("Reverse",30);
  defaultTurn(225);
  //The first push
  wingsOut();
  defaultDrive("Reverse",36);
  wingsIn();
  defaultDrive("Forward",36);
  //Go to second position
  defaultTurn(315);
  wingsOut();
  defaultDrive("Reverse",30);
  defaultTurn(225);
  //Second Push
  defaultDrive("Reverse",36);
  wingsIn();
  defaultDrive("Forward",36);
  //Go to tried position
  defaultTurn(315);
  defaultDrive("Reverse",36);
  defaultTurn(195);
  //Thrid push
  wingsOut();
  defaultDrive("Reverse",45);
  wingsIn();
  defaultDrive("Forward",20);
*/ 
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
      Controller1.Screen.setCursor(3, 2);
      Controller1.Screen.print("Left-Side Safe AWP");
    }
    if (currentAuton == AutonLeftNoAWP){
      Controller1.Screen.setCursor(3, 3);
      Controller1.Screen.print("Left-Side NO AWP");
    }
    if (currentAuton == AutonRightAWP){
      Controller1.Screen.setCursor(3, 5);
      Controller1.Screen.print("Right-Side Safe");
    }
    if (currentAuton == AutonRightSixTB){
      Controller1.Screen.setCursor(3, 3);
      Controller1.Screen.print("Right-Side Six Triball");
    }
    if (currentAuton == AutonSafeSkills){
      Controller1.Screen.setCursor(3, 6);
      Controller1.Screen.print("Safe Skills");
    }
    if (currentAuton == AutonRiskeySkillsLeft){
      Controller1.Screen.setCursor(3, 4);
      Controller1.Screen.print("Riskey Skills Left");
    }
    if (currentAuton == AutonRiskeySkillsRight){
      Controller1.Screen.setCursor(3, 4);
      Controller1.Screen.print("Riskey Skills Right");
    }

    if (Controller1.ButtonLeft.pressing()){
      Controller1.Screen.clearScreen();
      if (currentAuton == AutonNone || currentAuton == AutonLeftAWP){
        currentAuton = AutonRiskeySkillsRight;
      }
      else{
        currentAuton = static_cast<Auton> (static_cast<int> (currentAuton) - 1);
      }
    }
    if (Controller1.ButtonRight.pressing()){
      Controller1.Screen.clearScreen();
      if (currentAuton == AutonRiskeySkillsRight){
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
  Controller1.rumble("-.-.");
}

void calibrateInertial(double seconds){
  Controller1.Screen.setCursor(2, 6);
  Inertial.calibrate();
  Controller1.Screen.print("CALIBRATING!!!");
  wait(seconds, sec);
  Controller1.Screen.clearScreen();
  Inertial.resetHeading();
}

void tempCheck(double warningTemp){
  double leftDriveTemp = std::max(std::max(LeftFront.temperature(fahrenheit), LeftMiddle.temperature(fahrenheit)), LeftBack.temperature(fahrenheit));
  double rightDriveTemp = std::max(std::max(RightFront.temperature(fahrenheit), RightMiddle.temperature(fahrenheit)), RightBack.temperature(fahrenheit));
  double cataTemp = Catapult.temperature(fahrenheit);
  double intakeTemp = Intake.temperature(fahrenheit);

  double temperatures[4] = {leftDriveTemp, rightDriveTemp, cataTemp, intakeTemp};
  std::string mechs[4] = {"LEFT DRIVE", "RIGHT DRIVE", "CATAPULT", "INTAKE"};

  for (int i = 0; i < 4; i++){
    if (temperatures[i] > warningTemp){
      Controller1.Screen.setCursor(1, 6);
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

  calibrateInertial(3);
  tempCheck(120);
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
    case AutonRightAWP: {
      runAutonRightAWP();
      break;
    }
    case AutonRightSixTB: {
      runAutonRightSixTB();
      break;
    }
    case AutonSafeSkills: {
      runAutonSkillsSafe();
      break;
    }
    case AutonRiskeySkillsLeft: {
      runAutonSkillsRiskyLeft();
      break;
    }
    case AutonRiskeySkillsRight: {
      runAutonSkillsRiskyRight();
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
    case AutonRightAWP: {
      runAutonRightAWP();
      break;
    }
    case AutonRightSixTB: {
      runAutonRightSixTB();
      break;
    }
    case AutonSafeSkills: {
      runAutonSkillsSafe();
      break;
    }
    case AutonRiskeySkillsLeft: {
      runAutonSkillsRiskyLeft();
      break;
    }
    case AutonRiskeySkillsRight: {
      runAutonSkillsRiskyRight();
      break;
    }
    default: {
      break;
    }
  }
}