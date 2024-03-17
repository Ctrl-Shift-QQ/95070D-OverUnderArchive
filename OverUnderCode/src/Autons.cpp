#include "vex.h"
#include "Autonomous.h"
#include <iostream>

/********** Autons **********/

void runAutonLeftAWP(){
  drive(Forward, 7);
  wait(200, msec);
  LeftBackWing.set(true);
  turnTo(270); //Match Load Descored

  LeftBackWing.set(false);
  turnTo(180);
  drive(Forward, 17);
  wait(100, msec);
  turnTo(135);
  wait(200, msec);
  outake(0);
  crawl(Forward, 36); //Elevation Bar Touched

  wait(3, sec);
  Intake.stop(); //Triballs outaked
}

void runAutonLeftNoAWP(){
}

void runAutonLeftSabotage(){
  drive(Forward, 41);
  turnTo(270);
  outake(0.3);
  drive(Reverse, 4);
  turnTo(90);
  wait(100, msec);
  ram(Reverse, 12); //Preload Scored

  turnTo(90);
  wait(100, msec);
  ram(Forward, 10); //Middle of Field Blocked
}

void runAutonRightQuals(){
  double startTime = Brain.Timer.time();

  LeftBackWing.set(true);
  drive(Reverse, 6);
  turnTo(325);
  LeftBackWing.set(false);
  turnTo(0);
  drive(Reverse, 15);
  turnTo(315);
  ram(Reverse, 7); //Match Load Scored

  turnTo(315);
  wait(50, msec);
  drive(Forward, 5);
  turnTo(135);
  outake(0);
  wait(100, msec);
  ram(Forward, 10); //Preload Scored  

  turnTo(135);
  wait(50, msec);
  ram(Reverse, 10);
  turnTo(57.5);
  intake();
  drive(Forward, 48);
  turnTo(190);
  outake(0);
  drive(Forward, 10);
  turnTo(90);
  intake();
  drive(Forward, 21);
  wait(100, msec);
  turnTo(45);
  turnTo(45);
  LeftBackWing.set(true);
  RightBackWing.set(true);
  ram(Reverse, 26); //Middle and Corner Triballs Scored

  turnTo(45);
  drive(Forward, 6);
  LeftBackWing.set(false);
  RightBackWing.set(false);
  turnTo(225);
  wait(100, msec);
  outake(0);
  ram(Forward, 12); //Back Triball Scored
  
  turnTo(225);
  wait(100, msec);
  ram(Reverse, 10); 
  turnTo(160);
  ram(Reverse, 34);
  RightBackWing.set(true);
  turnTo(225);
  ram(Reverse, 8); //Elevation Bar Touched
  
  std::cout << (Brain.Timer.time() - startTime) / 1000 << std::endl;
}

void runAutonRightElimsRush(){
  double startTime = Brain.Timer.time();

  intake();
  swingTo(340, Right, Forward, 87);
  drive(Forward, 10);
  drive(Reverse, 40);
  outake(0);
  turnTo(45);
  std::cout << (Brain.Timer.time() - startTime) / 1000 << std::endl;
}