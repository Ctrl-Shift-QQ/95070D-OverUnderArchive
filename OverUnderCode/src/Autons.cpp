#include "vex.h"
#include "Autonomous.h"
#include <iostream>

/********** Autons **********/

void runAutonLeftAWP(){
  drive(Forward, 7);
  LeftWing.set(true);
  turnTo(320); //Match Load Descored

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
  wait(200, msec);
  ram(Reverse, 9); //Preload Scored

  wait(200, msec);
  turnTo(225);
  drive(Forward, 9);
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
  drive(Forward, 38);
  turnTo(270);
  RightWing.set(true);
  // wait(0.25, sec);
  // ram(Reverse, 30); //Middle and Back Triballs Popped Over

  // wait(0.5, sec);
  // RightWing.set(false);
  // turnTo(270); 
  // drive(Forward, 20);
  // outake(0.5);
  // turnTo(90);
  // Intake.stop();
  // RightWing.set(true);
  // ram(Reverse, 10); //Preload Scored

  // turnTo(90);
  // drive(Forward, 15); 
  // RightWing.set(false); //Middle of Field Blocked
}

void runAutonRightQuals(){
  LeftWing.set(true);
  drive(Reverse, 6);
  turnTo(325);
  LeftWing.set(false);
  turnTo(0);
  drive(Reverse, 15);
  turnTo(315);
  ram(Reverse, 6); //Match Load Scored

  turnTo(315);
  drive(Forward, 8);
  turnTo(135);
  outake(0);
  ram(Forward, 11); //Preload Scored  

  turnTo(135);
  ram(Reverse, 8);
  turnTo(65);
  intake();
  drive(Forward, 48);
  turnTo(190);
  drive(Forward, 8);
  Intake.setVelocity(60, percent);
  Intake.spin(reverse);
  wait(0.25, sec);
  turnTo(100);
  intake();
  drive(Forward, 16);
  wait(100, msec);
  turnTo(45);
  LeftWing.set(true);
  RightWing.set(true);
  ram(Reverse, 25); //Middle and Corner Triballs Scored

  drive(Forward, 5);
  LeftWing.set(false);
  RightWing.set(false);
  turnTo(225);
  outake(0);
  ram(Forward, 12); //Back Triball Scored
  
  turnTo(225);
  ram(Reverse, 10); 
  turnTo(160);
  ram(Reverse, 36);
  RightWing.set(true);
  turnTo(225);
  ram(Reverse, 6); //Elevation Bar Touched
}

void runAutonRightElimsSafe(){ 
  double a = Brain.Timer.time();
  intake();
  drive(Forward, 2);
  wait(100, msec);
  drive(Reverse, 32);
  crawl(Reverse, 3);
  turnTo(315);
  drive(Reverse, 10);
  wait(100, msec);
  LeftWing.set(true);
  wait(200, msec);
  turnTo(290); //Match Load Descored

  LeftWing.set(false);
  turnTo(315);
  drive(Reverse, 15);
  turnTo(270);
  RightWing.set(true);
  ram(Reverse, 10); //Preload and Match Load Scored

  RightWing.set(false);
  turnTo(270);
  drive(Forward, 8);
  turnTo(90);
  outake(0);
  ram(Forward, 11); //Below Bar Triball Scored

  drive(Reverse, 2);
  swingTo(0, Right, Reverse);
  intake();
  drive(Forward, 28);
  swingTo(240, Left, Reverse);
  turnTo(180);
  outake(0);
  ram(Forward, 18); //Side Triball Scored

  turnTo(180);
  drive(Reverse, 14); //Backed Out
  std::cout << (Brain.Timer.time() - a) / 1000 << std::endl;
}

void runAutonRightElimsRisky(){
  double a = Brain.Timer.time();
  intake();
  drive(Forward, 2);
  wait(100, msec);
  drive(Reverse, 32);
  crawl(Reverse, 3);
  turnTo(315);
  drive(Reverse, 10);
  wait(100, msec);
  LeftWing.set(true);
  wait(200, msec);
  turnTo(290); //Match Load Descored

  LeftWing.set(false);
  turnTo(315);
  drive(Reverse, 15);
  turnTo(270);
  RightWing.set(true);
  ram(Reverse, 10); //Preload and Match Load Scored

  RightWing.set(false);
  turnTo(270);
  drive(Forward, 8);
  turnTo(90);
  outake(0);
  ram(Forward, 11); //Below Bar Triball Scored

  drive(Reverse, 2);
  swingTo(0, Right, Reverse);
  intake();
  drive(Forward, 28);
  swingTo(220, Left, Reverse);
  turnTo(180);
  outake(0);
  ram(Forward, 18); //Side Triball Scored

  turnTo(180);
  drive(Reverse, 16);
  turnTo(45);
  intake();
  drive(Forward, 16);
  turnTo(0);
  LeftWing.set(true);
  RightWing.set(true);
  ram(Reverse, 24); //Middle Triball Scored

  drive(Forward, 8);
  LeftWing.set(false);
  RightWing.set(false); 
  turnTo(180);
  outake(0);
  ram(Forward, 11); //Back Triball Scored
}