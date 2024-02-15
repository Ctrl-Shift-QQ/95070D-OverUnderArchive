#include "vex.h"
#include "Autonomous.h"
#include <iostream>

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
  crawl(Forward, 28);
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
  drive(Forward, 42);
  turnTo(270);
  RightWing.set(true);
  wait(1, sec);
  ram(Reverse, 30);
  turnTo(270); //Middle and Back Triballs Popped Over

  drive(Forward, 10);
}

void runAutonRightSafe(){
  LeftWing.set(true);
  drive(Reverse, 6);
  turnTo(330);
  LeftWing.set(false);
  turnTo(0);
  drive(Reverse, 13);
  turnTo(315);
  ram(Reverse, 7); //Match Load Scored

  turnTo(315);
  drive(Forward, 8);
  turnTo(135);
  outake(0);
  ram(Forward, 12); //Preload Scored  

  turnTo(135);
  ram(Reverse, 8);
  turnTo(67.5);
  intake();
  drive(Forward, 48);
  turnTo(190);
  drive(Forward, 8);
  Intake.setVelocity(60, percent);
  Intake.spin(reverse);
  wait(0.5, sec);
  turnTo(85);
  intake();
  drive(Forward, 16);
  wait(100, msec);
  turnTo(45);
  LeftWing.set(true);
  RightWing.set(true);
  ram(Reverse, 21); //Middle and Corner Triballs Scored

  drive(Forward, 6);
  LeftWing.set(false);
  RightWing.set(false);
  turnTo(225);
  outake(0);
  ram(Forward, 10);
  turnTo(225);
  ram(Reverse, 10); //Back Triball Scored
}

void runAutonRightSixTB(){ 
}