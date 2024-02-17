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
  ram(Reverse, 12); //Preload Scored

  wait(200, msec);
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
  drive(Forward, 39);
  turnTo(270);
  RightWing.set(true);
  wait(1, sec);
  ram(Reverse, 30); //Middle and Back Triballs Popped Over

  wait(0.5, sec);
  RightWing.set(false);
  turnTo(270); 
  drive(Forward, 20);
  outake(0.5);
  turnTo(90);
  Intake.stop();
  ram(Reverse, 10); //Preload Scored

  turnTo(90);
  drive(Forward, 15); //Middle of Field Blocked
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
  turnTo(225);
  RightWing.set(true);
  ram(Reverse, 6); //Elevation Bar Touched
}

void runAutonRightElims(){ 
  intake();
  drive(Forward, 48);
  drive(Reverse, 5);
  turnTo(100);
  wait(100, msec);
  Inertial.resetHeading(); //Makes heading easier to measure
  outake(0);
  ram(Forward, 16); //Middle Triball Scored;

  drive(Reverse, 5);
  turnTo(155);
  intake();
  drive(Forward, 24);
  turnTo(40);
  drive(Forward, 40);
  outake(0.5);
  swingTo(180, Left, Forward);
}