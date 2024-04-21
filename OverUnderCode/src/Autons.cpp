#include "vex.h"
#include "Autonomous.h"
#include <iostream>

/********** Autons **********/

void runAutonLeftQuals(){
  drive(Forward, 6);
  wait(200, msec);
  LeftBackWing.set(true);
  turnTo(270); //Match Load Descored

  LeftBackWing.set(false);
  turnTo(180);
  drive(Forward, 17);
  wait(0.1, sec);
  turnTo(135);
  wait(200, msec);
  outake(0);
  crawl(Forward, 38); //Elevation Bar Touched

  wait(3, sec);
  Intake.stop(); //Triballs Outaked
}

void runAutonLeftElims(){
  Inertial.setHeading(11.4, degrees);
  intake();
  drive(Forward, 48);
  wait(0.1, sec);
  drive(Reverse, 12);
  wait(0.25, sec);
  drive(Forward, 5);
  turnTo(270);
  turnTo(270);
  wait(0.1, sec);
  drive(Reverse, 10);
  RightFrontWing.set(true);
  wait(0.1, sec);
  turnTo(60);
  RightFrontWing.set(false);
  turnTo(170);
  RightFrontWing.set(true);
  outake(0);
  drive(Forward, 34); //Middle and Back Triballs Pushed to Alley

  turnTo(180);
  wait(0.25, sec);
  RightFrontWing.set(false);
  drive(Reverse, 10);
  turnTo(180);
  swingWithPID(60, Left, Reverse, 0, 0, 0, 0, 2, 150, 40, 10);
  wait(200, msec);
  drive(Reverse, 40);
  turnTo(315);
  LeftBackWing.set(true);
  crawl(Reverse, 7); 
  turnTo(300); //Match Load Descored

  wait(0.25, sec);
  LeftBackWing.set(false);
  turnTo(135);
  LeftFrontWing.set(true);
  swingWithPID(90, Right, Forward, 65, 0.5, 0.1, 0.003, 2, 150, 20, 10);
  driveWithPID(20, 0, 0, 0, 0.5, 25, 0); //Elevation Bar Touched

  LeftFrontWing.set(false);
  wait(3, sec);
  Intake.stop(); //Triballs Outaked
}

void runAutonRightQuals(){
  intake();
  drive(Reverse, 2);
  LeftBackWing.set(true);
  drive(Reverse, 6);
  turnTo(310);
  LeftBackWing.set(false);
  turnTo(0);
  drive(Reverse, 13);
  turnTo(315);
  wait(0.1, sec);
  ram(Reverse, 10); //Match Load Scored

  turnTo(315);
  drive(Forward, 8);
  wait(0.1, sec);
  turnTo(135);
  wait(200, msec);
  outake(0);
  ram(Forward, 16); //Preload Scored  

  turnTo(135);
  wait(0.1, sec);
  drive(Reverse, 16);
  turnTo(60);
  intake();
  drive(Forward, 51);
  turnTo(205);
  Intake.spin(reverse, 60, percent);
  drive(Forward, 10);
  turnTo(82.5);
  intake();
  drive(Forward, 25);
  wait(0.1, sec);
  turnTo(225);
  turnTo(225);
  wait(0.1, sec);
  LeftFrontWing.set(true);
  RightFrontWing.set(true);
  outake(0);
  ram(Forward, 32); //Middle, Back, and Corner Triballs Scored

  Intake.stop();
  drive(Reverse, 8);
  LeftFrontWing.set(false);
  RightFrontWing.set(false);
  turnTo(150);
  ram(Reverse, 34);
  turnTo(225);
  RightBackWing.set(true);
  ram(Reverse, 10); //Elevation Bar Touched
}

void runAutonRightElimsSafe(){
}

void runAutonRightElimsSix(){
  intake();
  drive(Forward, 2);
  wait(0.1, sec);
  driveWithPID(-34, 0, 0, 0, 0.5, 45, 0);
  turnTo(315);
  drive(Reverse, 8);
  LeftBackWing.set(true);
  drive(Reverse, 6);
  turnTo(265);
  LeftBackWing.set(false);
  turnTo(315);
  drive(Reverse, 13);
  turnTo(270);
  wait(0.1, sec);
  ram(Reverse, 8); //Match Load and Pre Load Scored

  wait(0.1, sec);
  turnTo(270);
  drive(Forward, 8);
  wait(0.1, sec);
  turnTo(90);
  wait(200, msec);
  outake(0);
  ram(Forward, 14); //Preload Scored

  turnTo(90);
  drive(Reverse, 16);
  turnTo(15);
  intake();
  drive(Forward, 51);
  turnTo(160);
  Intake.spin(reverse, 60, percent);
  drive(Forward, 10);
  turnTo(37.5);
  intake();
  drive(Forward, 25);
  turnTo(180);
  wait(0.1, sec);
  LeftFrontWing.set(true);
  RightFrontWing.set(true);
  outake(0);
  ram(Forward, 32); //Middle, Back, and Corner Triballs Scored

  turnTo(180);
  drive(Reverse, 8);
  LeftFrontWing.set(false);
  RightFrontWing.set(false); //Backed Out of Goal
}

void runAutonRightElimsRush(){
  Inertial.setHeading(348.6, degrees);
  intake();
  RightFrontWing.set(true);
  drive(Forward, 48);
  RightFrontWing.set(false);
  drive(Reverse, 53);
  outake(0);
  turnTo(80);
  turnTo(270);
  intake();
  drive(Forward, 32);
}