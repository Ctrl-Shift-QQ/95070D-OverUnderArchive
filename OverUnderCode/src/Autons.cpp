#include "vex.h"
#include "Autonomous.h"
#include <iostream>

/********** Autons **********/

void runProgrammingSkills(){
  double currentTime = Brain.Timer.time();

  drive(Reverse, 10);
  wait(200, msec);
  turnTo(57.5);
  wait(100, msec);
  crawl(Reverse, 1);
  // Kicker.setVelocity(40, percent); 
  // Kicker.spin(forward);
  // wait(25, sec);
  // Kicker.stop(); //Kicker Fired

  drive(Forward, 6);
  wait(100, msec);
  swingTo(315, Left, Reverse, 5);
  wait(100, msec);
  ram(Reverse, 10); //Preloads Scored

  wait(100, msec);
  turnTo(315);
  wait(100, msec);
  drive(Forward, 6);
  turnTo(0);
  drive(Forward, 36);
  wait(100, msec);
  turnTo(225);
  wait(200, msec);
  turnTo(225);
  wait(100, msec);
  drive(Reverse, 72);
  LeftWing.set(true);
  swingTo(315, Right, Reverse, 40);
  ram(Reverse, 10);
  turnTo(325);
  wait(100, msec);
  drive(Forward, 10);
  LeftWing.set(false);
  wait(200, msec);
  turnTo(315);
  wait(100, msec);
  ram(Reverse, 14);
  turnTo(315); //Side Pushed

  drive(Forward, 2);
  turnTo(45);
  drive(Reverse, 12);
  swingWithPID(225, Left, Reverse, 15, 0.5, 0.1, 0.0003, 2, 10, 10);
  ram(Reverse, 8); //Left Front Pushed

  drive(Forward, 15);
  turnTo(315);
  drive(Reverse, 10);
  RightWing.set(true);
  turnTo(225);
  ram(Reverse, 18); //Left Middle Front Pushed

  wait(100, msec);
  drive(Forward, 15);
  RightWing.set(false);
  turnTo(315);
  drive(Reverse, 10);
  RightWing.set(true); 
  turnTo(225);
  ram(Reverse, 18); //Right Middle Front Pushed

  wait(100, msec);
  drive(Forward, 15);
  turnTo(315);
  drive(Reverse, 10);
  RightWing.set(true); //Right Middle Front Pushed

  std::cout << (Brain.Timer.time() - currentTime) / 1000 << std::endl;
}