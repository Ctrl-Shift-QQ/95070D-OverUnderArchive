#include "vex.h"
#include "Autonomous.h"
#include <iostream>

/********** Autons **********/

void runProgrammingSkills(){
  double currentTime = Brain.Timer.time();

  drive(Reverse, 8);
  wait(200, msec);
  turnTo(67.5);
  wait(100, msec);
  crawl(Reverse, 1);
  // Kicker.setVelocity(40, percent); 
  // Kicker.spin(forward);
  // wait(44, sec);
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
  drive(Forward, 35);
  wait(100, msec);
  turnTo(225);
  wait(200, msec);
  turnTo(225);
  wait(100, msec);
  drive(Reverse, 70);
  LeftWing.set(true);
  swingTo(315, Right, Reverse, 45);
  drive(Reverse, 5); 
  turnTo(315);
  drive(Forward, 10);
  ram(Reverse, 12); //Left Side Pushed

  turnTo(315); 
  drive(Forward, 3);
  turnTo(45);
  wait(100, msec);
  drive(Reverse, 18);
  wait(100, msec);
  RightWing.set(true);
  swingWithPID(265, Left, Reverse, 15, 0.3, 0.1, 0.0002, 2, 15, 10);
  drive(Reverse, 4);
  drive(Forward, 6);
  LeftWing.set(false);
  RightWing.set(false);
  ram(Reverse, 30); //Left Front Pushed

  turnTo(225);
  drive(Forward, 26);
  turnTo(135); 
  drive(Forward, 36);
  turnTo(190);
  LeftWing.set(true);
  RightWing.set(true);
  drive(Reverse, 32); //Right Front Pushed

  turnTo(225);
  drive(Forward, 12);
  turnTo(135);
  drive(Reverse, 4);
  turnTo(225);
  ram(Reverse, 15); //Middle Front Pushed

  drive(Forward, 5); //Backed Out

  std::cout << (Brain.Timer.time() - currentTime) / 1000 << std::endl;
}