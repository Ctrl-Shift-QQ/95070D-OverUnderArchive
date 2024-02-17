#include "vex.h"
#include "Autonomous.h"
#include <iostream>

/********** Autons **********/

void runProgrammingSkills(){
  drive(Reverse, 16);
  turnTo(250);
  drive(Forward, 2);
  Kicker.setVelocity(57, percent); 
  Kicker.spin(forward);
  wait(25, sec);
  Kicker.stop(); //Kicker Fired

  drive(Reverse, 4);
  turnTo(0);
  drive(Reverse, 8);
  swingTo(315, Right, Reverse);
  ram(Reverse, 12); //Preloads Scored

  
}