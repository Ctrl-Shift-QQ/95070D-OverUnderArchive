#include "vex.h"
#include "Autonomous.h"
#include <iostream>

/********** Autons **********/

void runProgrammingSkills(){
  drive(Reverse, 16);
  swingTo(315, Right, Reverse);
  ram(Reverse, 12);
}