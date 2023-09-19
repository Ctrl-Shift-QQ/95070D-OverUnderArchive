#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor LeftFront = motor(PORT16, ratio6_1, true);
motor LeftBack = motor(PORT19, ratio6_1, true);
motor LeftStack = motor(PORT18, ratio6_1, false);
motor RightFront = motor(PORT1, ratio6_1, false);
motor RightBack = motor(PORT14, ratio6_1, false);
motor RightStack = motor(PORT13, ratio6_1, true);
motor Intake = motor(PORT17, ratio6_1, true);
motor Catapult = motor(PORT2, ratio36_1, false);
inertial Inertial = inertial(PORT11);
digital_out Wings = digital_out(Brain.ThreeWirePort.B);
limit LimitSwitch = limit(Brain.ThreeWirePort.C);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}