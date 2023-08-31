#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor LeftBack = motor(PORT3, ratio18_1, false);
motor LeftFront = motor(PORT1, ratio18_1, false);
motor LeftStack = motor(PORT2, ratio18_1, true);
motor RightFront = motor(PORT8, ratio18_1, true);
motor RightBack = motor(PORT10, ratio18_1, true);
motor RightStack = motor(PORT9, ratio18_1, false);
controller Controller1 = controller(primary);
inertial Inertial = inertial(PORT20);
motor Intake = motor(PORT4, ratio18_1, true);

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