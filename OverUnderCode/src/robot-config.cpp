#include "vex.h"


using namespace vex;
using signature = vision::signature;
using code = vision::code;


// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;


// VEXcode device constructors

controller Controller1 = controller(primary);
motor LeftFront = motor(PORT17, ratio6_1, false);
motor LeftBack = motor(PORT18, ratio6_1, false);
motor LeftStack = motor(PORT16, ratio6_1, true);
motor RightFront = motor(PORT14, ratio6_1, true);
motor RightBack = motor(PORT13, ratio6_1, true);
motor RightStack = motor(PORT15, ratio6_1, false);
motor Intake = motor(PORT11, ratio6_1, false);
motor Kicker = motor(PORT9, ratio36_1, false);
inertial Inertial = inertial(PORT4);
digital_out Wings = digital_out(Brain.ThreeWirePort.H);
digital_out Blocker = digital_out(Brain.ThreeWirePort.F);

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
