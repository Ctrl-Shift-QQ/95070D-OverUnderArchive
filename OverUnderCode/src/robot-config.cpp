#include "vex.h"


using namespace vex;
using signature = vision::signature;
using code = vision::code;


// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;


// VEXcode device constructors

controller Controller1 = controller(primary);
motor LeftFront = motor(PORT7, ratio6_1, true);
motor LeftMiddle = motor(PORT6, ratio6_1, true);
motor LeftBack = motor(PORT5, ratio6_1, true);
motor RightFront = motor(PORT2, ratio6_1, false);
motor RightMiddle = motor(PORT3, ratio6_1, false);
motor RightBack = motor(PORT4, ratio6_1, false);
motor_group LeftDrive(LeftFront, LeftMiddle, LeftBack);
motor_group RightDrive(RightFront, RightMiddle, RightBack);
motor Intake = motor(PORT1, ratio6_1, false);
motor Kicker = motor(PORT8, ratio36_1, false);
inertial Inertial = inertial(PORT21);
digital_out LeftWing = digital_out(Brain.ThreeWirePort.A);
digital_out RightWing = digital_out(Brain.ThreeWirePort.B);

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
