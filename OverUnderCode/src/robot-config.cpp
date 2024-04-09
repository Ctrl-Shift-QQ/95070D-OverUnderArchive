#include "vex.h"


using namespace vex;
using signature = vision::signature;
using code = vision::code;


// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;


// VEXcode device constructors

controller Controller1 = controller(primary);
motor LeftFront = motor(PORT18, ratio6_1, true);
motor LeftMiddle = motor(PORT10, ratio6_1, true);
motor LeftBack = motor(PORT19, ratio6_1, true);
motor RightFront = motor(PORT14, ratio6_1, false);
motor RightMiddle = motor(PORT17, ratio6_1, false);
motor RightBack = motor(PORT1, ratio6_1, false);
motor_group LeftDrive(LeftFront, LeftMiddle, LeftBack);
motor_group RightDrive(RightFront, RightMiddle, RightBack);
motor LeftIntake = motor(PORT21, ratio6_1, true);
motor RightIntake = motor(PORT12, ratio6_1, false);
motor_group Intake(LeftIntake, RightIntake);
inertial Inertial = inertial(PORT16);
digital_out Hang = digital_out(Brain.ThreeWirePort.D);
digital_out LeftBackWing = digital_out(Brain.ThreeWirePort.A);
digital_out RightBackWing = digital_out(Brain.ThreeWirePort.B);
digital_out LeftFrontWing = digital_out(Brain.ThreeWirePort.H);
digital_out RightFrontWing = digital_out(Brain.ThreeWirePort.G);

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
