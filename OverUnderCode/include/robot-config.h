using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor LeftFront;
extern motor LeftBack;
extern motor LeftStack;
extern motor RightFront;
extern motor RightBack;
extern motor RightStack;
extern motor Intake;
extern motor Catapult;
extern inertial Inertial;
extern digital_out Wings;
extern limit LimitSwitch;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );