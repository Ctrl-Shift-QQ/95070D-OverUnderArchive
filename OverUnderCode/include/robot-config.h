using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor LeftFront;
extern motor LeftBack;
extern motor LeftStack;
extern motor RightFront;
extern motor RightBack;
extern motor RightStack;
extern controller Controller1;
extern inertial Inertial;
extern motor Intake;
extern digital_out DigitalOutA;
extern motor Catapult;
extern limit LimitSwitch;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );