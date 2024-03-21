using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor LeftFront;
extern motor LeftMiddle;
extern motor LeftBack;
extern motor RightFront;
extern motor RightMiddle;
extern motor RightBack;
extern motor_group LeftDrive;
extern motor_group RightDrive;
extern motor LeftIntake;
extern motor RightIntake;
extern motor_group Intake;
extern motor FrontKicker;
extern motor BackKicker;
extern motor_group Kicker;
extern inertial Inertial;
extern digital_out LeftBackWing;
extern digital_out RightBackWing;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );