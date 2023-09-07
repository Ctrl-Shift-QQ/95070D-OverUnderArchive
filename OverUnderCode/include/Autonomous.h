
//For testing
typedef enum {
    AutonNone = 0,
    AutonLeftSafe,
    AutonLeftRisky,
    AutonRightSafe,
    AutonRightRisky,
} Auton;

void runAuton(Auton testedAuton);

//For Competition
void preAuton();
void autonomous();




