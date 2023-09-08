
//For testing
typedef enum {
    AutonNone = 0,
    AutonLeftSafe,
    AutonLeftRisky,
    AutonRightSafe,
    AutonRightRisky,
} Auton;

void testAuton(Auton testedAuton);

//For Competition
void preAuton();
void autonomous();




