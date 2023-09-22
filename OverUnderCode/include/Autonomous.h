
//For Testing
void calibrate(double seconds);

void tempCheck(double warningTemp);

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



