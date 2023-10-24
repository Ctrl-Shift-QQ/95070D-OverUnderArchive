
//For Testing
void calibrateInertial(double seconds);

void tempCheck(double warningTemp);

typedef enum {
    AutonNone = 0,
    AutonLeftAWP,
    AutonLeftNoAWP,
    AutonRightSafe,
    AutonRightSixTB
} Auton;

void testAuton(Auton testedAuton);

//For Competition
void preAuton();

void autonomous();



