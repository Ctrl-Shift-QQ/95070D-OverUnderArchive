
//For Testing
void calibrate(double seconds);

void tempCheck(double warningTemp);

typedef enum {
    AutonNone = 0,
    AutonLeftAWP,
    AutonLeftSabotage,
    AutonRightSafe,
    AutonRightFourTB,
} Auton;

void testAuton(Auton testedAuton);

//For Competition
void preAuton();

void autonomous();



