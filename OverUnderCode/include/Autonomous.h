
//For Testing

typedef enum {
    AutonNone = 0,
    AutonLeftAWP,
    AutonLeftNoAWP,
    AutonLeftSabotage,
    AutonRightSafe,
    AutonRightFiveTB
} Auton;

void testAuton(Auton testedAuton);

//For Competition

void calibrateInertial();

void tempCheck(double warningTemp);

void preAuton();

void autonomous();



