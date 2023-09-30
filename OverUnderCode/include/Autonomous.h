
//For Testing
void calibrate(double seconds);

void tempCheck(double warningTemp);

void autonSelector();

typedef enum {
    AutonNone = 0,
    AutonLeftAWP,
    AutonLeftSabotage,
    AutonRightSafe,
    AutonRightFourTB,
} Auton;

Auton currentAuton = AutonNone;

void testAuton(Auton testedAuton);

//For Competition
void preAuton();

void autonomous();



