typedef enum {
    AutonNone = 0,
    AutonLeftAWP,
    AutonLeftNoAWP,
    AutonLeftSabotage,
    AutonRightSafe,
    AutonRightSixTB
} Auton;

typedef enum {
    Forward = 0,
    Reverse,
    Clockwise,
    CounterClockwise
} Direction;

void calibrateInertial();

void tempCheck(double warningTemp);

//For Testing

void testAuton(Auton testedAuton);

//For Competition

void preAuton();

void autonomous();



