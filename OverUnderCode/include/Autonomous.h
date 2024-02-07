typedef enum {
    AutonNone = 0,
    AutonLeftAWP,
    AutonLeftNoAWP,
    AutonLeftSabotage,
    AutonRightSafe,
    AutonRightSixTB
} Auton; //Enum for each of the autons

typedef enum {
    Forward = 0,
    Reverse,
    Left,
    Right,
    Clockwise,
    CounterClockwise
} Direction; //Enum for different directions

void calibrateInertial();

void tempCheck(double warningTemp);

//For Testing

void testAuton(Auton testedAuton);

//For Competition

void preAuton();

void autonomous();



