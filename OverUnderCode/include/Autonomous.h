
//For Testing

typedef enum {
    AutonNone = 0,
    AutonLeftAWP,
    AutonLeftNoAWP,
    AutonRightAWP,
    AutonRightSixTB
} Auton;

void testAuton(Auton testedAuton);

//For Competition

void calibrateInertial(double seconds);

void tempCheck(double warningTemp);

void preAuton();

void autonomous();



