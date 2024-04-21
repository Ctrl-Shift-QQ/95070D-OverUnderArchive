/********** Enums for Auton **********/

typedef enum {
    AutonNone = 0,
    AutonLeftQuals,
    AutonLeftElims,
    AutonRightQuals,
    AutonRightElimsSafe,
    AutonRightElimsSix,
    AutonRightElimsRush
} Auton; //Enum for each of the autons

typedef enum {
    Forward = 0,
    Reverse,
    Left,
    Right,
    Clockwise,
    CounterClockwise
} Direction; //Enum for different directions

/********** Auton Functions **********/

void crawl(Direction direction, double target);

void drive(Direction direction, double target);

void ram(Direction direction, double target);

void turnTo(double target);

void driveWithPID(double target, double kp, double ki, double kd, double tolerance, double minimumSpeed, double maxI);

void turnWithPID(double target, double kp, double ki, double kd, double tolerance, double exitSpeed, double minimumSpeed, double maxI);

void swingWithPID(double target, Direction leadSide, Direction direction, double percentage, double kp, double ki, double kd, double tolerance, double exitSpeed, double minimumSpeed, double maxI);

void intake();

void outake(double waitTime);

/********** Autons **********/

void runAutonLeftQuals();

void runAutonLeftElims();

void runAutonRightQuals();

void runAutonRightElimsSafe();

void runAutonRightElimsSix();

void runAutonRightElimsRush();

/********** Run In Main **********/

void preAuton();

void autonomous();



