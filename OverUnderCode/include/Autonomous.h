/********** Enums for Auton **********/

typedef enum {
    AutonNone = 0,
    AutonLeftAWP,
    AutonLeftNoAWP,
    AutonLeftSabotage,
    AutonRightQuals,
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

/********** Tuned Auton Functions **********/

void crawl(Direction direction, double target);

void drive(Direction direction, double target);

void ram(Direction direction, double target);

void turnTo(double target);

void swingTo(double target, Direction side, Direction direction, double percentage);

void intake();

void outake(double waitTime);

/********** Autons **********/

void runAutonLeftAWP();

void runAutonLeftNoAWP();

void runAutonLeftSabotage();

void runAutonRightQuals();

void runAutonRightElimsRush();

/********** For Testing **********/

void testAuton(Auton testedAuton);

/********** For Competition **********/

void preAuton();

void autonomous();



