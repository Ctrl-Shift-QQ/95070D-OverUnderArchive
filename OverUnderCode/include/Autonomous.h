/********** Enums for Auton **********/

typedef enum {
    ProgSkills = 0
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

void swingTo(double target, Direction side, Direction direction);

void intake();

void outake(double waitTime);

/********** Skills **********/

void runProgrammingSkills();

void preAuton();

void autonomous();



