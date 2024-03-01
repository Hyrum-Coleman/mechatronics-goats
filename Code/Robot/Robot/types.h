enum Directions {
  eForwards = 1,
  eLeft = 2,
  eBackwards = 3,
  eRight = 4,
  eCCW = 5,
  eCW = 6,
};

enum MoveType {
  eFreeDrive = 1,
  eLineFollow = 2,
  eScissor = 3,
  eBelt = 4,
  eCalibrate = 5,
};

// Onion for move-specific parameters
union MoveParameters {
  struct {
    Directions direction;  // which way to drive (will be [x,y,theta] in the future)
    unsigned long duration;    // how far to go (will be distance not time in the future)
  } freedriveParams;           // For eFreeDrive

  struct {
    unsigned long stopDistance;  // how far to stop away from obstacle when line following
    int speed;
  } linefollowParams;  // For eLineFollow

  struct {
    bool direction;  // which way to move the scissor jack. 1 for up 0 for down.
  } scissorParams;   // For eScissor

  struct {
    bool direction;          // which way to drive the belt
    unsigned long duration;  // how long to drive the belt
  } beltParams;              // For belt

  struct {
    unsigned long duration;  // How long to calibrate line follower
  } calibrationParams;       // For calibrating sensors
};

struct Move {
  MoveType moveType;
  MoveParameters params;
};

enum States {
  eMoving = 0,
  eStandby = 1,
};