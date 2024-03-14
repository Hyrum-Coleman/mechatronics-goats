enum States {
  eMoving = 0,
  eStandbyJSON = 1,
  eStandbyIR = 2,
  eStandbyRC = 3,
  eReloading = 4,
  eSensorDumpMode = 5
};

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

enum RemoteButtons {
  ePwr = 69,
  eVolPlus = 70,
  eFuncStop = 71,
  eBack = 68,         // |<<
  eForward = 64,      // >|
  eFastForward = 67,  // >>|
  eDown = 7,
  eVolMinus = 21,
  eUp = 9,
  eZero = 22,  // 0
  eEq = 25,
  eStRept = 13,
  eOne = 12,
  eTwo = 24,
  eThree = 94,
  eFour = 8,
  eFive = 28,
  eSix = 90,
  eSeven = 66,
  eEight = 82,
  eNine = 74
};

enum BlockColor {
  Red,
  Blue,
  Yellow,
  None,
  UnCalibrated,
};

enum DistanceCalibrationMaterial {
  Button,
  Chassis,
  Cardboard,
};

enum TerminationType {
  LineCovered,
  LineCentered,
  AverageDistanceAway,
  DistanceTraveled,
  TimeExpired,
};

struct Velocities {
  float xDot;
  float yDot;
  float thetaDot;
};

struct Block {
  BlockColor color;
};

struct RGB {
  uint16_t r;
  uint16_t g;
  uint16_t b;
};

struct DrivingTerminationCondition {
  TerminationType type;
  int terminationValue;
  bool tripped;
};


// Union for move-specific parameters
union MoveParameters {
  struct {
    Directions direction;    // which way to drive (will be [x,y,theta] in the future)
    unsigned long duration;  // how far to go (will be distance not time in the future)
  } freedriveParams;         // For eFreeDrive

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

