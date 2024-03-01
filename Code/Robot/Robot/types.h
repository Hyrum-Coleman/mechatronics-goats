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
    ePwr = 0xFD00FF,
    eVolPlus = 0xFD807F,
    eFuncStop = 0xFD40BF,
    eBack = 0xFD20DF,       // |<<
    eForward = 0xFDA05F,    // >|
    eFastForward = 0xFD609F,// >>|
    eDown = 0xFD10EF,
    eVolMinus = 0xFD906F,
    eUp = 0xFD50AF,
    eZero = 0xFD30CF,       // 0
    eEq = 0xFDB04F,
    eStRept = 0xFD708F,
    eOne = 0xFD08F7,
    eTwo = 0xFD8877,
    eThree = 0xFD48B7,
    eFour = 0xFD28D7,
    eFive = 0xFDA857,
    eSix = 0xFD6897,
    eSeven = 0xFD18E7,
    eEight = 0xFD9867,
    eNine = 0xFD58A7,
    eRepeat = 0xFFFFFF
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