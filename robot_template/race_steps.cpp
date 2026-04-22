#include "race_steps.h"
#include "sensors.h"
#include <Arduino.h>

enum GarageStepPhase {
  GARAGE_STEP_PHASE_IDLE = 0,
  GARAGE_STEP_PHASE_BACK_OUT,
  GARAGE_STEP_PHASE_TURN_LEFT,
  GARAGE_STEP_PHASE_COMPLETE
};

enum FriendsHousePhase {
  FRIENDS_HOUSE_PHASE_IDLE = 0,
  FRIENDS_HOUSE_PHASE_FIND_WALL,
  FRIENDS_HOUSE_PHASE_FOLLOW_WALL,
  FRIENDS_HOUSE_PHASE_LOW_LIGHT,
  FRIENDS_HOUSE_PHASE_COMPLETE
};

static const int BACK_OUT_OF_GARAGE_STEP_INDEX = 0;
static const int FRIENDS_HOUSE_STEP_INDEX = 1;

static const int GARAGE_STEP_SPEED_PERCENT = 100;
static const int GARAGE_STEP_STRAIGHT_ANGLE = 90;
static const int GARAGE_STEP_FULL_LEFT_ANGLE = 0;
static const unsigned long GARAGE_BACK_OUT_MS = 500;
static const unsigned long GARAGE_TURN_LEFT_MS = 1400;

static const int FRIENDS_HOUSE_STRAIGHT_ANGLE = 90;
static const int FRIENDS_HOUSE_RIGHT_WALL_ADC_START = 500;
static const int FRIENDS_HOUSE_LOW_LIGHT_LDR = 800;
static const int FRIENDS_HOUSE_AMBIENT_LDR = 2650;

static GarageStepPhase garageStepPhase = GARAGE_STEP_PHASE_IDLE;
static unsigned long garagePhaseStartedAtMs = 0;

static FriendsHousePhase friendsHousePhase = FRIENDS_HOUSE_PHASE_IDLE;

static WallFollowStatus sampleStatusForSide(WallFollowSide side) {
  WallFollowStatus status = {};
  status.selectedWall = side;
  status.centerRawAdc = readIRCenter();
  status.leftDistanceInches = readIRDistanceInches(IR_SENSOR_LEFT);
  status.centerDistanceInches = readIRDistanceInches(IR_SENSOR_CENTER);
  status.rightDistanceInches = readIRDistanceInches(IR_SENSOR_RIGHT);
  status.activeWallDistanceInches = (side == WALL_SIDE_RIGHT) ? status.rightDistanceInches : status.leftDistanceInches;
  status.wallErrorInches = 0.0f;
  status.controlOutputDegrees = 0.0f;
  status.steeringAngle = 90;
  status.driveCommand = 0;
  return status;
}

static WallFollowTuning garageStepTuning() {
  WallFollowTuning tuning = getWallFollowTuning();
  tuning.motorSpeedPercent = GARAGE_STEP_SPEED_PERCENT;
  return tuning;
}

static WallFollowTuning friendsHouseTuning() {
  WallFollowTuning tuning = getWallFollowTuning();
  tuning.motorSpeedPercent = 100;
  return tuning;
}

static RaceStepControl serviceBackOutOfGarage() {
  RaceStepControl control = {};
  control.handled = true;
  control.tuning = garageStepTuning();

  WallFollowStatus status = sampleStatusForSide(WALL_SIDE_LEFT);
  unsigned long now = millis();

  if (garageStepPhase == GARAGE_STEP_PHASE_BACK_OUT &&
      now - garagePhaseStartedAtMs >= GARAGE_BACK_OUT_MS) {
    garageStepPhase = GARAGE_STEP_PHASE_TURN_LEFT;
    garagePhaseStartedAtMs = now;
  }

  if (garageStepPhase == GARAGE_STEP_PHASE_TURN_LEFT &&
      now - garagePhaseStartedAtMs >= GARAGE_TURN_LEFT_MS) {
    garageStepPhase = GARAGE_STEP_PHASE_COMPLETE;
  }

  switch (garageStepPhase) {
    case GARAGE_STEP_PHASE_BACK_OUT:
      status.state = WALL_FOLLOW_STATE_GARAGE_BACK_OUT;
      status.wallErrorInches = status.activeWallDistanceInches - wallFollowNoWallDistanceInches();
      status.controlOutputDegrees = 0.0f;
      status.steeringAngle = GARAGE_STEP_STRAIGHT_ANGLE;
      status.driveCommand = -1;
      break;
    case GARAGE_STEP_PHASE_TURN_LEFT:
      status.state = WALL_FOLLOW_STATE_GARAGE_TURN_LEFT;
      status.wallErrorInches = status.activeWallDistanceInches - wallFollowNoWallDistanceInches();
      status.controlOutputDegrees = (float)(GARAGE_STEP_FULL_LEFT_ANGLE - GARAGE_STEP_STRAIGHT_ANGLE);
      status.steeringAngle = GARAGE_STEP_FULL_LEFT_ANGLE;
      status.driveCommand = -1;
      break;
    case GARAGE_STEP_PHASE_COMPLETE:
      status.state = WALL_FOLLOW_STATE_STEP_COMPLETE;
      status.wallErrorInches = status.activeWallDistanceInches - wallFollowNoWallDistanceInches();
      status.controlOutputDegrees = (float)(GARAGE_STEP_FULL_LEFT_ANGLE - GARAGE_STEP_STRAIGHT_ANGLE);
      status.steeringAngle = GARAGE_STEP_FULL_LEFT_ANGLE;
      status.driveCommand = 0;
      control.finished = true;
      break;
    case GARAGE_STEP_PHASE_IDLE:
    default:
      break;
  }

  control.status = status;
  return control;
}

static RaceStepControl serviceFriendsHouse() {
  RaceStepControl control = {};
  control.handled = true;
  control.tuning = friendsHouseTuning();

  int currentLdr = readLDR();
  int rightRaw = readIRRight();
  WallFollowStatus status = sampleStatusForSide(WALL_SIDE_RIGHT);

  if (friendsHousePhase == FRIENDS_HOUSE_PHASE_FIND_WALL &&
      rightRaw <= FRIENDS_HOUSE_RIGHT_WALL_ADC_START) {
    friendsHousePhase = FRIENDS_HOUSE_PHASE_FOLLOW_WALL;
    resetWallFollowController();
  }

  if (friendsHousePhase == FRIENDS_HOUSE_PHASE_FOLLOW_WALL || friendsHousePhase == FRIENDS_HOUSE_PHASE_LOW_LIGHT) {
    status = updateWallFollowControl(
      status.leftDistanceInches,
      status.centerDistanceInches,
      status.rightDistanceInches,
      status.centerRawAdc
    );

    if (friendsHousePhase == FRIENDS_HOUSE_PHASE_FOLLOW_WALL && currentLdr <= FRIENDS_HOUSE_LOW_LIGHT_LDR) {
      friendsHousePhase = FRIENDS_HOUSE_PHASE_LOW_LIGHT;
    } else if (friendsHousePhase == FRIENDS_HOUSE_PHASE_LOW_LIGHT && currentLdr >= FRIENDS_HOUSE_AMBIENT_LDR) {
      friendsHousePhase = FRIENDS_HOUSE_PHASE_COMPLETE;
    }
  }

  switch (friendsHousePhase) {
    case FRIENDS_HOUSE_PHASE_FIND_WALL:
      status.state = WALL_FOLLOW_STATE_FRIENDS_FIND_WALL;
      status.selectedWall = WALL_SIDE_RIGHT;
      status.activeWallDistanceInches = status.rightDistanceInches;
      status.wallErrorInches = (float)(rightRaw - FRIENDS_HOUSE_RIGHT_WALL_ADC_START);
      status.controlOutputDegrees = (float)(FRIENDS_HOUSE_STRAIGHT_ANGLE - 90);
      status.steeringAngle = FRIENDS_HOUSE_STRAIGHT_ANGLE;
      status.driveCommand = 1;
      break;
    case FRIENDS_HOUSE_PHASE_FOLLOW_WALL:
      status.state = WALL_FOLLOW_STATE_FRIENDS_FOLLOW;
      break;
    case FRIENDS_HOUSE_PHASE_LOW_LIGHT:
      status.state = WALL_FOLLOW_STATE_FRIENDS_LOW_LIGHT;
      break;
    case FRIENDS_HOUSE_PHASE_COMPLETE:
      status.state = WALL_FOLLOW_STATE_STEP_COMPLETE;
      status.driveCommand = 0;
      control.finished = true;
      break;
    case FRIENDS_HOUSE_PHASE_IDLE:
    default:
      break;
  }

  control.status = status;
  return control;
}

void initRaceSteps() {
  resetRaceStepControl();
}

void resetRaceStepControl() {
  garageStepPhase = GARAGE_STEP_PHASE_IDLE;
  garagePhaseStartedAtMs = 0;
  friendsHousePhase = FRIENDS_HOUSE_PHASE_IDLE;
}

bool raceStepUsesCustomControl(int stepIndex) {
  return stepIndex == BACK_OUT_OF_GARAGE_STEP_INDEX || stepIndex == FRIENDS_HOUSE_STEP_INDEX;
}

int nextImplementedRaceStepIndex(int stepIndex) {
  switch (stepIndex) {
    case BACK_OUT_OF_GARAGE_STEP_INDEX:
      return FRIENDS_HOUSE_STEP_INDEX;
    default:
      return -1;
  }
}

bool raceStepCoastsOnFinish(int stepIndex) {
  return stepIndex == FRIENDS_HOUSE_STEP_INDEX;
}

void beginRaceStepControl(int stepIndex) {
  switch (stepIndex) {
    case BACK_OUT_OF_GARAGE_STEP_INDEX:
      garageStepPhase = GARAGE_STEP_PHASE_BACK_OUT;
      garagePhaseStartedAtMs = millis();
      break;
    case FRIENDS_HOUSE_STEP_INDEX:
      setWallFollowSide(WALL_SIDE_RIGHT);
      resetWallFollowController();
      friendsHousePhase = FRIENDS_HOUSE_PHASE_FIND_WALL;
      break;
    default:
      break;
  }
}

RaceStepControl serviceRaceStepControl(int stepIndex) {
  if (!raceStepUsesCustomControl(stepIndex)) {
    return RaceStepControl();
  }

  switch (stepIndex) {
    case BACK_OUT_OF_GARAGE_STEP_INDEX:
      if (garageStepPhase == GARAGE_STEP_PHASE_IDLE) {
        return RaceStepControl();
      }
      return serviceBackOutOfGarage();
    case FRIENDS_HOUSE_STEP_INDEX:
      if (friendsHousePhase == FRIENDS_HOUSE_PHASE_IDLE) {
        return RaceStepControl();
      }
      return serviceFriendsHouse();
    default:
      return RaceStepControl();
  }
}
