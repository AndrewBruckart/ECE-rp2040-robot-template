#include "config.h"
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

enum FollowToTunnelPhase {
  FOLLOW_TO_TUNNEL_PHASE_IDLE = 0,
  FOLLOW_TO_TUNNEL_PHASE_FOLLOW_WALL,
  FOLLOW_TO_TUNNEL_PHASE_COMPLETE
};

enum DriveThroughTunnelPhase {
  DRIVE_THROUGH_TUNNEL_PHASE_IDLE = 0,
  DRIVE_THROUGH_TUNNEL_PHASE_DRIVE_STRAIGHT,
  DRIVE_THROUGH_TUNNEL_PHASE_COMPLETE
};

static const int BACK_OUT_OF_GARAGE_STEP_INDEX = 0;
static const int FRIENDS_HOUSE_STEP_INDEX = 1;
static const int FOLLOW_TO_TUNNEL_STEP_INDEX = 2;
static const int DRIVE_THROUGH_TUNNEL_STEP_INDEX = 3;

static const int GARAGE_STEP_SPEED_PERCENT = 100;
static const int GARAGE_STEP_STRAIGHT_ANGLE = STEERING_STRAIGHT_ANGLE;
static const int GARAGE_STEP_FULL_LEFT_ANGLE = 10;
static const unsigned long GARAGE_BACK_OUT_MS = 500;
static const unsigned long GARAGE_TURN_LEFT_MS = 1400;

static const int FRIENDS_HOUSE_STRAIGHT_ANGLE = STEERING_STRAIGHT_ANGLE;
static const float FRIENDS_HOUSE_RIGHT_WALL_START_INCHES = 4.0f;
static const int FRIENDS_HOUSE_LOW_LIGHT_LDR = 800;
static const int FRIENDS_HOUSE_AMBIENT_LDR = 2650;
static const int TUNNEL_STRAIGHT_ANGLE = STEERING_STRAIGHT_ANGLE;
static const int TUNNEL_STRAIGHT_LOCK_TOLERANCE_DEG = 2;
static const int TUNNEL_LOW_LIGHT_LDR = 2400;
static const int TUNNEL_AMBIENT_LDR = 2600;

static GarageStepPhase garageStepPhase = GARAGE_STEP_PHASE_IDLE;
static unsigned long garagePhaseStartedAtMs = 0;

static FriendsHousePhase friendsHousePhase = FRIENDS_HOUSE_PHASE_IDLE;
static FollowToTunnelPhase followToTunnelPhase = FOLLOW_TO_TUNNEL_PHASE_IDLE;
static DriveThroughTunnelPhase driveThroughTunnelPhase = DRIVE_THROUGH_TUNNEL_PHASE_IDLE;
static bool tunnelSawLowLight = false;

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
  status.steeringAngle = STEERING_STRAIGHT_ANGLE;
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

static WallFollowTuning tunnelStepTuning() {
  return getWallFollowTuning();
}

static bool tunnelWallsPresent(const WallFollowStatus &status) {
  float noWallDistance = wallFollowNoWallDistanceInches();
  return status.leftDistanceInches < noWallDistance && status.rightDistanceInches < noWallDistance;
}

static bool tunnelStraightLockReached(const WallFollowStatus &status) {
  return abs(status.steeringAngle - TUNNEL_STRAIGHT_ANGLE) <= TUNNEL_STRAIGHT_LOCK_TOLERANCE_DEG;
}

static void updateTunnelLightState(int currentLdr) {
  if (currentLdr <= TUNNEL_LOW_LIGHT_LDR) {
    tunnelSawLowLight = true;
  }
}

static bool tunnelAmbientRecovered(int currentLdr) {
  return tunnelSawLowLight && currentLdr >= TUNNEL_AMBIENT_LDR;
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
  WallFollowStatus status = sampleStatusForSide(WALL_SIDE_RIGHT);

  if (friendsHousePhase == FRIENDS_HOUSE_PHASE_FIND_WALL &&
      status.rightDistanceInches <= FRIENDS_HOUSE_RIGHT_WALL_START_INCHES) {
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
      status.wallErrorInches = status.rightDistanceInches - FRIENDS_HOUSE_RIGHT_WALL_START_INCHES;
      status.controlOutputDegrees = 0.0f;
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

static RaceStepControl serviceFollowToTunnel() {
  RaceStepControl control = {};
  control.handled = true;
  control.tuning = tunnelStepTuning();

  WallFollowStatus status = sampleStatusForSide(WALL_SIDE_RIGHT);
  int currentLdr = readLDR();
  updateTunnelLightState(currentLdr);

  if (followToTunnelPhase == FOLLOW_TO_TUNNEL_PHASE_FOLLOW_WALL) {
    status = updateWallFollowControl(
      status.leftDistanceInches,
      status.centerDistanceInches,
      status.rightDistanceInches,
      status.centerRawAdc
    );

    if (tunnelWallsPresent(status) && tunnelStraightLockReached(status)) {
      followToTunnelPhase = FOLLOW_TO_TUNNEL_PHASE_COMPLETE;
    } else if (tunnelAmbientRecovered(currentLdr)) {
      followToTunnelPhase = FOLLOW_TO_TUNNEL_PHASE_COMPLETE;
    }
  }

  switch (followToTunnelPhase) {
    case FOLLOW_TO_TUNNEL_PHASE_FOLLOW_WALL:
      status.state = WALL_FOLLOW_STATE_TUNNEL_FOLLOW;
      break;
    case FOLLOW_TO_TUNNEL_PHASE_COMPLETE:
      status.state = WALL_FOLLOW_STATE_STEP_COMPLETE;
      status.controlOutputDegrees = 0.0f;
      status.steeringAngle = TUNNEL_STRAIGHT_ANGLE;
      status.driveCommand = 0;
      control.finished = true;
      break;
    case FOLLOW_TO_TUNNEL_PHASE_IDLE:
    default:
      break;
  }

  control.status = status;
  return control;
}

static RaceStepControl serviceDriveThroughTunnel() {
  RaceStepControl control = {};
  control.handled = true;
  control.tuning = tunnelStepTuning();

  WallFollowStatus status = sampleStatusForSide(WALL_SIDE_RIGHT);
  int currentLdr = readLDR();
  updateTunnelLightState(currentLdr);

  if (driveThroughTunnelPhase == DRIVE_THROUGH_TUNNEL_PHASE_DRIVE_STRAIGHT) {
    if (tunnelAmbientRecovered(currentLdr)) {
      driveThroughTunnelPhase = DRIVE_THROUGH_TUNNEL_PHASE_COMPLETE;
    }
  }

  switch (driveThroughTunnelPhase) {
    case DRIVE_THROUGH_TUNNEL_PHASE_DRIVE_STRAIGHT:
      status.state = WALL_FOLLOW_STATE_TUNNEL_STRAIGHT;
      status.selectedWall = WALL_SIDE_RIGHT;
      status.activeWallDistanceInches = status.rightDistanceInches;
      status.wallErrorInches = 0.0f;
      status.controlOutputDegrees = 0.0f;
      status.steeringAngle = TUNNEL_STRAIGHT_ANGLE;
      status.driveCommand = 1;
      break;
    case DRIVE_THROUGH_TUNNEL_PHASE_COMPLETE:
      status.state = WALL_FOLLOW_STATE_STEP_COMPLETE;
      status.selectedWall = WALL_SIDE_RIGHT;
      status.activeWallDistanceInches = status.rightDistanceInches;
      status.wallErrorInches = 0.0f;
      status.controlOutputDegrees = 0.0f;
      status.steeringAngle = TUNNEL_STRAIGHT_ANGLE;
      status.driveCommand = 0;
      control.finished = true;
      break;
    case DRIVE_THROUGH_TUNNEL_PHASE_IDLE:
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
  followToTunnelPhase = FOLLOW_TO_TUNNEL_PHASE_IDLE;
  driveThroughTunnelPhase = DRIVE_THROUGH_TUNNEL_PHASE_IDLE;
  tunnelSawLowLight = false;
  setWallFollowBackupEnabled(true);
}

bool raceStepUsesCustomControl(int stepIndex) {
  return stepIndex == BACK_OUT_OF_GARAGE_STEP_INDEX ||
    stepIndex == FRIENDS_HOUSE_STEP_INDEX ||
    stepIndex == FOLLOW_TO_TUNNEL_STEP_INDEX ||
    stepIndex == DRIVE_THROUGH_TUNNEL_STEP_INDEX;
}

int nextImplementedRaceStepIndex(int stepIndex) {
  switch (stepIndex) {
    case BACK_OUT_OF_GARAGE_STEP_INDEX:
      return FRIENDS_HOUSE_STEP_INDEX;
    case FRIENDS_HOUSE_STEP_INDEX:
      return FOLLOW_TO_TUNNEL_STEP_INDEX;
    case FOLLOW_TO_TUNNEL_STEP_INDEX:
      return DRIVE_THROUGH_TUNNEL_STEP_INDEX;
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
      setWallFollowBackupEnabled(true);
      garageStepPhase = GARAGE_STEP_PHASE_BACK_OUT;
      garagePhaseStartedAtMs = millis();
      break;
    case FRIENDS_HOUSE_STEP_INDEX:
      setWallFollowBackupEnabled(false);
      setWallFollowSide(WALL_SIDE_RIGHT);
      resetWallFollowController();
      friendsHousePhase = FRIENDS_HOUSE_PHASE_FIND_WALL;
      break;
    case FOLLOW_TO_TUNNEL_STEP_INDEX:
      setWallFollowBackupEnabled(true);
      setWallFollowSide(WALL_SIDE_RIGHT);
      resetWallFollowController();
      followToTunnelPhase = FOLLOW_TO_TUNNEL_PHASE_FOLLOW_WALL;
      tunnelSawLowLight = false;
      break;
    case DRIVE_THROUGH_TUNNEL_STEP_INDEX:
      setWallFollowBackupEnabled(false);
      driveThroughTunnelPhase = DRIVE_THROUGH_TUNNEL_PHASE_DRIVE_STRAIGHT;
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
    case FOLLOW_TO_TUNNEL_STEP_INDEX:
      if (followToTunnelPhase == FOLLOW_TO_TUNNEL_PHASE_IDLE) {
        return RaceStepControl();
      }
      return serviceFollowToTunnel();
    case DRIVE_THROUGH_TUNNEL_STEP_INDEX:
      if (driveThroughTunnelPhase == DRIVE_THROUGH_TUNNEL_PHASE_IDLE) {
        return RaceStepControl();
      }
      return serviceDriveThroughTunnel();
    default:
      return RaceStepControl();
  }
}
