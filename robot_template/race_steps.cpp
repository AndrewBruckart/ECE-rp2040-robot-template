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
  FOLLOW_TO_TUNNEL_PHASE_TURN,
  FOLLOW_TO_TUNNEL_PHASE_FIND_LEFT_WALL,
  FOLLOW_TO_TUNNEL_PHASE_FOLLOW_LEFT_WALL,
  FOLLOW_TO_TUNNEL_PHASE_COMPLETE
};

enum DriveThroughTunnelPhase {
  DRIVE_THROUGH_TUNNEL_PHASE_IDLE = 0,
  DRIVE_THROUGH_TUNNEL_PHASE_FOLLOW_RIGHT_WALL,
  DRIVE_THROUGH_TUNNEL_PHASE_COMPLETE
};

enum AfterTunnelPhase {
  AFTER_TUNNEL_PHASE_IDLE = 0,
  AFTER_TUNNEL_PHASE_EXIT_TURN,
  AFTER_TUNNEL_PHASE_DRIVE_STRAIGHT,
  AFTER_TUNNEL_PHASE_APPROACH_WALL,
  AFTER_TUNNEL_PHASE_WALL_FOLLOW,
  AFTER_TUNNEL_PHASE_FIRST_WF_PAUSE,
  AFTER_TUNNEL_PHASE_CLOSE_LEFT_WF,
  AFTER_TUNNEL_PHASE_CHARGE_STOP,
  AFTER_TUNNEL_PHASE_WALL_FOLLOW_2,
  AFTER_TUNNEL_PHASE_FINAL_LEFT_WF,
  AFTER_TUNNEL_PHASE_BACK_INTO_GAP
};

static const int BACK_OUT_OF_GARAGE_STEP_INDEX = 0;
static const int FRIENDS_HOUSE_STEP_INDEX = 1;
static const int FOLLOW_TO_TUNNEL_STEP_INDEX = 2;
static const int DRIVE_THROUGH_TUNNEL_STEP_INDEX = 3;
static const int AFTER_TUNNEL_STEP_INDEX = 4;

static const int GARAGE_STEP_SPEED_PERCENT = 100;
static const int GARAGE_STEP_STRAIGHT_ANGLE = STEERING_STRAIGHT_ANGLE;
static const int GARAGE_STEP_FULL_LEFT_ANGLE = 10;
static const unsigned long GARAGE_BACK_OUT_MS = 500;
static const unsigned long GARAGE_TURN_LEFT_MS = 1400;

static const int FRIENDS_HOUSE_STRAIGHT_ANGLE = STEERING_STRAIGHT_ANGLE;
static const float FRIENDS_HOUSE_RIGHT_WALL_START_INCHES = 4.0f;
static const int FRIENDS_HOUSE_LOW_LIGHT_LDR = 1200;
static const int FRIENDS_HOUSE_EXIT_DARK_LDR = 2400;
static const unsigned long FRIENDS_HOUSE_DARK_DWELL_MS = 200;
static const int FOLLOW_TO_TUNNEL_TURN_ANGLE = 45;
static const unsigned long FOLLOW_TO_TUNNEL_TURN_MS = 500;
static const int FOLLOW_TO_TUNNEL_STRAIGHT_ANGLE = 77;
static const float FOLLOW_TO_TUNNEL_LEFT_WALL_START_INCHES = 8.0f;
static const int FOLLOW_TO_TUNNEL_LIGHT_DROP_DELTA_LDR = 100;
static const float TUNNEL_TARGET_DISTANCE_INCHES = 3.0f;
static const int TUNNEL_AMBIENT_LDR = 2600;

static const int AFTER_TUNNEL_EXIT_TURN_ANGLE = FOLLOW_TO_TUNNEL_TURN_ANGLE;
static const unsigned long AFTER_TUNNEL_EXIT_TURN_MS = FOLLOW_TO_TUNNEL_TURN_MS;
static const int AFTER_TUNNEL_STRAIGHT_ANGLE = 76;
static const unsigned long AFTER_TUNNEL_STRAIGHT_MS = 1000;
static const float AFTER_TUNNEL_WALL_DETECT_INCHES = 6.0f;
static const float AFTER_TUNNEL_LEFT_WALL_DETECT_INCHES = 6.0f;
static const float AFTER_TUNNEL_LEFT_WALL_LOST_INCHES = 9.0f;
static const float AFTER_TUNNEL_RIGHT_WALL_CONFIRM_INCHES = 8.0f;
static const unsigned long AFTER_TUNNEL_LEFT_WALL_SUSTAIN_MS = 750;
static const unsigned long AFTER_TUNNEL_WALL2_SWAP_SUSTAIN_MS = 200;
static const unsigned long AFTER_TUNNEL_FIRST_WF_PAUSE_MS = 500;
static const float AFTER_TUNNEL_CLOSE_LEFT_TARGET_INCHES = 2.5f;
static const unsigned long AFTER_TUNNEL_CLOSE_LEFT_WF_MS = 2000;
static const unsigned long AFTER_TUNNEL_CHARGE_STOP_MS = 2000;
static const float AFTER_TUNNEL_FINAL_LEFT_TARGET_INCHES = 2.5f;
static const unsigned long AFTER_TUNNEL_BACK_INTO_GAP_MS = 2200;
static const int AFTER_TUNNEL_BACK_STEER_ANGLE = 160;

static GarageStepPhase garageStepPhase = GARAGE_STEP_PHASE_IDLE;
static unsigned long garagePhaseStartedAtMs = 0;

static FriendsHousePhase friendsHousePhase = FRIENDS_HOUSE_PHASE_IDLE;
static unsigned long friendsHouseDarkDetectedAtMs = 0;
static bool friendsHouseDarkTimingActive = false;
static FollowToTunnelPhase followToTunnelPhase = FOLLOW_TO_TUNNEL_PHASE_IDLE;
static unsigned long followToTunnelPhaseStartedAtMs = 0;
static int followToTunnelStartLdr = 0;
static DriveThroughTunnelPhase driveThroughTunnelPhase = DRIVE_THROUGH_TUNNEL_PHASE_IDLE;

static AfterTunnelPhase afterTunnelPhase = AFTER_TUNNEL_PHASE_IDLE;
static unsigned long afterTunnelPhaseStartedAtMs = 0;
static int afterTunnelLeftWallCount = 0;
static unsigned long afterTunnelLeftWallSeenAtMs = 0;
static bool afterTunnelLeftWallActive = false;
static bool afterTunnelWall2Armed = false;
static WallFollowTuning afterTunnelSavedTuning = {};

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
  WallFollowTuning tuning = getWallFollowTuning();
  tuning.targetWallDistanceInches = TUNNEL_TARGET_DISTANCE_INCHES;
  return tuning;
}

static WallFollowTuning afterTunnelTuning() {
  WallFollowTuning tuning = getWallFollowTuning();
  tuning.motorSpeedPercent = 100;
  return tuning;
}

static int followToTunnelLightDropThreshold() {
  return max(0, followToTunnelStartLdr - FOLLOW_TO_TUNNEL_LIGHT_DROP_DELTA_LDR);
}

static bool followToTunnelLightDropped(int currentLdr) {
  return currentLdr <= followToTunnelLightDropThreshold();
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
  unsigned long now = millis();
  WallFollowStatus status = sampleStatusForSide(WALL_SIDE_RIGHT);

  if (friendsHousePhase != FRIENDS_HOUSE_PHASE_LOW_LIGHT &&
      friendsHousePhase != FRIENDS_HOUSE_PHASE_COMPLETE) {
    if (currentLdr <= FRIENDS_HOUSE_LOW_LIGHT_LDR) {
      if (!friendsHouseDarkTimingActive) {
        friendsHouseDarkTimingActive = true;
        friendsHouseDarkDetectedAtMs = now;
      } else if (now - friendsHouseDarkDetectedAtMs >= FRIENDS_HOUSE_DARK_DWELL_MS) {
        friendsHousePhase = FRIENDS_HOUSE_PHASE_LOW_LIGHT;
      }
    } else {
      friendsHouseDarkTimingActive = false;
      friendsHouseDarkDetectedAtMs = 0;
    }
  }

  if (friendsHousePhase == FRIENDS_HOUSE_PHASE_LOW_LIGHT &&
      currentLdr >= FRIENDS_HOUSE_EXIT_DARK_LDR) {
    friendsHousePhase = FRIENDS_HOUSE_PHASE_COMPLETE;
  }

  if (friendsHousePhase == FRIENDS_HOUSE_PHASE_FIND_WALL &&
      status.rightDistanceInches <= FRIENDS_HOUSE_RIGHT_WALL_START_INCHES) {
    friendsHousePhase = FRIENDS_HOUSE_PHASE_FOLLOW_WALL;
    resetWallFollowController();
  }

  if (friendsHousePhase == FRIENDS_HOUSE_PHASE_FOLLOW_WALL) {
    status = updateWallFollowControl(
      status.leftDistanceInches,
      status.centerDistanceInches,
      status.rightDistanceInches,
      status.centerRawAdc
    );
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
      status.selectedWall = WALL_SIDE_RIGHT;
      status.activeWallDistanceInches = status.rightDistanceInches;
      status.wallErrorInches = 0.0f;
      status.controlOutputDegrees = 0.0f;
      status.steeringAngle = FRIENDS_HOUSE_STRAIGHT_ANGLE;
      status.driveCommand = 1;
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

  WallFollowStatus status = sampleStatusForSide(WALL_SIDE_LEFT);
  int currentLdr = readLDR();
  unsigned long now = millis();

  if (followToTunnelPhase == FOLLOW_TO_TUNNEL_PHASE_TURN &&
      now - followToTunnelPhaseStartedAtMs >= FOLLOW_TO_TUNNEL_TURN_MS) {
    followToTunnelPhase = FOLLOW_TO_TUNNEL_PHASE_FIND_LEFT_WALL;
    followToTunnelPhaseStartedAtMs = now;
  }

  if (followToTunnelPhase != FOLLOW_TO_TUNNEL_PHASE_IDLE &&
      followToTunnelPhase != FOLLOW_TO_TUNNEL_PHASE_COMPLETE &&
      followToTunnelLightDropped(currentLdr)) {
    followToTunnelPhase = FOLLOW_TO_TUNNEL_PHASE_COMPLETE;
  }

  if (followToTunnelPhase == FOLLOW_TO_TUNNEL_PHASE_FIND_LEFT_WALL &&
      status.leftDistanceInches <= FOLLOW_TO_TUNNEL_LEFT_WALL_START_INCHES) {
    followToTunnelPhase = FOLLOW_TO_TUNNEL_PHASE_FOLLOW_LEFT_WALL;
    setWallFollowSide(WALL_SIDE_LEFT);
    resetWallFollowController();
  }

  if (followToTunnelPhase == FOLLOW_TO_TUNNEL_PHASE_FOLLOW_LEFT_WALL) {
    status = updateWallFollowControl(
      status.leftDistanceInches,
      status.centerDistanceInches,
      status.rightDistanceInches,
      status.centerRawAdc
    );
  }

  switch (followToTunnelPhase) {
    case FOLLOW_TO_TUNNEL_PHASE_TURN:
      status.state = WALL_FOLLOW_STATE_TUNNEL_TURN;
      status.selectedWall = WALL_SIDE_LEFT;
      status.activeWallDistanceInches = status.leftDistanceInches;
      status.wallErrorInches = status.leftDistanceInches - wallFollowNoWallDistanceInches();
      status.controlOutputDegrees = (float)(FOLLOW_TO_TUNNEL_TURN_ANGLE - FOLLOW_TO_TUNNEL_STRAIGHT_ANGLE);
      status.steeringAngle = FOLLOW_TO_TUNNEL_TURN_ANGLE;
      status.driveCommand = 1;
      break;
    case FOLLOW_TO_TUNNEL_PHASE_FIND_LEFT_WALL:
      status.state = WALL_FOLLOW_STATE_TUNNEL_FIND_WALL;
      status.selectedWall = WALL_SIDE_LEFT;
      status.activeWallDistanceInches = status.leftDistanceInches;
      status.wallErrorInches = status.leftDistanceInches - FOLLOW_TO_TUNNEL_LEFT_WALL_START_INCHES;
      status.controlOutputDegrees = 0.0f;
      status.steeringAngle = FOLLOW_TO_TUNNEL_STRAIGHT_ANGLE;
      status.driveCommand = 1;
      break;
    case FOLLOW_TO_TUNNEL_PHASE_FOLLOW_LEFT_WALL:
      status.state = WALL_FOLLOW_STATE_TUNNEL_FOLLOW;
      break;
    case FOLLOW_TO_TUNNEL_PHASE_COMPLETE:
      status.state = WALL_FOLLOW_STATE_STEP_COMPLETE;
      status.selectedWall = WALL_SIDE_LEFT;
      status.activeWallDistanceInches = status.leftDistanceInches;
      status.wallErrorInches = 0.0f;
      status.controlOutputDegrees = 0.0f;
      status.steeringAngle = FOLLOW_TO_TUNNEL_STRAIGHT_ANGLE;
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

  if (driveThroughTunnelPhase == DRIVE_THROUGH_TUNNEL_PHASE_FOLLOW_RIGHT_WALL &&
      currentLdr >= TUNNEL_AMBIENT_LDR) {
    driveThroughTunnelPhase = DRIVE_THROUGH_TUNNEL_PHASE_COMPLETE;
  }

  if (driveThroughTunnelPhase == DRIVE_THROUGH_TUNNEL_PHASE_FOLLOW_RIGHT_WALL) {
    status = updateWallFollowControl(
      status.leftDistanceInches,
      status.centerDistanceInches,
      status.rightDistanceInches,
      status.centerRawAdc
    );
  }

  switch (driveThroughTunnelPhase) {
    case DRIVE_THROUGH_TUNNEL_PHASE_FOLLOW_RIGHT_WALL:
      break;
    case DRIVE_THROUGH_TUNNEL_PHASE_COMPLETE:
      status.state = WALL_FOLLOW_STATE_STEP_COMPLETE;
      status.selectedWall = WALL_SIDE_RIGHT;
      status.activeWallDistanceInches = status.rightDistanceInches;
      status.wallErrorInches = 0.0f;
      status.controlOutputDegrees = 0.0f;
      status.steeringAngle = STEERING_STRAIGHT_ANGLE;
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

static RaceStepControl serviceAfterTunnel() {
  RaceStepControl control = {};
  control.handled = true;
  control.tuning = afterTunnelTuning();

  WallFollowStatus status = sampleStatusForSide(WALL_SIDE_RIGHT);
  unsigned long now = millis();

  if (afterTunnelPhase == AFTER_TUNNEL_PHASE_EXIT_TURN &&
      now - afterTunnelPhaseStartedAtMs >= AFTER_TUNNEL_EXIT_TURN_MS) {
    afterTunnelPhase = AFTER_TUNNEL_PHASE_DRIVE_STRAIGHT;
    afterTunnelPhaseStartedAtMs = now;
  }

  if (afterTunnelPhase == AFTER_TUNNEL_PHASE_DRIVE_STRAIGHT &&
      now - afterTunnelPhaseStartedAtMs >= AFTER_TUNNEL_STRAIGHT_MS) {
    afterTunnelPhase = AFTER_TUNNEL_PHASE_APPROACH_WALL;
  }

  if (afterTunnelPhase == AFTER_TUNNEL_PHASE_APPROACH_WALL &&
      status.centerDistanceInches <= AFTER_TUNNEL_WALL_DETECT_INCHES) {
    afterTunnelPhase = AFTER_TUNNEL_PHASE_WALL_FOLLOW;
    resetWallFollowController();
  }

  if (afterTunnelPhase == AFTER_TUNNEL_PHASE_FIRST_WF_PAUSE &&
      now - afterTunnelPhaseStartedAtMs >= AFTER_TUNNEL_FIRST_WF_PAUSE_MS) {
    afterTunnelSavedTuning = getWallFollowTuning();
    afterTunnelPhase = AFTER_TUNNEL_PHASE_CLOSE_LEFT_WF;
    afterTunnelPhaseStartedAtMs = now;
    setWallFollowSide(WALL_SIDE_LEFT);
    setWallFollowTargetDistance(AFTER_TUNNEL_CLOSE_LEFT_TARGET_INCHES);
    resetWallFollowController();
  }

  if (afterTunnelPhase == AFTER_TUNNEL_PHASE_CLOSE_LEFT_WF &&
      now - afterTunnelPhaseStartedAtMs >= AFTER_TUNNEL_CLOSE_LEFT_WF_MS) {
    afterTunnelPhase = AFTER_TUNNEL_PHASE_CHARGE_STOP;
    afterTunnelPhaseStartedAtMs = now;
    setWallFollowTargetDistance(afterTunnelSavedTuning.targetWallDistanceInches);
  }

  if (afterTunnelPhase == AFTER_TUNNEL_PHASE_CHARGE_STOP &&
      now - afterTunnelPhaseStartedAtMs >= AFTER_TUNNEL_CHARGE_STOP_MS) {
    afterTunnelPhase = AFTER_TUNNEL_PHASE_WALL_FOLLOW_2;
    setWallFollowSide(WALL_SIDE_RIGHT);
    resetWallFollowController();
    afterTunnelLeftWallActive = false;
    afterTunnelLeftWallSeenAtMs = 0;
    afterTunnelWall2Armed = false;
  }

  if (afterTunnelPhase == AFTER_TUNNEL_PHASE_FINAL_LEFT_WF &&
      status.leftDistanceInches >= AFTER_TUNNEL_LEFT_WALL_LOST_INCHES) {
    afterTunnelPhase = AFTER_TUNNEL_PHASE_BACK_INTO_GAP;
    afterTunnelPhaseStartedAtMs = now;
    setWallFollowTargetDistance(afterTunnelSavedTuning.targetWallDistanceInches);
  }

  bool isWfPhase = afterTunnelPhase == AFTER_TUNNEL_PHASE_WALL_FOLLOW ||
                   afterTunnelPhase == AFTER_TUNNEL_PHASE_FIRST_WF_PAUSE ||
                   afterTunnelPhase == AFTER_TUNNEL_PHASE_CLOSE_LEFT_WF ||
                   afterTunnelPhase == AFTER_TUNNEL_PHASE_WALL_FOLLOW_2 ||
                   afterTunnelPhase == AFTER_TUNNEL_PHASE_FINAL_LEFT_WF;

  if (isWfPhase) {
    status = updateWallFollowControl(
      status.leftDistanceInches,
      status.centerDistanceInches,
      status.rightDistanceInches,
      status.centerRawAdc
    );
  }

  bool isRightWfPhase = afterTunnelPhase == AFTER_TUNNEL_PHASE_WALL_FOLLOW ||
                        afterTunnelPhase == AFTER_TUNNEL_PHASE_WALL_FOLLOW_2;

  if (isRightWfPhase) {
    if (afterTunnelPhase == AFTER_TUNNEL_PHASE_WALL_FOLLOW_2 && !afterTunnelWall2Armed) {
      if (status.leftDistanceInches >= AFTER_TUNNEL_LEFT_WALL_LOST_INCHES) {
        afterTunnelWall2Armed = true;
        afterTunnelLeftWallSeenAtMs = 0;
      }
    }

    bool canDetect = (afterTunnelPhase == AFTER_TUNNEL_PHASE_WALL_FOLLOW) ||
                     (afterTunnelPhase == AFTER_TUNNEL_PHASE_WALL_FOLLOW_2 && afterTunnelWall2Armed);

    if (canDetect) {
      if (!afterTunnelLeftWallActive) {
        bool firstTriggerDetected = status.leftDistanceInches <= AFTER_TUNNEL_LEFT_WALL_DETECT_INCHES &&
                                    status.rightDistanceInches <= AFTER_TUNNEL_RIGHT_WALL_CONFIRM_INCHES;
        bool secondTriggerDetected = status.leftDistanceInches <= AFTER_TUNNEL_LEFT_WALL_DETECT_INCHES;
        bool triggerDetected = (afterTunnelLeftWallCount == 0) ? firstTriggerDetected : secondTriggerDetected;

        if (triggerDetected) {
          if (afterTunnelLeftWallSeenAtMs == 0) {
            afterTunnelLeftWallSeenAtMs = now;
          } else if (now - afterTunnelLeftWallSeenAtMs >=
                     ((afterTunnelLeftWallCount == 0) ? AFTER_TUNNEL_LEFT_WALL_SUSTAIN_MS
                                                      : AFTER_TUNNEL_WALL2_SWAP_SUSTAIN_MS)) {
            afterTunnelLeftWallActive = true;
            afterTunnelLeftWallCount++;
            if (afterTunnelLeftWallCount == 1) {
              afterTunnelPhase = AFTER_TUNNEL_PHASE_FIRST_WF_PAUSE;
              afterTunnelPhaseStartedAtMs = now;
            } else {
              afterTunnelPhase = AFTER_TUNNEL_PHASE_FINAL_LEFT_WF;
              afterTunnelPhaseStartedAtMs = now;
              setWallFollowSide(WALL_SIDE_LEFT);
              setWallFollowTargetDistance(AFTER_TUNNEL_FINAL_LEFT_TARGET_INCHES);
              resetWallFollowController();
            }
          }
        } else {
          afterTunnelLeftWallSeenAtMs = 0;
        }
      } else if (status.leftDistanceInches >= AFTER_TUNNEL_LEFT_WALL_LOST_INCHES) {
        afterTunnelLeftWallActive = false;
        afterTunnelLeftWallSeenAtMs = 0;
      }
    }
  }

  switch (afterTunnelPhase) {
    case AFTER_TUNNEL_PHASE_EXIT_TURN:
      status.state = WALL_FOLLOW_STATE_AFTER_TUNNEL_EXIT_TURN;
      status.wallErrorInches = 0.0f;
      status.controlOutputDegrees = (float)(AFTER_TUNNEL_EXIT_TURN_ANGLE - AFTER_TUNNEL_STRAIGHT_ANGLE);
      status.steeringAngle = AFTER_TUNNEL_EXIT_TURN_ANGLE;
      status.driveCommand = 1;
      break;
    case AFTER_TUNNEL_PHASE_DRIVE_STRAIGHT:
      status.state = WALL_FOLLOW_STATE_AFTER_TUNNEL_STRAIGHT;
      status.wallErrorInches = 0.0f;
      status.controlOutputDegrees = 0.0f;
      status.steeringAngle = AFTER_TUNNEL_STRAIGHT_ANGLE;
      status.driveCommand = 1;
      break;
    case AFTER_TUNNEL_PHASE_APPROACH_WALL:
      status.state = WALL_FOLLOW_STATE_AFTER_TUNNEL_APPROACH;
      status.wallErrorInches = 0.0f;
      status.controlOutputDegrees = 0.0f;
      status.steeringAngle = AFTER_TUNNEL_STRAIGHT_ANGLE;
      status.driveCommand = 1;
      break;
    case AFTER_TUNNEL_PHASE_WALL_FOLLOW:
      status.state = WALL_FOLLOW_STATE_AFTER_TUNNEL_FOLLOW;
      break;
    case AFTER_TUNNEL_PHASE_FIRST_WF_PAUSE:
      status.state = WALL_FOLLOW_STATE_AFTER_TUNNEL_FIRST_PAUSE;
      break;
    case AFTER_TUNNEL_PHASE_CLOSE_LEFT_WF:
      status.state = WALL_FOLLOW_STATE_AFTER_TUNNEL_CLOSE_LEFT;
      break;
    case AFTER_TUNNEL_PHASE_CHARGE_STOP:
      status.state = WALL_FOLLOW_STATE_AFTER_TUNNEL_CHARGE_STOP;
      status.wallErrorInches = 0.0f;
      status.controlOutputDegrees = 0.0f;
      status.steeringAngle = AFTER_TUNNEL_STRAIGHT_ANGLE;
      status.driveCommand = 0;
      break;
    case AFTER_TUNNEL_PHASE_WALL_FOLLOW_2:
      status.state = WALL_FOLLOW_STATE_AFTER_TUNNEL_WF2;
      break;
    case AFTER_TUNNEL_PHASE_FINAL_LEFT_WF:
      status.state = WALL_FOLLOW_STATE_AFTER_TUNNEL_FINAL_LEFT;
      break;
    case AFTER_TUNNEL_PHASE_BACK_INTO_GAP:
      status.state = WALL_FOLLOW_STATE_AFTER_TUNNEL_BACK;
      status.wallErrorInches = 0.0f;
      status.controlOutputDegrees = (float)(AFTER_TUNNEL_BACK_STEER_ANGLE - AFTER_TUNNEL_STRAIGHT_ANGLE);
      status.steeringAngle = AFTER_TUNNEL_BACK_STEER_ANGLE;
      status.driveCommand = -1;
      if (now - afterTunnelPhaseStartedAtMs >= AFTER_TUNNEL_BACK_INTO_GAP_MS) {
        status.driveCommand = 0;
        control.finished = true;
      }
      break;
    case AFTER_TUNNEL_PHASE_IDLE:
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
  friendsHouseDarkDetectedAtMs = 0;
  friendsHouseDarkTimingActive = false;
  followToTunnelPhase = FOLLOW_TO_TUNNEL_PHASE_IDLE;
  followToTunnelPhaseStartedAtMs = 0;
  followToTunnelStartLdr = 0;
  driveThroughTunnelPhase = DRIVE_THROUGH_TUNNEL_PHASE_IDLE;
  afterTunnelPhase = AFTER_TUNNEL_PHASE_IDLE;
  afterTunnelPhaseStartedAtMs = 0;
  afterTunnelLeftWallCount = 0;
  afterTunnelLeftWallSeenAtMs = 0;
  afterTunnelLeftWallActive = false;
  afterTunnelWall2Armed = false;
  afterTunnelSavedTuning = WallFollowTuning();
  setWallFollowBackupEnabled(true);
}

bool raceStepUsesCustomControl(int stepIndex) {
  return stepIndex == BACK_OUT_OF_GARAGE_STEP_INDEX ||
         stepIndex == FRIENDS_HOUSE_STEP_INDEX ||
         stepIndex == FOLLOW_TO_TUNNEL_STEP_INDEX ||
         stepIndex == DRIVE_THROUGH_TUNNEL_STEP_INDEX ||
         stepIndex == AFTER_TUNNEL_STEP_INDEX;
}

int nextImplementedRaceStepIndex(int stepIndex) {
  switch (stepIndex) {
    case BACK_OUT_OF_GARAGE_STEP_INDEX:
      return FRIENDS_HOUSE_STEP_INDEX;
    case FRIENDS_HOUSE_STEP_INDEX:
      return FOLLOW_TO_TUNNEL_STEP_INDEX;
    case FOLLOW_TO_TUNNEL_STEP_INDEX:
      return DRIVE_THROUGH_TUNNEL_STEP_INDEX;
    case DRIVE_THROUGH_TUNNEL_STEP_INDEX:
      return AFTER_TUNNEL_STEP_INDEX;
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
      friendsHouseDarkDetectedAtMs = 0;
      friendsHouseDarkTimingActive = false;
      break;
    case FOLLOW_TO_TUNNEL_STEP_INDEX:
      setWallFollowBackupEnabled(true);
      setWallFollowSide(WALL_SIDE_LEFT);
      resetWallFollowController();
      followToTunnelPhase = FOLLOW_TO_TUNNEL_PHASE_TURN;
      followToTunnelPhaseStartedAtMs = millis();
      followToTunnelStartLdr = readLDR();
      break;
    case DRIVE_THROUGH_TUNNEL_STEP_INDEX:
      setWallFollowBackupEnabled(false);
      setWallFollowSide(WALL_SIDE_RIGHT);
      resetWallFollowController();
      driveThroughTunnelPhase = DRIVE_THROUGH_TUNNEL_PHASE_FOLLOW_RIGHT_WALL;
      break;
    case AFTER_TUNNEL_STEP_INDEX:
      setWallFollowBackupEnabled(true);
      setWallFollowSide(WALL_SIDE_RIGHT);
      resetWallFollowController();
      afterTunnelPhase = AFTER_TUNNEL_PHASE_EXIT_TURN;
      afterTunnelPhaseStartedAtMs = millis();
      afterTunnelLeftWallCount = 0;
      afterTunnelLeftWallSeenAtMs = 0;
      afterTunnelLeftWallActive = false;
      afterTunnelWall2Armed = false;
      afterTunnelSavedTuning = getWallFollowTuning();
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
    case AFTER_TUNNEL_STEP_INDEX:
      if (afterTunnelPhase == AFTER_TUNNEL_PHASE_IDLE) {
        return RaceStepControl();
      }
      return serviceAfterTunnel();
    default:
      return RaceStepControl();
  }
}
