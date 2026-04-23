#include "wall_follow.h"
#include "config.h"
#include <Arduino.h>
#include <math.h>

static const int STRAIGHT_STEERING_ANGLE = STEERING_STRAIGHT_ANGLE;
static const int WALL_FOLLOW_SERVO_LIMIT_DEG = 55;
static const float NO_WALL_DISTANCE_INCHES = 14.0f;
static const float FRONT_TURN_DETECT_INCHES = 7.5f;
static const float FRONT_BACKUP_DETECT_INCHES = 4.5f;
static const float FRONT_BACKUP_CLEAR_INCHES = 4.75f;
static const int NO_WALL_LEFT_STEERING_ANGLE = 50;
static const int NO_WALL_RIGHT_STEERING_ANGLE = 130;
static const int FRONT_TURN_STEER_DEG = 38;
static const int BACKUP_STEER_DEG = 50;
static const unsigned long BACKUP_DURATION_MS = 260;
static const unsigned long POST_BACKUP_TURN_MS = 220;

static const float DEFAULT_TARGET_DISTANCE_INCHES = 4.0f;
static const float DEFAULT_LEFT_KP = 12.0f;
static const float DEFAULT_RIGHT_KP = 15.0f;
static const int DEFAULT_MOTOR_SPEED_PERCENT = 100;

static const float TARGET_DISTANCE_STEP = 0.5f;
static const float KP_STEP = 0.5f;
static const int MOTOR_SPEED_STEP = 5;

static WallFollowSide selectedWall = WALL_SIDE_LEFT;
static WallFollowState currentState = WALL_FOLLOW_STATE_TRACKING;
static WallFollowTuning tuning = {
  DEFAULT_TARGET_DISTANCE_INCHES,
  DEFAULT_LEFT_KP,
  DEFAULT_MOTOR_SPEED_PERCENT
};
static unsigned long backupUntilMs = 0;
static unsigned long postBackupTurnUntilMs = 0;
static bool backupArmed = true;
static bool backupEnabled = true;

static float defaultKpForSide(WallFollowSide side) {
  return (side == WALL_SIDE_RIGHT) ? DEFAULT_RIGHT_KP : DEFAULT_LEFT_KP;
}

static float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}

static void normalizeTuning() {
  tuning.targetWallDistanceInches = clampFloat(tuning.targetWallDistanceInches, 1.0f, 12.0f);
  tuning.kp = clampFloat(tuning.kp, 0.0f, 60.0f);
  tuning.motorSpeedPercent = constrain(tuning.motorSpeedPercent, 0, 100);
}

static float computeDistanceError(float wallDistanceInches) {
  return wallDistanceInches - tuning.targetWallDistanceInches;
}

static float computeSteeringOffsetDegrees(float distanceErrorInches) {
  // On this chassis, positive steering angles turn away from the left wall and
  // negative steering angles turn away from the right wall.
  float sideGain = (selectedWall == WALL_SIDE_LEFT) ? -1.0f : 1.0f;
  return sideGain * tuning.kp * distanceErrorInches;
}

static int steeringOffsetToAngle(float steeringOffsetDegrees) {
  int boundedOffset = (int)roundf(clampFloat(steeringOffsetDegrees, -WALL_FOLLOW_SERVO_LIMIT_DEG, WALL_FOLLOW_SERVO_LIMIT_DEG));
  return constrain(STRAIGHT_STEERING_ANGLE + boundedOffset, 0, 180);
}

static float noWallSeekOffsetDegrees() {
  int targetAngle = (selectedWall == WALL_SIDE_LEFT) ? NO_WALL_LEFT_STEERING_ANGLE : NO_WALL_RIGHT_STEERING_ANGLE;
  return (float)(targetAngle - STRAIGHT_STEERING_ANGLE);
}

float wallFollowNoWallDistanceInches() {
  return NO_WALL_DISTANCE_INCHES;
}

void initWallFollow() {
  selectedWall = WALL_SIDE_LEFT;
  currentState = WALL_FOLLOW_STATE_TRACKING;
  tuning.targetWallDistanceInches = DEFAULT_TARGET_DISTANCE_INCHES;
  tuning.kp = defaultKpForSide(selectedWall);
  tuning.motorSpeedPercent = DEFAULT_MOTOR_SPEED_PERCENT;
  backupUntilMs = 0;
  postBackupTurnUntilMs = 0;
  backupArmed = true;
  backupEnabled = true;
}

void resetWallFollowController() {
  currentState = WALL_FOLLOW_STATE_TRACKING;
  backupUntilMs = 0;
  postBackupTurnUntilMs = 0;
  backupArmed = true;
}

WallFollowStatus updateWallFollowControl(float leftDistanceInches, float centerDistanceInches, float rightDistanceInches, int centerRawAdc) {
  normalizeTuning();

  unsigned long now = millis();
  float trackedWallDistance = (selectedWall == WALL_SIDE_LEFT) ? leftDistanceInches : rightDistanceInches;
  float distanceError = computeDistanceError(trackedWallDistance);
  bool wallMissing = trackedWallDistance >= NO_WALL_DISTANCE_INCHES;
  bool frontTooClose = centerDistanceInches <= FRONT_BACKUP_DETECT_INCHES;
  bool frontCorner = centerDistanceInches <= FRONT_TURN_DETECT_INCHES;
  float steeringOffset = computeSteeringOffsetDegrees(distanceError);
  int driveCommand = 1;

  if (!backupEnabled) {
    backupUntilMs = 0;
    postBackupTurnUntilMs = 0;
    backupArmed = false;
  } else if (centerDistanceInches >= FRONT_BACKUP_CLEAR_INCHES) {
    backupArmed = true;
  }

  if (backupEnabled && now < backupUntilMs) {
    currentState = WALL_FOLLOW_STATE_BACKUP;
    steeringOffset = (selectedWall == WALL_SIDE_LEFT) ? -BACKUP_STEER_DEG : BACKUP_STEER_DEG;
    driveCommand = -1;
  } else if (backupEnabled && frontTooClose && backupArmed) {
    currentState = WALL_FOLLOW_STATE_BACKUP;
    backupUntilMs = now + BACKUP_DURATION_MS;
    postBackupTurnUntilMs = backupUntilMs + POST_BACKUP_TURN_MS;
    backupArmed = false;
    steeringOffset = (selectedWall == WALL_SIDE_LEFT) ? -BACKUP_STEER_DEG : BACKUP_STEER_DEG;
    driveCommand = -1;
  } else if (now < postBackupTurnUntilMs || frontCorner) {
    currentState = WALL_FOLLOW_STATE_FRONT_TURN;
    steeringOffset = (selectedWall == WALL_SIDE_LEFT) ? FRONT_TURN_STEER_DEG : -FRONT_TURN_STEER_DEG;
  } else if (wallMissing) {
    currentState = WALL_FOLLOW_STATE_NO_WALL;
    steeringOffset = noWallSeekOffsetDegrees();
  } else {
    currentState = WALL_FOLLOW_STATE_TRACKING;
  }

  WallFollowStatus status = {};
  status.selectedWall = selectedWall;
  status.state = currentState;
  status.centerRawAdc = centerRawAdc;
  status.leftDistanceInches = leftDistanceInches;
  status.centerDistanceInches = centerDistanceInches;
  status.rightDistanceInches = rightDistanceInches;
  status.activeWallDistanceInches = trackedWallDistance;
  status.wallErrorInches = distanceError;
  status.controlOutputDegrees = steeringOffset;
  status.steeringAngle = steeringOffsetToAngle(steeringOffset);
  status.driveCommand = driveCommand;
  return status;
}

void setWallFollowBackupEnabled(bool enabled) {
  backupEnabled = enabled;
  if (!backupEnabled) {
    backupUntilMs = 0;
    postBackupTurnUntilMs = 0;
    backupArmed = false;
  } else {
    backupArmed = true;
  }
}

bool isWallFollowBackupEnabled() {
  return backupEnabled;
}

void setWallFollowSide(WallFollowSide side) {
  bool usingDefaultKp = fabsf(tuning.kp - defaultKpForSide(selectedWall)) < 0.001f;
  selectedWall = side;
  if (usingDefaultKp) {
    tuning.kp = defaultKpForSide(selectedWall);
  }
  resetWallFollowController();
}

WallFollowSide getWallFollowSide() {
  return selectedWall;
}

const char *wallFollowSideName(WallFollowSide side) {
  return (side == WALL_SIDE_RIGHT) ? "RIGHT" : "LEFT";
}

const char *wallFollowStateName(WallFollowState state) {
  switch (state) {
    case WALL_FOLLOW_STATE_NO_WALL: return "SEEK";
    case WALL_FOLLOW_STATE_FRONT_TURN: return "TURN";
    case WALL_FOLLOW_STATE_BACKUP: return "BACK";
    case WALL_FOLLOW_STATE_GARAGE_BACK_OUT: return "GARAGE_BACK";
    case WALL_FOLLOW_STATE_GARAGE_TURN_LEFT: return "GARAGE_TURN";
    case WALL_FOLLOW_STATE_STEP_COMPLETE: return "STEP_DONE";
    case WALL_FOLLOW_STATE_FRIENDS_FIND_WALL: return "FIND_WALL";
    case WALL_FOLLOW_STATE_FRIENDS_FOLLOW: return "FRIENDS_FOLLOW";
    case WALL_FOLLOW_STATE_FRIENDS_LOW_LIGHT: return "LOW_LIGHT";
    case WALL_FOLLOW_STATE_TUNNEL_TURN: return "TUNNEL_TURN";
    case WALL_FOLLOW_STATE_TUNNEL_FIND_WALL: return "TUNNEL_FIND";
    case WALL_FOLLOW_STATE_TUNNEL_FOLLOW: return "TUNNEL_FOLLOW";
    case WALL_FOLLOW_STATE_TUNNEL_STRAIGHT: return "TUNNEL_STRAIGHT";
    case WALL_FOLLOW_STATE_TRACKING:
    default:
      return "TRACK";
  }
}

WallFollowTuning getWallFollowTuning() {
  normalizeTuning();
  return tuning;
}

void adjustWallFollowMenuItem(WallFollowMenuItem item, int delta) {
  if (delta == 0) {
    return;
  }

  switch (item) {
    case WALL_MENU_SIDE:
      setWallFollowSide(delta > 0 ? WALL_SIDE_RIGHT : WALL_SIDE_LEFT);
      break;
    case WALL_MENU_DISTANCE:
      tuning.targetWallDistanceInches += TARGET_DISTANCE_STEP * delta;
      break;
    case WALL_MENU_KP:
      tuning.kp += KP_STEP * delta;
      break;
    case WALL_MENU_MOTOR_SPEED:
      tuning.motorSpeedPercent += MOTOR_SPEED_STEP * delta;
      break;
    default:
      break;
  }

  normalizeTuning();
}
