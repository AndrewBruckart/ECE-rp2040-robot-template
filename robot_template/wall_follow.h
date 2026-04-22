#pragma once

enum WallFollowSide {
  WALL_SIDE_LEFT = 0,
  WALL_SIDE_RIGHT
};

enum WallFollowState {
  WALL_FOLLOW_STATE_TRACKING = 0,
  WALL_FOLLOW_STATE_NO_WALL,
  WALL_FOLLOW_STATE_FRONT_TURN,
  WALL_FOLLOW_STATE_BACKUP,
  WALL_FOLLOW_STATE_GARAGE_BACK_OUT,
  WALL_FOLLOW_STATE_GARAGE_TURN_LEFT,
  WALL_FOLLOW_STATE_STEP_COMPLETE,
  WALL_FOLLOW_STATE_FRIENDS_FIND_WALL,
  WALL_FOLLOW_STATE_FRIENDS_FOLLOW,
  WALL_FOLLOW_STATE_FRIENDS_LOW_LIGHT
};

enum WallFollowMenuItem {
  WALL_MENU_SIDE = 0,
  WALL_MENU_DISTANCE,
  WALL_MENU_KP,
  WALL_MENU_MOTOR_SPEED,
  WALL_MENU_ITEM_COUNT
};

struct WallFollowTuning {
  float targetWallDistanceInches;
  float kp;
  int motorSpeedPercent;
};

struct WallFollowStatus {
  WallFollowSide selectedWall;
  WallFollowState state;
  int centerRawAdc;
  float leftDistanceInches;
  float centerDistanceInches;
  float rightDistanceInches;
  float activeWallDistanceInches;
  float wallErrorInches;
  float controlOutputDegrees;
  int steeringAngle;
  int driveCommand;
};

struct WallFollowScreenData {
  WallFollowStatus status;
  WallFollowTuning tuning;
  bool motorsRunning;
  bool editing;
  int selectedItem;
};

void initWallFollow();
void resetWallFollowController();
WallFollowStatus updateWallFollowControl(float leftDistanceInches, float centerDistanceInches, float rightDistanceInches, int centerRawAdc);

void setWallFollowSide(WallFollowSide side);
WallFollowSide getWallFollowSide();
const char *wallFollowSideName(WallFollowSide side);
const char *wallFollowStateName(WallFollowState state);

WallFollowTuning getWallFollowTuning();
void adjustWallFollowMenuItem(WallFollowMenuItem item, int delta);
float wallFollowNoWallDistanceInches();
