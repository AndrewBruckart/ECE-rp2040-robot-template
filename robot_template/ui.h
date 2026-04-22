#pragma once

#include "wall_follow.h"

enum ScreenId {
  SCREEN_HOME = 0,
  SCREEN_SENSORS,
  SCREEN_TUNE,
  SCREEN_OUTPUTS,
  SCREEN_WALL_FOLLOW,
  SCREEN_RUN_RACE,
  SCREEN_STEPS
};

void initUI();
void drawHomeScreen(int selectedIndex);
void drawSensorsScreen(int irL, int irC, int irR, int ldr, int ax, int ay, int az, bool accelReady);
void drawTuneScreen(int servoAngle, int motorPercent, bool motorRunning, int selectedItem, bool editing);
void drawOutputsScreen(const char *label, bool outputOn, int index, int count);
void drawWallFollowScreen(const WallFollowScreenData &screenData);
void drawRunRaceScreen(bool running, const WallFollowStatus &status);
void drawStepsScreen(const char *const *stepNames, int stepCount, int selectedIndex, bool running);
