#pragma once

enum ScreenId {
  SCREEN_HOME = 0,
  SCREEN_SENSORS,
  SCREEN_TUNE,
  SCREEN_OUTPUTS,
  SCREEN_WALL_FOLLOW   // replaces SCREEN_ABOUT (Quick Help)
};

void initUI();
void drawHomeScreen(int selectedIndex);
void drawSensorsScreen(int irL, int irC, int irR, int ldr, int ax, int ay, int az, bool accelReady);
void drawTuneScreen(int servoAngle, int motorPercent, bool servoSelected, bool motorRunning);
void drawOutputsScreen(const char *label, bool outputOn, int index, int count);
void drawWallFollowScreen(const char *wallName, const char *stateName,
                          int paramSelect,
                          float frontDet, float frontClr, float target,
                          float kp, int speed);
