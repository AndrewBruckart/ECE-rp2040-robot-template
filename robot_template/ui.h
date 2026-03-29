#pragma once

enum ScreenId {
  SCREEN_HOME = 0,
  SCREEN_SENSORS,
  SCREEN_TUNE,
  SCREEN_OUTPUTS,
  SCREEN_ABOUT
};

void initUI();
void drawHomeScreen(int selectedIndex);
void drawSensorsScreen(int irL, int irC, int irR, int ldr, int ax, int ay, int az, bool accelReady);
void drawTuneScreen(int servoAngle, int motorPercent, bool servoSelected, bool motorRunning);
void drawOutputsScreen(const char *label, bool outputOn, int index, int count);
void drawAboutScreen();
