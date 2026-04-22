#pragma once

#include "wall_follow.h"

enum RaceLogMode {
  RACE_LOG_MODE_NONE = 0,
  RACE_LOG_MODE_RACE,
  RACE_LOG_MODE_STEP
};

enum RaceLogStopReason {
  RACE_LOG_STOP_REASON_NONE = 0,
  RACE_LOG_STOP_REASON_STOP_BUTTON,
  RACE_LOG_STOP_REASON_EXIT_SCREEN,
  RACE_LOG_STOP_REASON_MODE_CHANGED,
  RACE_LOG_STOP_REASON_COMPLETE
};

void initRaceLogger();
bool raceLoggerReady();

void beginRaceLog(RaceLogMode mode, int stepIndex, const WallFollowStatus &status, const WallFollowTuning &tuning, bool motorRunning);
void serviceRaceLog(const WallFollowStatus &status, const WallFollowTuning &tuning, bool motorRunning);
void endRaceLog(const WallFollowStatus &status, const WallFollowTuning &tuning, bool motorRunning, RaceLogStopReason reason);
void setRaceLogStepIndex(int stepIndex);

void serviceRaceLoggerSerial();
