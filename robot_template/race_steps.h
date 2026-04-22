#pragma once

#include "wall_follow.h"

struct RaceStepControl {
  bool handled;
  bool finished;
  WallFollowStatus status;
  WallFollowTuning tuning;
};

void initRaceSteps();
void resetRaceStepControl();
bool raceStepUsesCustomControl(int stepIndex);
int nextImplementedRaceStepIndex(int stepIndex);
bool raceStepCoastsOnFinish(int stepIndex);
void beginRaceStepControl(int stepIndex);
RaceStepControl serviceRaceStepControl(int stepIndex);
