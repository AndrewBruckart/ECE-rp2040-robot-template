#include "config.h"
#include "encoder.h"
#include "motors.h"
#include "race_logger.h"
#include "race_steps.h"
#include "sensors.h"
#include "steering.h"
#include "ui.h"
#include "wall_follow.h"

enum PressType {
  PRESS_NONE = 0,
  PRESS_SHORT,
  PRESS_LONG
};

enum OutputId {
  OUTPUT_RED = 0,
  OUTPUT_YELLOW,
  OUTPUT_GREEN,
  OUTPUT_LEFT_TURN,
  OUTPUT_RIGHT_TURN,
  OUTPUT_BRAKE,
  OUTPUT_SPEAKER
};

enum AutoMode {
  AUTO_MODE_NONE = 0,
  AUTO_MODE_RACE,
  AUTO_MODE_STEP
};

struct ButtonTracker {
  bool lastState;
  unsigned long pressedAt;
};

struct MelodyNote {
  int freq;
  int duration;
};

struct OutputAutoTest {
  bool active;
  int outputIndex;
  int phase;
  unsigned long nextStepAt;
};

static const unsigned long LONG_PRESS_MS = 900;
static const unsigned long UI_REFRESH_MS = 120;
static const unsigned long OUTPUT_TEST_STEP_MS = 660;
static const unsigned long TURN_SIGNAL_BLINK_MS = 250;
static const unsigned long AUTO_START_DELAY_MS = 1000;
static const int HOME_ITEM_COUNT = 6;
static const int OUTPUT_COUNT = 7;
static const int TUNE_ITEM_COUNT = 3;
static const int STEP_COUNT = 7;
static const MelodyNote SPEAKER_MELODY[] = {
  {784, 100}, {988, 100}, {1175, 140}, {1568, 180}
};
static const char *const STEP_NAMES[STEP_COUNT] = {
  "Back Out of Garage",
  "Friends House",
  "Follow To Tunnel",
  "Drive Through Tunnel",
  "Drive To Charge",
  "Stop At Charge",
  "Back Up Into Church"
};

static ScreenId currentScreen = SCREEN_HOME;
static int homeSelection = 0;
static int tuneSelection = 0;
static bool tuneEditing = false;
static ButtonTracker encoderSwitch = {false, 0};
static unsigned long lastUiAt = 0;
static bool motorRunning = false;
static int outputSelection = 0;
static bool outputStates[OUTPUT_COUNT] = {false, false, false, false, false, false, false};
static bool melodyPlaying = false;
static int melodyIndex = 0;
static unsigned long nextMelodyAt = 0;
static OutputAutoTest outputAutoTest = {false, 0, 0, 0};
static int wallFollowSelection = 0;
static bool wallFollowEditing = false;
static int stepsSelection = 0;
static int activeStepIndex = -1;
static bool blinkPhaseOn = false;
static unsigned long nextBlinkToggleAt = 0;
static WallFollowStatus lastWallFollowStatus = {};
static WallFollowTuning lastAutoTuning = {};
static AutoMode activeAutoMode = AUTO_MODE_NONE;
static bool autoStartPending = false;
static AutoMode pendingAutoMode = AUTO_MODE_NONE;
static int pendingAutoStepIndex = -1;
static unsigned long pendingAutoStartAt = 0;
static bool autoWallSideSaved = false;
static WallFollowSide wallSideBeforeAuto = WALL_SIDE_LEFT;

static WallFollowStatus sampleWallFollowStatus();
static RaceLogMode toRaceLogMode(AutoMode mode);
static int autoStepIndexForMode(AutoMode mode, int requestedStepIndex);
static void startActiveAutoStep();
static bool advanceRaceModeToNextStep();
static bool shouldCoastOnAutoStepFinish();
static void queueAutoModeStart(AutoMode mode, int stepIndex);
static void cancelPendingAutoStart();
static void servicePendingAutoStart();

static PressType updateButton(ButtonTracker &tracker, int pin) {
  bool pressed = digitalRead(pin) == HIGH;
  unsigned long now = millis();
  if (pressed != tracker.lastState) {
    tracker.lastState = pressed;
    if (pressed) {
      tracker.pressedAt = now;
    } else {
      unsigned long held = now - tracker.pressedAt;
      if (held >= LONG_PRESS_MS) {
        return PRESS_LONG;
      }
      return PRESS_SHORT;
    }
  }
  return PRESS_NONE;
}

static void stopSpeakerMelody() {
  noTone(PIN_SPEAKER);
  digitalWrite(PIN_SPEAKER, LOW);
  melodyPlaying = false;
  melodyIndex = 0;
  nextMelodyAt = 0;
  outputStates[OUTPUT_SPEAKER] = false;
}

static void startSpeakerMelody() {
  melodyPlaying = true;
  melodyIndex = 0;
  nextMelodyAt = 0;
  outputStates[OUTPUT_SPEAKER] = true;
}

static void serviceSpeakerMelody() {
  if (!melodyPlaying) {
    return;
  }

  unsigned long now = millis();
  if (now < nextMelodyAt) {
    return;
  }

  if (melodyIndex >= (int)(sizeof(SPEAKER_MELODY) / sizeof(SPEAKER_MELODY[0]))) {
    stopSpeakerMelody();
    return;
  }

  const MelodyNote &note = SPEAKER_MELODY[melodyIndex];
  tone(PIN_SPEAKER, note.freq, note.duration);
  nextMelodyAt = now + note.duration + 30;
  melodyIndex++;
}

static void applyOutputState(int outputId) {
  bool on = outputStates[outputId];
  switch (outputId) {
    case OUTPUT_RED: digitalWrite(PIN_LED_RED, on ? HIGH : LOW); break;
    case OUTPUT_YELLOW: digitalWrite(PIN_LED_YELLOW, on ? HIGH : LOW); break;
    case OUTPUT_GREEN: digitalWrite(PIN_LED_GREEN, on ? HIGH : LOW); break;
    case OUTPUT_LEFT_TURN: digitalWrite(PIN_LEFT_TURN, on ? HIGH : LOW); break;
    case OUTPUT_RIGHT_TURN: digitalWrite(PIN_RIGHT_TURN, on ? HIGH : LOW); break;
    case OUTPUT_BRAKE: setBrakeLightOverride(on); break;
    case OUTPUT_SPEAKER:
      if (on) {
        startSpeakerMelody();
      } else {
        stopSpeakerMelody();
      }
      break;
  }
}

static void clearAllOutputs() {
  for (int i = 0; i < OUTPUT_COUNT; ++i) {
    outputStates[i] = false;
    applyOutputState(i);
  }
}

static void setNonSpeakerOutput(int outputId, bool on) {
  if (outputId < 0 || outputId >= OUTPUT_COUNT || outputId == OUTPUT_SPEAKER) {
    return;
  }
  outputStates[outputId] = on;
  applyOutputState(outputId);
}

static void stopOutputAutoTest() {
  outputAutoTest.active = false;
  clearAllOutputs();
}

static void startOutputAutoTest() {
  stopOutputAutoTest();
  outputAutoTest.active = true;
  outputAutoTest.outputIndex = 0;
  outputAutoTest.phase = 0;
  outputAutoTest.nextStepAt = millis();
  outputSelection = 0;
}

static const char *outputNameForIndex(int index) {
  switch (index) {
    case OUTPUT_RED: return "Red LED";
    case OUTPUT_YELLOW: return "Yellow LED";
    case OUTPUT_GREEN: return "Green LED";
    case OUTPUT_LEFT_TURN: return "Left Turn";
    case OUTPUT_RIGHT_TURN: return "Right Turn";
    case OUTPUT_BRAKE: return "Brake Light";
    case OUTPUT_SPEAKER: return "Speaker";
    default: return "Output";
  }
}

static bool wallFollowDriveActive() {
  return currentScreen == SCREEN_WALL_FOLLOW || activeAutoMode != AUTO_MODE_NONE;
}

static void startWallFollowDrive() {
  setWallFollowBackupEnabled(true);
  resetWallFollowController();
  lastWallFollowStatus = sampleWallFollowStatus();
  lastAutoTuning = getWallFollowTuning();
  setMotorSpeedPercent(lastAutoTuning.motorSpeedPercent);
  setSteeringAngle(lastWallFollowStatus.steeringAngle);
  motorRunning = true;
}

static void startRaceStepDrive(int stepIndex) {
  beginRaceStepControl(stepIndex);
  RaceStepControl stepControl = serviceRaceStepControl(stepIndex);
  lastWallFollowStatus = stepControl.status;
  lastAutoTuning = stepControl.tuning;
  setMotorSpeedPercent(lastAutoTuning.motorSpeedPercent);
  setSteeringAngle(lastWallFollowStatus.steeringAngle);
  motorRunning = true;
}

static RaceLogMode toRaceLogMode(AutoMode mode) {
  switch (mode) {
    case AUTO_MODE_RACE: return RACE_LOG_MODE_RACE;
    case AUTO_MODE_STEP: return RACE_LOG_MODE_STEP;
    case AUTO_MODE_NONE:
    default:
      return RACE_LOG_MODE_NONE;
  }
}

static int autoStepIndexForMode(AutoMode mode, int requestedStepIndex) {
  switch (mode) {
    case AUTO_MODE_RACE:
      return 0;
    case AUTO_MODE_STEP:
      return constrain(requestedStepIndex, 0, STEP_COUNT - 1);
    case AUTO_MODE_NONE:
    default:
      return -1;
  }
}

static void startActiveAutoStep() {
  if (raceStepUsesCustomControl(activeStepIndex)) {
    startRaceStepDrive(activeStepIndex);
    return;
  }

  setWallFollowSide(WALL_SIDE_RIGHT);
  startWallFollowDrive();
}

static void beginAutoMode(AutoMode mode, int stepIndex) {
  int autoStepIndex = autoStepIndexForMode(mode, stepIndex);
  if (!autoWallSideSaved) {
    wallSideBeforeAuto = getWallFollowSide();
    autoWallSideSaved = true;
  }

  activeAutoMode = mode;
  activeStepIndex = autoStepIndex;
  startActiveAutoStep();
  beginRaceLog(toRaceLogMode(mode), activeStepIndex, lastWallFollowStatus, lastAutoTuning, motorRunning);
}

static void queueAutoModeStart(AutoMode mode, int stepIndex) {
  if (activeAutoMode != AUTO_MODE_NONE || autoStartPending) {
    return;
  }

  autoStartPending = true;
  pendingAutoMode = mode;
  pendingAutoStepIndex = stepIndex;
  pendingAutoStartAt = millis() + AUTO_START_DELAY_MS;
}

static void cancelPendingAutoStart() {
  autoStartPending = false;
  pendingAutoMode = AUTO_MODE_NONE;
  pendingAutoStepIndex = -1;
  pendingAutoStartAt = 0;
}

static void servicePendingAutoStart() {
  if (!autoStartPending || activeAutoMode != AUTO_MODE_NONE) {
    return;
  }

  if (millis() < pendingAutoStartAt) {
    return;
  }

  AutoMode mode = pendingAutoMode;
  int stepIndex = pendingAutoStepIndex;
  cancelPendingAutoStart();
  beginAutoMode(mode, stepIndex);
}

static void stopAutoMode() {
  activeAutoMode = AUTO_MODE_NONE;
  activeStepIndex = -1;
  resetRaceStepControl();
  if (autoWallSideSaved) {
    setWallFollowSide(wallSideBeforeAuto);
    autoWallSideSaved = false;
  }
}

static void stopMotorTest(RaceLogStopReason reason = RACE_LOG_STOP_REASON_MODE_CHANGED, bool coast = false) {
  bool autoModeWasActive = activeAutoMode != AUTO_MODE_NONE;
  WallFollowStatus finalStatus = lastWallFollowStatus;
  WallFollowTuning finalTuning = autoModeWasActive ? lastAutoTuning : getWallFollowTuning();

  motorRunning = false;
  if (autoModeWasActive) {
    stopAutoMode();
  } else {
    resetWallFollowController();
  }
  if (coast) {
    motorsCoast();
  } else {
    motorsBrake();
  }
  lastWallFollowStatus = sampleWallFollowStatus();
  if (autoModeWasActive) {
    endRaceLog(finalStatus, finalTuning, false, reason);
  }
}

static void startMotorTest() {
  if (currentScreen == SCREEN_WALL_FOLLOW) {
    if (!motorRunning) {
      startWallFollowDrive();
    }
    return;
  }

  motorRunning = true;
  setMotorRawLeft(1);
  setMotorRawRight(1);
}

static WallFollowStatus sampleWallFollowStatus() {
  float leftDistance = readIRDistanceInches(IR_SENSOR_LEFT);
  float centerDistance = readIRDistanceInches(IR_SENSOR_CENTER);
  float rightDistance = readIRDistanceInches(IR_SENSOR_RIGHT);
  int centerRaw = readIRCenter();
  return updateWallFollowControl(leftDistance, centerDistance, rightDistance, centerRaw);
}

static void handleHomeInput(int clicks, PressType press) {
  if (clicks != 0) {
    homeSelection = constrain(homeSelection + clicks, 0, HOME_ITEM_COUNT - 1);
  }
  if (press == PRESS_SHORT) {
    currentScreen = (ScreenId)(homeSelection + 1);
    if (currentScreen == SCREEN_TUNE) {
      tuneSelection = 0;
      tuneEditing = false;
    } else if (currentScreen == SCREEN_OUTPUTS) {
      startOutputAutoTest();
    } else if (currentScreen == SCREEN_WALL_FOLLOW) {
      wallFollowEditing = false;
      wallFollowSelection = 0;
      resetWallFollowController();
      lastWallFollowStatus = sampleWallFollowStatus();
    } else if (currentScreen == SCREEN_RUN_RACE) {
      activeStepIndex = -1;
      lastWallFollowStatus = sampleWallFollowStatus();
    } else if (currentScreen == SCREEN_STEPS) {
      activeStepIndex = -1;
      lastWallFollowStatus = sampleWallFollowStatus();
    }
  }
}

static void handleSensorsInput(PressType press) {
  if (press == PRESS_LONG) {
    currentScreen = SCREEN_HOME;
  }
}

static void handleTuneInput(int clicks, PressType press) {
  if (press == PRESS_SHORT) {
    if (tuneSelection == 2) {
      if (motorRunning) {
        stopMotorTest();
      } else {
        startMotorTest();
      }
    } else {
      tuneEditing = !tuneEditing;
    }
  } else if (press == PRESS_LONG) {
    tuneEditing = false;
    stopMotorTest();
    currentScreen = SCREEN_HOME;
    return;
  }

  if (clicks == 0) {
    return;
  }

  if (!tuneEditing) {
    tuneSelection = constrain(tuneSelection + clicks, 0, TUNE_ITEM_COUNT - 1);
    return;
  }

  if (tuneSelection == 0) {
    setSteeringAngle(getSteeringAngle() + clicks);
  } else {
    setMotorSpeedPercent(getMotorSpeedPercent() + clicks);
  }
}

static void handleOutputsInput(int clicks, PressType press) {
  if (outputAutoTest.active) {
    if (press == PRESS_LONG) {
      stopOutputAutoTest();
      currentScreen = SCREEN_HOME;
    }
    return;
  }

  if (clicks != 0) {
    outputSelection = constrain(outputSelection + clicks, 0, OUTPUT_COUNT - 1);
  }

  if (press == PRESS_SHORT) {
    outputStates[outputSelection] = !outputStates[outputSelection];
    applyOutputState(outputSelection);
  } else if (press == PRESS_LONG) {
    clearAllOutputs();
    currentScreen = SCREEN_HOME;
  }
}

static void handleWallFollowInput(int clicks, PressType press) {
  if (press == PRESS_SHORT) {
    wallFollowEditing = !wallFollowEditing;
  } else if (press == PRESS_LONG) {
    wallFollowEditing = false;
    stopMotorTest();
    currentScreen = SCREEN_HOME;
    return;
  }

  if (clicks == 0) {
    return;
  }

  if (wallFollowEditing) {
    adjustWallFollowMenuItem((WallFollowMenuItem)wallFollowSelection, clicks);
    lastWallFollowStatus = sampleWallFollowStatus();
  } else {
    wallFollowSelection = constrain(wallFollowSelection + clicks, 0, WALL_MENU_ITEM_COUNT - 1);
  }
}

static void handleRunRaceInput(PressType press) {
  if (press == PRESS_LONG) {
    cancelPendingAutoStart();
    stopMotorTest(RACE_LOG_STOP_REASON_EXIT_SCREEN);
    currentScreen = SCREEN_HOME;
  }
}

static void handleStepsInput(int clicks, PressType press) {
  if (press == PRESS_LONG) {
    cancelPendingAutoStart();
    stopMotorTest(RACE_LOG_STOP_REASON_EXIT_SCREEN);
    currentScreen = SCREEN_HOME;
    return;
  }

  if (activeAutoMode != AUTO_MODE_NONE || clicks == 0) {
    return;
  }

  stepsSelection = constrain(stepsSelection + clicks, 0, STEP_COUNT - 1);
}

static void updateBlinkPhase() {
  unsigned long now = millis();
  if (now >= nextBlinkToggleAt) {
    blinkPhaseOn = !blinkPhaseOn;
    nextBlinkToggleAt = now + TURN_SIGNAL_BLINK_MS;
  }
}

static void applyWallFollowIndicators(const WallFollowStatus &status) {
  bool stopped = !motorRunning;
  bool tracking = motorRunning && status.state == WALL_FOLLOW_STATE_TRACKING;
  bool noWall = motorRunning && status.state == WALL_FOLLOW_STATE_NO_WALL;
  bool turning = motorRunning && status.state == WALL_FOLLOW_STATE_FRONT_TURN;
  bool backing = motorRunning && status.state == WALL_FOLLOW_STATE_BACKUP;
  bool blinkLeft = (noWall && status.selectedWall == WALL_SIDE_LEFT && blinkPhaseOn) ||
    (turning && status.selectedWall == WALL_SIDE_RIGHT && blinkPhaseOn);
  bool blinkRight = (noWall && status.selectedWall == WALL_SIDE_RIGHT && blinkPhaseOn) ||
    (turning && status.selectedWall == WALL_SIDE_LEFT && blinkPhaseOn);

  setNonSpeakerOutput(OUTPUT_GREEN, tracking || turning);
  setNonSpeakerOutput(OUTPUT_YELLOW, backing);
  setNonSpeakerOutput(OUTPUT_LEFT_TURN, blinkLeft);
  setNonSpeakerOutput(OUTPUT_RIGHT_TURN, blinkRight);
  setBrakeLightOverride(stopped || backing);
}

static void clearWallFollowIndicators() {
  clearBrakeLightOverride();
  setNonSpeakerOutput(OUTPUT_GREEN, false);
  setNonSpeakerOutput(OUTPUT_YELLOW, false);
  setNonSpeakerOutput(OUTPUT_LEFT_TURN, false);
  setNonSpeakerOutput(OUTPUT_RIGHT_TURN, false);
}

static bool advanceRaceModeToNextStep() {
  if (activeAutoMode != AUTO_MODE_RACE) {
    return false;
  }

  int nextStepIndex = nextImplementedRaceStepIndex(activeStepIndex);
  if (nextStepIndex < 0 || nextStepIndex >= STEP_COUNT) {
    return false;
  }

  activeStepIndex = nextStepIndex;
  startActiveAutoStep();
  setRaceLogStepIndex(activeStepIndex);
  return true;
}

static bool shouldCoastOnAutoStepFinish() {
  return raceStepCoastsOnFinish(activeStepIndex);
}

static void applyAutoDriveCommand(const WallFollowStatus &status, const WallFollowTuning &tuning, bool useWallFollowIndicators) {
  lastWallFollowStatus = status;
  lastAutoTuning = tuning;
  setSteeringAngle(lastWallFollowStatus.steeringAngle);
  setMotorSpeedPercent(tuning.motorSpeedPercent);
  if (motorRunning) {
    setMotorRawLeft(lastWallFollowStatus.driveCommand);
    setMotorRawRight(lastWallFollowStatus.driveCommand);
  } else {
    motorsBrake();
  }
  if (useWallFollowIndicators) {
    applyWallFollowIndicators(lastWallFollowStatus);
  } else {
    clearWallFollowIndicators();
  }
  serviceRaceLog(lastWallFollowStatus, tuning, motorRunning);
}

static void serviceWallFollow() {
  if (!wallFollowDriveActive()) {
    return;
  }

  updateBlinkPhase();
  WallFollowStatus status = sampleWallFollowStatus();
  WallFollowTuning tuning = getWallFollowTuning();
  applyAutoDriveCommand(status, tuning, true);
}

static void serviceActiveAutoMode() {
  if (activeAutoMode == AUTO_MODE_NONE || activeStepIndex < 0) {
    return;
  }

  if (raceStepUsesCustomControl(activeStepIndex)) {
    RaceStepControl stepControl = serviceRaceStepControl(activeStepIndex);
    if (!stepControl.handled) {
      return;
    }
    applyAutoDriveCommand(stepControl.status, stepControl.tuning, false);
    if (stepControl.finished) {
      if (advanceRaceModeToNextStep()) {
        applyAutoDriveCommand(lastWallFollowStatus, lastAutoTuning, false);
      } else {
        stopMotorTest(RACE_LOG_STOP_REASON_COMPLETE, shouldCoastOnAutoStepFinish());
      }
    }
    return;
  }

  serviceWallFollow();
}

static void updateScreen() {
  int irL = readIRLeft();
  int irC = readIRCenter();
  int irR = readIRRight();
  int ldr = readLDR();
  int ax = 0;
  int ay = 0;
  int az = 0;
  bool accelReady = accelAvailable();
  if (accelReady) {
    readAccelXYZ(ax, ay, az);
  }

  switch (currentScreen) {
    case SCREEN_HOME:
      drawHomeScreen(homeSelection);
      break;
    case SCREEN_SENSORS:
      drawSensorsScreen(irL, irC, irR, ldr, ax, ay, az, accelReady);
      break;
    case SCREEN_TUNE:
      drawTuneScreen(getSteeringAngle(), getMotorSpeedPercent(), motorRunning, tuneSelection, tuneEditing);
      break;
    case SCREEN_OUTPUTS:
      drawOutputsScreen(outputNameForIndex(outputSelection), outputStates[outputSelection], outputSelection, OUTPUT_COUNT);
      break;
    case SCREEN_WALL_FOLLOW: {
      WallFollowScreenData wallScreen = {};
      wallScreen.status = lastWallFollowStatus;
      wallScreen.tuning = getWallFollowTuning();
      wallScreen.motorsRunning = motorRunning;
      wallScreen.editing = wallFollowEditing;
      wallScreen.selectedItem = wallFollowSelection;
      drawWallFollowScreen(wallScreen);
      break;
    }
    case SCREEN_RUN_RACE:
      drawRunRaceScreen(activeAutoMode == AUTO_MODE_RACE && motorRunning, lastWallFollowStatus);
      break;
    case SCREEN_STEPS:
      drawStepsScreen(
        STEP_NAMES,
        STEP_COUNT,
        (activeAutoMode == AUTO_MODE_STEP && activeStepIndex >= 0) ? activeStepIndex : stepsSelection,
        activeAutoMode == AUTO_MODE_STEP && motorRunning
      );
      break;
  }
}

static void serviceOutputAutoTest() {
  if (!outputAutoTest.active) {
    return;
  }

  unsigned long now = millis();
  if (now < outputAutoTest.nextStepAt) {
    return;
  }

  outputSelection = outputAutoTest.outputIndex;

  switch (outputAutoTest.phase) {
    case 0:
      outputStates[outputSelection] = true;
      applyOutputState(outputSelection);
      outputAutoTest.phase = 1;
      outputAutoTest.nextStepAt = now + OUTPUT_TEST_STEP_MS;
      break;
    case 1:
      outputStates[outputSelection] = false;
      applyOutputState(outputSelection);
      outputAutoTest.phase = 2;
      outputAutoTest.nextStepAt = now + OUTPUT_TEST_STEP_MS;
      break;
    case 2:
      outputStates[outputSelection] = true;
      applyOutputState(outputSelection);
      outputAutoTest.phase = 3;
      outputAutoTest.nextStepAt = now + OUTPUT_TEST_STEP_MS;
      break;
    default:
      outputStates[outputSelection] = false;
      applyOutputState(outputSelection);
      outputAutoTest.outputIndex++;
      outputAutoTest.phase = 0;
      if (outputAutoTest.outputIndex >= OUTPUT_COUNT) {
        stopOutputAutoTest();
      } else {
        outputAutoTest.nextStepAt = now + OUTPUT_TEST_STEP_MS;
      }
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(50);

  initRaceLogger();
  initUI();
  initMotors();
  initSteering();
  initSensors();
  initEncoder();
  initWallFollow();
  initRaceSteps();

  pinMode(PIN_BTN_START, INPUT);
  pinMode(PIN_BTN_STOP, INPUT);
  pinMode(PIN_ENC_SW, INPUT);
  encoderSwitch.lastState = (digitalRead(PIN_ENC_SW) == HIGH);
  encoderSwitch.pressedAt = millis();

  setMotorSpeedPercent(35);
  centerSteering();
  setAccelDataRate(MMA8451_DATARATE_100_HZ);
  clearAllOutputs();
  motorsCoast();
  lastAutoTuning = getWallFollowTuning();
  lastWallFollowStatus = sampleWallFollowStatus();
  updateScreen();
}

void loop() {
  serviceRaceLoggerSerial();
  serviceSpeakerMelody();
  serviceOutputAutoTest();
  servicePendingAutoStart();

  int clicks = consumeEncoderClicks();
  PressType encoderPress = updateButton(encoderSwitch, PIN_ENC_SW);
  bool startPressed = digitalRead(PIN_BTN_START) == HIGH;
  bool stopPressed = digitalRead(PIN_BTN_STOP) == HIGH;
  static bool lastStartPressed = false;
  bool startPressedEdge = startPressed && !lastStartPressed;

  switch (currentScreen) {
    case SCREEN_HOME:
      handleHomeInput(clicks, encoderPress);
      break;
    case SCREEN_SENSORS:
      handleSensorsInput(encoderPress);
      break;
    case SCREEN_TUNE:
      handleTuneInput(clicks, encoderPress);
      break;
    case SCREEN_OUTPUTS:
      handleOutputsInput(clicks, encoderPress);
      break;
    case SCREEN_WALL_FOLLOW:
      handleWallFollowInput(clicks, encoderPress);
      break;
    case SCREEN_RUN_RACE:
      handleRunRaceInput(encoderPress);
      break;
    case SCREEN_STEPS:
      handleStepsInput(clicks, encoderPress);
      break;
  }

  if (currentScreen == SCREEN_TUNE) {
    if (startPressed) {
      startMotorTest();
    }
    if (stopPressed) {
      stopMotorTest();
    }
    clearWallFollowIndicators();
  } else if (currentScreen == SCREEN_WALL_FOLLOW) {
    if (startPressed) {
      startMotorTest();
    }
    if (stopPressed) {
      stopMotorTest();
    }
    serviceWallFollow();
  } else if (currentScreen == SCREEN_RUN_RACE) {
    if (startPressedEdge && activeAutoMode == AUTO_MODE_NONE) {
      queueAutoModeStart(AUTO_MODE_RACE, -1);
    }
    if (stopPressed) {
      if (autoStartPending) {
        cancelPendingAutoStart();
      } else {
        stopMotorTest(RACE_LOG_STOP_REASON_STOP_BUTTON);
      }
    }
    if (activeAutoMode != AUTO_MODE_NONE) {
      serviceActiveAutoMode();
    } else {
      clearWallFollowIndicators();
    }
  } else if (currentScreen == SCREEN_STEPS) {
    if (startPressedEdge && activeAutoMode == AUTO_MODE_NONE) {
      queueAutoModeStart(AUTO_MODE_STEP, stepsSelection);
    }
    if (stopPressed) {
      if (autoStartPending) {
        cancelPendingAutoStart();
      } else {
        stopMotorTest(RACE_LOG_STOP_REASON_STOP_BUTTON);
      }
    }
    if (activeAutoMode != AUTO_MODE_NONE) {
      serviceActiveAutoMode();
    } else {
      clearWallFollowIndicators();
    }
  } else if (motorRunning) {
    stopMotorTest();
    if (currentScreen != SCREEN_OUTPUTS) {
      clearWallFollowIndicators();
    }
  } else if (currentScreen != SCREEN_OUTPUTS) {
    clearWallFollowIndicators();
  }

  unsigned long now = millis();
  if (now - lastUiAt >= UI_REFRESH_MS) {
    lastUiAt = now;
    updateScreen();
  }

  lastStartPressed = startPressed;
}
