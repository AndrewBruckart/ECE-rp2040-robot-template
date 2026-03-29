#include "config.h"
#include "encoder.h"
#include "motors.h"
#include "sensors.h"
#include "steering.h"
#include "ui.h"

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
static const int HOME_ITEM_COUNT = 4;
static const int OUTPUT_COUNT = 7;
static const MelodyNote SPEAKER_MELODY[] = {
  {784, 100}, {988, 100}, {1175, 140}, {1568, 180}
};

static ScreenId currentScreen = SCREEN_HOME;
static int homeSelection = 0;
static bool tuneServoSelected = true;
static ButtonTracker encoderSwitch = {false, 0};
static unsigned long lastUiAt = 0;
static bool motorRunning = false;
static int outputSelection = 0;
static bool outputStates[OUTPUT_COUNT] = {false, false, false, false, false, false, false};
static bool melodyPlaying = false;
static int melodyIndex = 0;
static unsigned long nextMelodyAt = 0;
static OutputAutoTest outputAutoTest = {false, 0, 0, 0};

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
    case OUTPUT_BRAKE: digitalWrite(PIN_BRAKE_LIGHT, on ? HIGH : LOW); break;
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

static void stopMotorTest() {
  motorRunning = false;
  motorsBrake();
}

static void startMotorTest() {
  motorRunning = true;
  setMotorRawLeft(1);
  setMotorRawRight(1);
}

static void handleHomeInput(int clicks, PressType press) {
  if (clicks != 0) {
    homeSelection = constrain(homeSelection + (clicks > 0 ? 1 : -1), 0, HOME_ITEM_COUNT - 1);
  }
  if (press == PRESS_SHORT) {
    currentScreen = (ScreenId)(homeSelection + 1);
    if (currentScreen == SCREEN_OUTPUTS) {
      startOutputAutoTest();
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
    tuneServoSelected = !tuneServoSelected;
  } else if (press == PRESS_LONG) {
    stopMotorTest();
    currentScreen = SCREEN_HOME;
    return;
  }

  if (clicks == 0) {
    return;
  }

  int delta = clicks > 0 ? 1 : -1;
  if (tuneServoSelected) {
    setSteeringAngle(getSteeringAngle() + delta);
  } else {
    setMotorSpeedPercent(getMotorSpeedPercent() + delta);
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
    outputSelection = constrain(outputSelection + (clicks > 0 ? 1 : -1), 0, OUTPUT_COUNT - 1);
  }

  if (press == PRESS_SHORT) {
    outputStates[outputSelection] = !outputStates[outputSelection];
    applyOutputState(outputSelection);
  } else if (press == PRESS_LONG) {
    clearAllOutputs();
    currentScreen = SCREEN_HOME;
  }
}

static void handleAboutInput(PressType press) {
  if (press == PRESS_LONG) {
    currentScreen = SCREEN_HOME;
  }
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
      drawTuneScreen(getSteeringAngle(), getMotorSpeedPercent(), tuneServoSelected, motorRunning);
      break;
    case SCREEN_OUTPUTS:
      drawOutputsScreen(outputNameForIndex(outputSelection), outputStates[outputSelection], outputSelection, OUTPUT_COUNT);
      break;
    case SCREEN_ABOUT:
      drawAboutScreen();
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

  initUI();
  initMotors();
  initSteering();
  initSensors();
  initEncoder();

  pinMode(PIN_BTN_START, INPUT);
  pinMode(PIN_BTN_STOP, INPUT);
  pinMode(PIN_ENC_SW, INPUT);

  setMotorSpeedPercent(35);
  setSteeringAngle(90);
  setAccelDataRate(MMA8451_DATARATE_100_HZ);
  clearAllOutputs();
  motorsCoast();
  updateScreen();
}

void loop() {
  serviceSpeakerMelody();
  serviceOutputAutoTest();

  int clicks = consumeEncoderClicks();
  PressType encoderPress = updateButton(encoderSwitch, PIN_ENC_SW);

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
    case SCREEN_ABOUT:
      handleAboutInput(encoderPress);
      break;
  }

  if (currentScreen == SCREEN_TUNE) {
    if (digitalRead(PIN_BTN_START) == HIGH) {
      startMotorTest();
    }
    if (digitalRead(PIN_BTN_STOP) == HIGH) {
      stopMotorTest();
    }
  } else if (motorRunning) {
    stopMotorTest();
  }

  unsigned long now = millis();
  if (now - lastUiAt >= UI_REFRESH_MS) {
    lastUiAt = now;
    updateScreen();
  }
}
