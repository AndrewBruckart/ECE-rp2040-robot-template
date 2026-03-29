#include "motors.h"
#include "config.h"

static int8_t lastLeftCmd = 2;
static int8_t lastRightCmd = 2;
static int lastSpeedPercent = 0;
static bool brakeLightOn = false;

static void setBrakeLight(bool on) {
  if (brakeLightOn == on) {
    return;
  }
  digitalWrite(PIN_BRAKE_LIGHT, on ? HIGH : LOW);
  brakeLightOn = on;
}

static void applyForwardPWM(uint8_t pinA, uint8_t pinB, int percent) {
  int duty = map(percent, 0, 100, 255, 0);
  digitalWrite(pinA, HIGH);
  analogWrite(pinB, duty);
}

static void applyReversePWM(uint8_t pinA, uint8_t pinB, int percent) {
  int duty = map(percent, 0, 100, 255, 0);
  analogWrite(pinA, duty);
  digitalWrite(pinB, HIGH);
}

static void applyMotorPins(uint8_t pinA, uint8_t pinB, int8_t cmd) {
  int percent = constrain(lastSpeedPercent, 0, 100);
  if (cmd > 0) {
    applyForwardPWM(pinA, pinB, percent);
  } else if (cmd < 0) {
    applyReversePWM(pinA, pinB, percent);
  } else {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, LOW);
  }
}

void initMotors() {
  pinMode(PIN_LEFT_A, OUTPUT);
  pinMode(PIN_LEFT_B, OUTPUT);
  pinMode(PIN_RIGHT_A, OUTPUT);
  pinMode(PIN_RIGHT_B, OUTPUT);
  pinMode(PIN_BRAKE_LIGHT, OUTPUT);
  analogWriteRange(255);
  analogWriteFreq(20000);
  motorsCoast();
}

void setMotorRawLeft(int8_t cmd) {
  if (cmd == lastLeftCmd) {
    return;
  }
  applyMotorPins(PIN_LEFT_A, PIN_LEFT_B, cmd);
  lastLeftCmd = cmd;
}

void setMotorRawRight(int8_t cmd) {
  if (cmd == lastRightCmd) {
    return;
  }
  applyMotorPins(PIN_RIGHT_A, PIN_RIGHT_B, cmd);
  lastRightCmd = cmd;
}

void setMotorSpeedPercent(int percent) {
  lastSpeedPercent = constrain(percent, 0, 100);
  if (lastLeftCmd > 0) {
    applyForwardPWM(PIN_LEFT_A, PIN_LEFT_B, lastSpeedPercent);
  } else if (lastLeftCmd < 0) {
    applyReversePWM(PIN_LEFT_A, PIN_LEFT_B, lastSpeedPercent);
  }
  if (lastRightCmd > 0) {
    applyForwardPWM(PIN_RIGHT_A, PIN_RIGHT_B, lastSpeedPercent);
  } else if (lastRightCmd < 0) {
    applyReversePWM(PIN_RIGHT_A, PIN_RIGHT_B, lastSpeedPercent);
  }
}

int getMotorSpeedPercent() {
  return lastSpeedPercent;
}

void motorsBrake() {
  if (lastLeftCmd == 127 && lastRightCmd == 127) {
    setBrakeLight(true);
    return;
  }
  digitalWrite(PIN_LEFT_A, HIGH);
  digitalWrite(PIN_LEFT_B, HIGH);
  digitalWrite(PIN_RIGHT_A, HIGH);
  digitalWrite(PIN_RIGHT_B, HIGH);
  lastLeftCmd = 127;
  lastRightCmd = 127;
  setBrakeLight(true);
}

void motorsCoast() {
  digitalWrite(PIN_LEFT_A, LOW);
  digitalWrite(PIN_LEFT_B, LOW);
  digitalWrite(PIN_RIGHT_A, LOW);
  digitalWrite(PIN_RIGHT_B, LOW);
  lastLeftCmd = 0;
  lastRightCmd = 0;
  setBrakeLight(false);
}
