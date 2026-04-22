#include "steering.h"
#include "config.h"
#include <Servo.h>

static Servo steer;
static int steeringTrimUs = SERVO_CENTER_TRIM_US;
static int steeringAngle = STEERING_STRAIGHT_ANGLE;
static int lastCommandedUs = -1;

static void ensureAttachedOnce() {
  if (!steer.attached()) {
    steer.attach(PIN_STEER_SERVO);
  }
}

static int angleToUs(int angle) {
  angle = constrain(angle, 0, 180);
  int centered = angle - 90;
  int us = map(centered, -90, 90, SERVO_MIN_US, SERVO_MAX_US);
  us += steeringTrimUs;
  return constrain(us, SERVO_MIN_US, SERVO_MAX_US);
}

void initSteering() {
  steer.attach(PIN_STEER_SERVO);
  setSteeringAngle(STEERING_STRAIGHT_ANGLE);
}

void setSteeringUs(int us) {
  ensureAttachedOnce();
  us = constrain(us, SERVO_MIN_US, SERVO_MAX_US);
  if (us == lastCommandedUs) {
    return;
  }
  steer.writeMicroseconds(us);
  lastCommandedUs = us;
}

void centerSteering() {
  setSteeringAngle(STEERING_STRAIGHT_ANGLE);
}

void setSteeringAngle(int angle) {
  steeringAngle = constrain(angle, 0, 180);
  setSteeringUs(angleToUs(steeringAngle));
}

int getSteeringAngle() {
  return steeringAngle;
}
