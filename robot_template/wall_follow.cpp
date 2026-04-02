/*
 * wall_follow.cpp  –  Wall-following finite state machine
 *
 * State machine overview
 * ─────────────────────
 *
 *                     motors start
 *    ┌─────────┐  ─────────────────▶  ┌─────────────┐
 *    │  IDLE   │                       │  FOLLOWING  │◀──┐
 *    └─────────┘  ◀─────────────────  └─────────────┘   │
 *         ▲           motors stop            │           │
 *         │                                  │ front     │ front
 *         │       motors stop                │ < obstThr │ > clearThr
 *         │   ◀────────────────────          ▼           │
 *         └──────────────────────── ┌──────────────────┐ │
 *                                   │   AVOID_FRONT    │─┘
 *                                   └──────────────────┘
 *
 * FOLLOWING:   reads the wall-side IR sensor, converts ADC → inches using
 *              the per-sensor calibration table, then drives
 *                  angle = 90 ± Kp × (measured_dist − target_dist)
 *              (+ for RIGHT wall following, − for LEFT wall following).
 *
 * AVOID_FRONT: steers hard away from the followed wall (0° for RIGHT,
 *              180° for LEFT) and blinks the matching turn signal until
 *              the center sensor reports the path is clear.
 *
 * Sensor calibration
 * ──────────────────
 * Raw ADC values are non-linear.  Each sensor has its own look-up table
 * (distance_inches, adc_count).  adcToDistance() interpolates linearly
 * between the two bracketing entries.  The "NoWall" row is mapped to
 * 15 inches; readings at or above that ADC value return 15 in.
 *
 * Do NOT compare raw ADC readings from different sensors directly.
 */

#include "wall_follow.h"
#include "config.h"
#include "sensors.h"
#include "steering.h"
#include "motors.h"
#include <Arduino.h>

// ═══════════════════════════════════════════════════════════════════════
//  Tuning parameter defaults (all adjustable from the Wall Follow screen)
// ═══════════════════════════════════════════════════════════════════════
WallSide wfWallSide         = WALL_RIGHT;
float    wfTargetDistance   = 3.0f;   // inches – target distance from wall
float    wfFrontObstThresh  = 8.0f;   // inches – enter avoidance below this
float    wfFrontClearThresh = 12.0f;  // inches – exit avoidance above this
float    wfKp               = 15.0f;  // degrees per inch of error
int      wfMotorSpeed       = 75;     // motor speed percent (initial 75%)

// ═══════════════════════════════════════════════════════════════════════
//  Internal state
// ═══════════════════════════════════════════════════════════════════════
static WfState        wfState      = WF_IDLE;
static bool           lastMotors   = false;
static unsigned long  nextBlinkAt  = 0;
static bool           blinkOn      = false;

// ═══════════════════════════════════════════════════════════════════════
//  Calibration tables  (distance inches → ADC)
//  ADC increases monotonically with distance for all three sensors.
//  "NoWall" is assigned 15 inches; extrapolation beyond table limits
//  is clamped to the nearest end value.
// ═══════════════════════════════════════════════════════════════════════

struct CalEntry { float distIn; int adc; };

// Center sensor (GP A2 / PIN_IR_CENTER) – 0–14 in + NoWall
static const CalEntry centerCal[] = {
  {0,   44}, {1,   53}, {2,   62}, {3,   72}, {4,   93},
  {5,  243}, {6,  412}, {7,  532}, {8,  615}, {9,  673},
  {10, 712}, {11, 760}, {12, 792}, {13, 812}, {14, 832},
  {15, 933}  // NoWall → treated as 15 in
};
static const int CENTER_N = (int)(sizeof(centerCal) / sizeof(centerCal[0]));

// Left sensor (GP A3 / PIN_IR_LEFT) – 0–10 in + NoWall
static const CalEntry leftCal[] = {
  {0,   47}, {1,   65}, {2,  196}, {3,  504}, {4,  686},
  {5,  762}, {6,  812}, {7,  866}, {8,  885}, {9,  895},
  {10, 911},
  {15, 957}  // NoWall → 15 in
};
static const int LEFT_N = (int)(sizeof(leftCal) / sizeof(leftCal[0]));

// Right sensor (GP A1 / PIN_IR_RIGHT) – 0–10 in + NoWall
static const CalEntry rightCal[] = {
  {0,   48}, {1,   63}, {2,  311}, {3,  615}, {4,  782},
  {5,  828}, {6,  864}, {7,  884}, {8,  895}, {9,  908},
  {10, 914},
  {15, 951}  // NoWall → 15 in
};
static const int RIGHT_N = (int)(sizeof(rightCal) / sizeof(rightCal[0]));

// ── Linear interpolation between calibration points ─────────────────────────
// Returns distance in inches corresponding to the given ADC reading.
// Clamps to the table's minimum/maximum distance when out of range.
static float adcToDistance(int adc, const CalEntry *table, int n) {
  if (adc <= table[0].adc)         return table[0].distIn;
  if (adc >= table[n - 1].adc)     return table[n - 1].distIn;
  for (int i = 1; i < n; ++i) {
    if (adc <= table[i].adc) {
      float t = (float)(adc - table[i - 1].adc)
              / (float)(table[i].adc - table[i - 1].adc);
      return table[i - 1].distIn + t * (table[i].distIn - table[i - 1].distIn);
    }
  }
  return table[n - 1].distIn;
}

// Dispatch to the correct calibration for the active wall side sensor
static float wallSensorDistance(int adc) {
  if (wfWallSide == WALL_LEFT)
    return adcToDistance(adc, leftCal,  LEFT_N);
  else
    return adcToDistance(adc, rightCal, RIGHT_N);
}

// Always use center calibration for front obstacle detection
static float frontSensorDistance(int adc) {
  return adcToDistance(adc, centerCal, CENTER_N);
}

// ═══════════════════════════════════════════════════════════════════════
//  Proportional steering controller
// ═══════════════════════════════════════════════════════════════════════
//
//  error      = measuredDist − targetDist  (positive = too far from wall)
//  correction = Kp × error               (degrees; positive = steer toward wall)
//
//  RIGHT wall: steer right (increase angle) when too far → angle = 90 + correction
//  LEFT  wall: steer left  (decrease angle) when too far → angle = 90 − correction
//
static int computeFollowSteering(float dist) {
  float error      = dist - wfTargetDistance;
  float correction = wfKp * error;

  int angle;
  if (wfWallSide == WALL_RIGHT) {
    angle = 90 + (int)correction;   // steer right if too far right
  } else {
    angle = 90 - (int)correction;   // steer left  if too far left
  }
  return constrain(angle, 0, 180);
}

// ═══════════════════════════════════════════════════════════════════════
//  LED helpers  (direct GPIO – do not use outputStates[] array here)
// ═══════════════════════════════════════════════════════════════════════

static void setGreenLed(bool on) {
  digitalWrite(PIN_LED_GREEN, on ? HIGH : LOW);
}

static void clearTurnSignals() {
  digitalWrite(PIN_LEFT_TURN,  LOW);
  digitalWrite(PIN_RIGHT_TURN, LOW);
}

// Non-blocking blink for the turn signal on the side we are avoiding toward.
// Call every loop iteration while in WF_AVOID_FRONT.
static void serviceAvoidBlink() {
  unsigned long now = millis();
  if (now >= nextBlinkAt) {
    blinkOn     = !blinkOn;
    nextBlinkAt = now + 400UL;   // 400 ms on / 400 ms off
    // Steer away from the followed wall, so blink that direction's signal
    int blinkPin = (wfWallSide == WALL_RIGHT) ? PIN_LEFT_TURN : PIN_RIGHT_TURN;
    digitalWrite(blinkPin, blinkOn ? HIGH : LOW);
  }
}

// ═══════════════════════════════════════════════════════════════════════
//  Public interface
// ═══════════════════════════════════════════════════════════════════════

void initWallFollow() {
  wfState    = WF_IDLE;
  lastMotors = false;
  setGreenLed(false);
  clearTurnSignals();
}

// Main state machine – call once per main-loop iteration from robot_template.ino.
// motorsRunning: true when the motors have been started via the START button.
void runWallFollow(bool motorsRunning) {

  // ── Detect motor start / stop transitions ────────────────────────────────
  if (motorsRunning != lastMotors) {
    lastMotors = motorsRunning;

    if (motorsRunning) {
      // Transition any state → WF_FOLLOWING
      wfState = WF_FOLLOWING;
      setMotorSpeedPercent(wfMotorSpeed);  // apply tuned speed
      setBrakeLightOverride(false);         // release brake light to motor driver
      // Motor code left brake light ON from previous motorsBrake(); clear it now
      // setBrakeLightOverride(false) already drove the pin LOW
      setGreenLed(true);
      clearTurnSignals();
    } else {
      // Transition any state → WF_IDLE
      wfState = WF_IDLE;
      setGreenLed(false);
      clearTurnSignals();
      return;
    }
  }

  if (!motorsRunning) {
    return;   // Nothing to do while idle
  }

  // ── Read sensors ──────────────────────────────────────────────────────────
  int wallAdc  = (wfWallSide == WALL_RIGHT) ? readIRRight() : readIRLeft();
  int frontAdc = readIRCenter();

  float wallDist  = wallSensorDistance(wallAdc);
  float frontDist = frontSensorDistance(frontAdc);

  // ── FSM transitions and control outputs ──────────────────────────────────
  switch (wfState) {

    // ── Normal proportional wall following ──────────────────────────────────
    case WF_FOLLOWING:
      if (frontDist < wfFrontObstThresh) {
        // Front obstacle detected – enter avoidance state
        wfState     = WF_AVOID_FRONT;
        blinkOn     = false;
        nextBlinkAt = 0;
        setGreenLed(false);
        clearTurnSignals();
        // Immediately command hard-away steering (also applied in AVOID case below)
      } else {
        // Proportional steering toward target distance
        setSteeringAngle(computeFollowSteering(wallDist));
      }
      break;

    // ── Front obstacle avoidance ─────────────────────────────────────────────
    case WF_AVOID_FRONT:
      // Steer as hard as possible away from the followed wall
      if (wfWallSide == WALL_RIGHT) {
        setSteeringAngle(0);    // hard left  (away from right wall)
      } else {
        setSteeringAngle(180);  // hard right (away from left wall)
      }

      // Blink the appropriate turn signal (non-blocking)
      serviceAvoidBlink();

      // Check for path-clear exit condition
      if (frontDist > wfFrontClearThresh) {
        wfState = WF_FOLLOWING;
        setGreenLed(true);
        clearTurnSignals();
      }
      break;

    case WF_IDLE:
    default:
      break;
  }
}

// ── State query helpers ─────────────────────────────────────────────────────
WfState getWfState() { return wfState; }

const char *wfStateName() {
  switch (wfState) {
    case WF_FOLLOWING:   return "FOLLOW";
    case WF_AVOID_FRONT: return "AVOID ";
    default:             return "IDLE  ";
  }
}

const char *wfWallName() {
  return (wfWallSide == WALL_LEFT) ? "LEFT " : "RIGHT";
}
