#pragma once

// ── Wall side selection ─────────────────────────────────────────────────────
enum WallSide {
  WALL_LEFT  = 0,
  WALL_RIGHT = 1
};

// ── FSM states ──────────────────────────────────────────────────────────────
//
//  WF_IDLE        – Motors are not running; no steering commands issued.
//  WF_FOLLOWING   – Proportional control keeps robot at target wall distance.
//  WF_AVOID_FRONT – Center sensor too close; steer hard away from followed wall.
//
enum WfState {
  WF_IDLE        = 0,
  WF_FOLLOWING   = 1,
  WF_AVOID_FRONT = 2
};

// ── Tuning parameters (extern – adjust from Wall Follow screen) ─────────────
extern WallSide wfWallSide;         // Which wall to follow (LEFT or RIGHT)
extern float    wfTargetDistance;   // Target wall distance, inches
extern float    wfFrontObstThresh;  // Front obstacle detect threshold, inches
extern float    wfFrontClearThresh; // Front path clear threshold, inches
extern float    wfKp;               // Proportional gain: degrees per inch error
extern int      wfMotorSpeed;       // Motor speed percent (use 5% increments)

// ── Public interface ────────────────────────────────────────────────────────
void        initWallFollow();
void        runWallFollow(bool motorsRunning);  // call every loop iteration

WfState     getWfState();
const char *wfStateName();   // "IDLE  " / "FOLLOW" / "AVOID "
const char *wfWallName();    // "LEFT " / "RIGHT"
