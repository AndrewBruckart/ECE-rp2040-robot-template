# Robot Template

This sketch is a clean starter template for the RP2040 robot board used in ECE.

It is meant to be the first project students clone or copy when starting a new robot assignment. The goal is to provide:

- known-good pin definitions
- working OLED setup
- working encoder input
- working motor driver setup
- working steering servo setup
- working IR and light sensor reads
- per-sensor IR distance interpolation for wall following
- working accelerometer reads
- simple on-board hardware self-tests

## What This Template Includes

- `config.h`: board pin map and shared hardware constants
- `encoder.*`: shaft encoder support
- `motors.*`: MX1515 motor control wrapper
- `steering.*`: steering servo wrapper
- `sensors.*`: IR, light, and accelerometer reads
- `ui.*`: small OLED drawing helpers
- `Robot Template.ino`: the main hardware bring-up menu

## On-Board Menu

1. `Display Sensors`
Shows left, center, and right IR values, the light sensor, and accelerometer values.

2. `Tune Motors/Servo`
Use the encoder to navigate `Servo`, `Motor`, and `Drive`.
- short press edits `Servo` or `Motor`, and toggles `Drive`
- the `Drive` row can start or stop both motors from the tune menu
- `START` still runs both motors forward
- `STOP` brakes the motors
- long press returns home

3. `Outputs Test`
Lets you turn LEDs, turn signals, brake light, and speaker on and off.

4. `Wall Follow`
Runs wall following with proportional steering on the servo.
- short press toggles between menu navigation and editing
- menu items are `Wall`, `Dist`, `Kp`, and `Motor`
- `START` begins the wall-follow drive using the currently selected values
- `STOP` brakes the motors
- long press returns home

5. `Run The Race`
Waits for the `START` button, then begins the currently defined autonomous race behavior.
- right now the first two implemented race steps are `Back Out of Garage` and `Friends House`
- pressing `START` in `Run The Race` or `Steps` waits 1 second before the robot begins moving
- `Back Out of Garage` backs straight out until both side IR sensors no longer see the garage walls, then backs with full-left steering for 1500 ms
- `Friends House` then centers steering to 90 degrees, drives forward until the right IR sensor raw reading reaches about 500, and right-follows at 100% speed until the light sensor drops to about 800 and returns to about 2650
- race mode currently chains `Back Out of Garage` into `Friends House`, then stops there until the next step is implemented
- `STOP` brakes the motors
- long press returns home

6. `Steps`
Lets you scroll through named race steps and run one step at a time.
- the current step list is `Back Out of Garage`, `Friends House`, `Follow To Tunnel`, `Drive Through Tunnel`, `Drive To Charge`, `Stop At Charge`, and `Back Up Into Church`
- `Back Out of Garage` is implemented as a custom step:
  back straight at 100% until both side sensors lose the wall, then back full-left for 1500 ms
- `Friends House` is implemented as a custom step:
  center steering to 90 degrees, drive until the right IR sensor raw reading reaches about 500, then right-follow at 100% until the light sensor goes down to about 800 and comes back to about 2650
- the remaining steps still default to right wall follow for now
- `STOP` brakes the motors
- long press returns home

## Why Use This Template

Use this template when starting a new project so you do not need to rediscover:

- which pin goes to which device
- whether the OLED wiring works
- whether the encoder works
- whether the servo centers correctly
- whether the motors and brake light respond
- whether the sensors are alive

## Recommended Workflow For New Projects

1. Copy this folder to a new project folder with a new name.
2. Keep `.git` at the repo root, not inside the sketch folder.
3. Make an initial commit like:

```powershell
git add .
git commit -m "Start new project from robot template"
```

4. Put project-specific logic in new files instead of changing the low-level hardware modules unless needed.

## Suggested First Checks On New Hardware

1. Confirm the OLED shows the home screen.
2. Confirm the encoder moves the menu selection.
3. Confirm the tune screen moves the servo.
4. Confirm the motors run with `START` and stop with `STOP`.
5. Confirm the outputs test toggles each LED and the speaker.
6. Confirm the sensors screen updates all sensor values.

## Notes

- The accelerometer values are shown in milli-g.
- The motor driver wrapper uses the MX1515 Mode B style already used in this project.
- This template is for hardware bring-up and starting new projects. It is not the final contest behavior.
