# Robot Template

This sketch is a clean starter template for the RP2040 robot board used in ECE.

It is meant to be the first project students clone or copy when starting a new robot assignment. The goal is to provide:

- known-good pin definitions
- working OLED setup
- working encoder input
- working motor driver setup
- working steering servo setup
- working IR and light sensor reads
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
Use the encoder to adjust either steering angle or motor speed.
- short press toggles between servo and motor percent
- `START` runs both motors forward
- `STOP` brakes the motors
- long press returns home

3. `Outputs Test`
Lets you turn LEDs, turn signals, brake light, and speaker on and off.

4. `Quick Help`
Shows the core operator controls on the OLED.

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
