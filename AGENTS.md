## Purpose

This repository is a portable starter template for the ECE RP2040 robot. Keep it easy for students and agents to reuse on different computers.

## Repository Layout

- `robot_template/`: Arduino sketch and hardware modules
- `arduino-cli.yaml`: shared Arduino CLI config
- `arduino-cli.cmd` and `arduino-cli.ps1`: Windows wrapper scripts
- `arduino-cli`: macOS/Linux wrapper script
- `libraries.txt`: required Arduino libraries

## Working Rules

- Prefer small, reviewable changes over large refactors.
- Avoid adding machine-specific absolute paths, usernames, or COM port assumptions.
- Do not check in generated build output or local Arduino package caches.
- Keep the root repo usable on Windows, macOS, and Linux when practical.
- When adding a new library dependency, also add it to `libraries.txt`.
- Preserve the sketch folder name `robot_template` unless the user explicitly wants it renamed.

## Tooling

- Prefer `arduino-cli` for compile and library management tasks.
- Use the provided wrapper scripts if they work in the current environment.
- If the wrappers do not work, use any working `arduino-cli` on the machine instead.
- The recommended core is Earle Philhower's RP2040 core from the URL in `arduino-cli.yaml`.

Common commands:

```text
./arduino-cli core update-index
./arduino-cli core install rp2040:rp2040
./arduino-cli compile --fqbn rp2040:rp2040:sparkfun_thingplusrp2040:flash=16777216_8388608 ./robot_template
```

## Hardware Notes

Target board: SparkFun Thing Plus RP2040 with 8 MB program flash and 8 MB LittleFS.

Pin map summary from `robot_template/config.h`:
- Start button: GPIO 16
- Stop button: GPIO 17
- Left motor: GPIO 24 and GPIO 23
- Right motor: GPIO 22 and GPIO 21
- Steering servo: GPIO 14
- Encoder: GPIO 18, GPIO 19, GPIO 20
- OLED I2C: SDA GPIO 4, SCL GPIO 5, address `0x3C`
- Accelerometer I2C address: `0x1C`
- LEDs and indicators are defined in `robot_template/config.h`

## Editing Guidance

- Treat this repo as a hardware bring-up and starter template, not a finished contest robot.
- Keep sample code simple and reliable for first use on new hardware.
- If changing robot behavior substantially, document why in the README or sketch docs.
- If environment assumptions seem wrong, stop and ask rather than guessing.
