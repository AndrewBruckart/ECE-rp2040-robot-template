# robot-template

Standard starting repository for the ECE RP2040 robot.

The Arduino sketch lives in [robot_template](./robot_template), and the sketch file is [robot_template.ino](./robot_template/robot_template.ino).

Why it is structured this way:
- the GitHub repo name can be `robot-template`
- the Arduino sketch folder and `.ino` file must match each other
- using `robot_template/robot_template.ino` keeps the Arduino side clean and predictable

Typical workflow:
1. Clone this repo.
2. Open the `robot_template` sketch folder in Arduino IDE or compile it with `arduino-cli`.
3. Copy this repo or use it as a template when starting a new robot project.
