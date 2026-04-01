# robot-template

Standard starting repository for the ECE RP2040 robot. Fork or copy this repository to begin a new robot project.

The Arduino sketch lives in [robot_template](./robot_template), and the sketch file is [robot_template.ino](./robot_template/robot_template.ino).

Why it is structured this way:
- the GitHub repo name can be `robot-template`
- the Arduino sketch folder and `.ino` file must match each other
- using `robot_template/robot_template.ino` keeps the Arduino side clean and predictable

Files at the repo root:
- `AGENTS.md`: project guidance for Codex and other agents
- `arduino-cli.yaml`: shared Arduino CLI board manager config
- `arduino-cli.cmd` and `arduino-cli.ps1`: Windows wrapper scripts
- `arduino-cli`: macOS/Linux wrapper script
- `libraries.txt`: required Arduino libraries for this template

Typical workflow:
1. Fork this repo to your own GitHub account or use it as a template.
2. Clone your copy of the repo.
3. Install `arduino-cli` or Arduino IDE 2.x.
4. Install the RP2040 core and required libraries.
5. Open the `robot_template` sketch folder in Arduino IDE or compile it with `arduino-cli`.

Useful setup commands:

```powershell
./arduino-cli core update-index
./arduino-cli core install rp2040:rp2040
Get-Content .\libraries.txt | ForEach-Object { ./arduino-cli lib install $_ }
./arduino-cli compile --fqbn rp2040:rp2040:sparkfun_thingplusrp2040:flash=16777216_8388608 .\robot_template
```
