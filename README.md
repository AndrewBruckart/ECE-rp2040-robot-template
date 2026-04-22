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

## Run Logging

The robot now logs autonomous runs from the existing `Run The Race` and `Steps` screens.

- Log records stay in SRAM while the robot is running, so the main loop does not pause to write flash during the run.
- A `START` row is recorded when a run begins, state changes are recorded immediately, and a `SAMPLE` row is recorded every 200 ms.
- When the run stops, the buffered records are written to LittleFS as `/race_0000.csv`, `/race_0001.csv`, and so on.
- CSV rows include elapsed time, mode, step index, wall-follow state transitions, steering and drive commands, tuning values, IR distance readings, the center IR ADC reading, the light sensor, and accelerometer values.

Serial commands:

```text
HELP
LISTLOGS
DUMPLATEST
DUMPLOG /race_0000.csv
LISTFILES
DUMPFILE /name.txt
```

`LISTLOGS` and `LISTFILES` return machine-friendly `path=` / `size=` lines. `DUMPLATEST`, `DUMPLOG`, and `DUMPFILE` wrap file contents between `BEGIN FILE ...` and `END FILE ...`, which makes them easy for PowerShell to capture directly into a `.csv` file.

Windows download helper:

```powershell
.\Download-RaceLog.ps1 -Port COM5
```

That command asks the robot for `DUMPLATEST` and saves the returned CSV into `.\downloaded-logs\`. You can also fetch a specific file with:

```powershell
.\Download-RaceLog.ps1 -Port COM5 -RemotePath /race_0003.csv
```

LittleFS / flash setting:

- Build the sketch with `flash=16777216_8388608` so the 16 MB board keeps at least 8 MB available for LittleFS.
- In Arduino IDE, set `Tools > Flash Size` to `16MB (Sketch: 8MB, FS: 8MB)`. The board package default is `16MB (no FS)`, which will make logging fail with `ERR FS unavailable`.
- `robot_template/sketch.yaml` already uses that flash layout as the default FQBN.
- On a fresh board or after changing flash layouts, the firmware may format LittleFS once on boot before logs become available.
