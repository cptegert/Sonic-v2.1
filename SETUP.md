# Setup and uploading firmware (Windows / Linux / macOS)

One firmware for all robots (Sonic, Shadow, Knuckles). Which robot a board is,
is stored on the ESP32 itself (see `src/robot_config.h`).

## 1. One-time setup

1. Install **VS Code** and the **PlatformIO IDE** extension.
2. Get the code: `git clone https://github.com/cptegert/Sonic-v2.1.git`
   (or `git pull` in an existing copy - you need the latest `main`).
3. Open the folder in VS Code. PlatformIO installs everything else by itself on
   the first build (ESP32 platform 8.0.0, libraries): the exact versions are
   pinned in `platformio.ini`. The first build takes a few minutes.
   - Build errors about Wi-Fi / network headers mean an old ESP32 platform:
     make sure you have the latest `platformio.ini` (`git pull`), then build again.
4. Windows: if the robot does not show up as a COM port, install the
   **CP210x USB driver** (Silicon Labs).

## 2. Upload to a robot

Connect the robot by USB, **battery off**.

### Easiest: VS Code
1. PlatformIO status bar: environment `env:robot` (the only one).
2. Click **Upload** (→).
3. The board keeps its stored identity. A new / erased board starts as
   `Robot-xxxx`: open the **Serial Monitor** (plug icon, 115200) and type
   `name sonic` (or `shadow`, `knuckles`) + Enter. The robot saves it and restarts.
4. Check: type `name` in the Serial Monitor:
   `robot: Sonic-X  key=sonic  bluetooth=Sonic-X  wifi=Sonic-X-log  stored=1`

### Or with the upload tool (detects the robot, sets the identity)
Run in the project folder (a terminal in VS Code):

- Windows:
  ```
  %USERPROFILE%\.platformio\penv\Scripts\python.exe tools\robot.py upload
  ```
  (PowerShell: `& "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" tools\robot.py upload`)
- Linux / macOS:
  ```
  ~/.platformio/penv/bin/python tools/robot.py upload
  ```

New board: add `--robot sonic` (and `--erase` for a used ESP32).
`tools/robot.py info` only shows which robot is connected.

## 3. Live logging over Wi-Fi: the dashboard

- **Linux:** `tools/log.sh shadow` (or `sonic` / `knuckles`) joins the robot's
  Wi-Fi by itself and opens the dashboard in the browser (http://localhost:8770).
- **Windows:** join the robot's Wi-Fi in the Windows Wi-Fi menu
  (`Shadow-log` / `Sonic-X-log` / `Knuckles-log`, password `sonicx123`), then run
  `tools\log.bat sonic` (or `shadow` / `knuckles`).

The dashboard shows everything live and records to `crashlogs/`:
- **Marks:** buttons for wrong way / right way again / tipped / stuck / crash … and
  your own notes. They are saved with the exact time and sent to the robot. While
  "wrong way" is on, the robot does not learn laps or its speed map.
- **Track:** pick or create a named track at the top. The robot keeps a speed map
  and a lap mark per track (up to 6). A new track name = it learns from scratch.
- **Build your track from tiles** (third tab, "Tiles"): pick straight / 90° turn /
  ramp (arrow = uphill) / bridge, click squares to place, click again to turn.
  Put the robot on its start tile with the Robot tool and enter the tile size.
  Before each drive press **Place robot for the next drive** on the map and click
  the tile where the robot stands (click again to turn it). The robot's data is then
  placed on your track. After 8+ laps with a good match the robot's steering is
  calibrated, which makes its own guesses for new tracks better.
  ("Draw freely" and "Describe", e.g. `S3 L90 S1 H1 L90`, also work.)
- **Speed calibration:** drives several speeds (robot on the track, main switch OFF,
  phone app closed), scores laps per minute, and "Apply" saves the best speed on the
  robot. Pause waits after the current run (battery swap).
- **Dirty wheels:** the robot notices when a part of the track gets closer to the
  wall than usual (sliding wider) and slows down there. After cleaning the wheels
  press **Wheels cleaned**: the slowed parts recover at once. The blue "margin" line
  in the laps chart falling lap after lap = time to clean.
- **Wrong-way detection** (logging only): the robot compares its steering and wall
  distances with the learned lap, forwards and backwards. The panel compares that
  with your Wrong way / Right way marks. A figure-8 looks the same backwards, so
  it can't work there - the panel tells you.
- Laps with their trouble, speed along the lap, track parts, sensors, robot log,
  command line. Click a section title to fold it away.

One program at a time talks to the robot (close the dashboard before
`auto_run.py` / `batch_run.py`). Old terminal viewer: `tools/log.sh shadow term`
or `tools\log.bat shadow term`.

## 4. RemoteXY app

After a name change the phone may still show the old Bluetooth name (cache):
delete the robot in the app and scan again; if needed switch the phone's
Bluetooth off and on.

## Serial Monitor

PlatformIO's Serial Monitor (plug icon in the status bar) opens in the terminal
panel: click into it, type the command, press Enter. Typing is visible and the
line is sent on Enter thanks to `monitor_echo` / `send_on_enter` in
`platformio.ini` (needs the latest `git pull`).
Opening the monitor restarts the robot; wait a few seconds until it has started.

## Useful serial commands (Serial Monitor, 115200)

`name`, `params`, `sensors`, `scan`, `log`, `status`, `track [name]`, `map`, `mark <text>`, `save`, `help`
