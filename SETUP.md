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

## 3. Live logging over Wi-Fi

- **Linux:** `tools/log.sh shadow` (or `sonic` / `knuckles`) joins the robot's
  Wi-Fi by itself, shows the live viewer and records to `crashlogs/`.
- **Windows:** join the robot's Wi-Fi in the Windows Wi-Fi menu
  (`Shadow-log` / `Sonic-X-log` / `Knuckles-log`, password `sonicx123`), then run
  `tools\log.bat` (double-click or from a terminal). Simple scrolling view.

In the viewer: type a command (`status`, `sensors`, `log`) or a **mark**
(`!3` slower, `!4` stuck, `!5` stopped, `!7` crash, `!0` start, `!any text`) +
Enter. `q` quits. One laptop can log one robot at a time.

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

`name`, `params`, `sensors`, `scan`, `log`, `status`, `help`
