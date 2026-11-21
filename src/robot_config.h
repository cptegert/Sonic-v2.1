// Robot identity: ONE firmware for all robots, the identity is stored on the board.
// -----------------------------------------------------------------------------
// The profiles below (name, Bluetooth name, RemoteXY GUI, motor scale) are in every
// firmware. The board's flash (NVS "robot"/"id") only says WHICH profile it is,
// and survives firmware updates. Set it with:
//   tools/robot.py upload --robot shadow      (does it automatically), or
//   serial / Wi-Fi command:  name shadow       (saves and restarts)
// A board without a stored identity starts as "Robot-xxxx" (last MAC bytes) with
// Sonic's GUI, so it is always reachable (Wi-Fi "Robot-xxxx-log").
// New robot: add a line to PROFILES in robot_config.cpp and to tools/robots.json.
// -----------------------------------------------------------------------------
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <Arduino.h>

enum RobotGui : uint8_t { GUI_SONIC = 0, GUI_SHADOW = 1 };

void robot_identity_begin();          // FIRST thing in setup(): reads the stored identity
const char *robotKey();               // "sonic" / "shadow" / "knuckles" / "" (none stored)
const char *robotName();              // "Sonic-X", "Shadow", ... or "Robot-xxxx"
const char *robotBleName();           // Bluetooth name in the RemoteXY app
const char *robotWifiSsid();          // "<name>-log"
RobotGui robotGui();                  // which RemoteXY GUI array
float robotMotorScale();              // ESC scale around neutral (1.0 = Sonic's ESC)
bool robotIdentityStored();           // false = default "Robot-xxxx"
bool robotSetIdentity(const char *key);   // saves to flash; takes effect after restart
void robotListProfiles(Print &out);

#endif
