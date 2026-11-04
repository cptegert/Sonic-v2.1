// Crash / black-box logger
// -----------------------------------------------------------------------------
// Goal: find out why the robot sometimes stops dead (no motor, no BLE, button
// ignored) even when nobody is watching the serial monitor.
//
// Three layers:
//  1. RTC "black box" (survives panic / watchdog / brownout resets, NOT power loss):
//     live snapshot updated every loop + ring buffer of recent events.
//  2. Flash log file on LittleFS (/log.txt, survives everything):
//     boot reasons, previous-session black box, crash summaries, events, and a
//     heartbeat every few seconds so even a total power loss shows "last seen".
//  3. Loop watchdog: if loop() hangs >5 s the ESP32 resets and the reason +
//     the stage it was stuck in is logged on the next boot.
//
// Reading the log: USB serial monitor at 115200, type  log  + Enter.
// Other commands:  status  (live snapshot),  clear  (delete log file),  help
// (or run tools/serial_capture.py <port> 10 log  - it does NOT reset the robot)
// -----------------------------------------------------------------------------
#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>

// Where in loop() we currently are. Stored in RTC memory, so after a watchdog
// reset we know which part of the code was hanging.
enum LogStage : uint8_t {
  STAGE_SETUP = 0,
  STAGE_LOOP_START,
  STAGE_REMOTEXY,
  STAGE_SENSOR_FL,
  STAGE_SENSOR_L,
  STAGE_SENSOR_M,
  STAGE_SENSOR_R,
  STAGE_SENSOR_FR,
  STAGE_BUTTON,
  STAGE_DRIVE,
  STAGE_COLOR,
  STAGE_OUTPUTS,
  STAGE_SAVE,
  STAGE_LOGGER,
  STAGE_LOOP_END,
  STAGE_COUNT
};

// Live state copied into the black box once per loop
struct LogSnapshot {
  uint16_t cycleMs;
  int16_t  dist[7];
  int16_t  speedUs;
  int16_t  steerUs;
  uint8_t  startSwitch;
  uint8_t  manual;
  uint8_t  driving;      // countdown finished, motors under control
  uint8_t  connected;    // RemoteXY BLE app connected
  uint8_t  reversing;
};

void logger_begin();                        // FIRST thing in setup() (after Serial.begin)
void logger_setupDone();                    // LAST thing in setup(): enables loop watchdog
void logger_loop(const LogSnapshot &s);     // once per loop: snapshot, heartbeat, flush, serial cmds
void logStage(LogStage stage);              // cheap breadcrumb
void logEvent(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
void logI2CError(const char *sensorName, uint8_t status);  // rate-limited
bool logger_lastResetAbnormal();            // brownout / crash / watchdog on last boot

#endif
