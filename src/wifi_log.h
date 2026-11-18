// Wi-Fi live log for a laptop (Bluetooth stays for the RemoteXY phone app)
// -----------------------------------------------------------------------------
// The robot opens its own access point. Connect the laptop to it and run:
//   ~/.platformio/penv/bin/python tools/wifi_log.py
// You get: every log event in real time, a live status line 5x per second, and
// the same commands as the USB serial port (log, status, clear).
//
// UDP is used on purpose: sending never blocks the control loop, even if the
// laptop walks out of range. The robot only sends while a laptop is registered
// (the viewer sends "hello" every 2 s; after 10 s of silence the robot stops).
// -----------------------------------------------------------------------------
#ifndef WIFI_LOG_H
#define WIFI_LOG_H

#include <Arduino.h>

#define WIFI_LOG_ENABLED   1               // 0 = no Wi-Fi at all (e.g. if rules forbid it)
#include "robot_config.h"
#define WIFI_LOG_SSID      ROBOT_WIFI_SSID  // "<robot name>-log", see platformio.ini
#define WIFI_LOG_PASSWORD  "sonicx123"     // min 8 chars (WPA2)
#define WIFI_LOG_CHANNEL   6
#define WIFI_LOG_PORT      4210            // robot listens here, replies to the sender
#define WIFI_LOG_STATUS_HZ 5

void wifiLog_begin();
void wifiLog_send(const char *line);       // one line (no newline needed); no-op without a viewer
bool wifiLog_hasViewer();
uint32_t wifiLog_msSinceViewer();          // ms since the last packet from the laptop (UINT32_MAX = never)
// Call once per loop. Returns a received command (e.g. "log") or nullptr.
const char *wifiLog_poll();
Print &wifiLog_printer();                  // Print target for long command output (e.g. log dump)
void wifiLog_flush();                      // send what has been printed to wifiLog_printer()

#endif
