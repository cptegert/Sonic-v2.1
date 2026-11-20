#include "wifi_log.h"

#if WIFI_LOG_ENABLED

#include <WiFi.h>
#include <WiFiUdp.h>
#include "logger.h"

static WiFiUDP udp;
static bool started = false;
static IPAddress viewerIp;
static uint16_t viewerPort = 0;
static uint32_t viewerLastSeenMs = 0;
static const uint32_t VIEWER_TIMEOUT_MS = 10000;

static char cmdBuf[32];

// Wi-Fi events arrive in the system event task -> only set flags here, log them
// from the loop (wifiLog_poll). 2026-11-04: the AP vanished after a batch run
// while the program and BLE kept working; these tell us what happened next time.
static volatile uint16_t evStaConnected = 0, evStaDisconnected = 0, evApStop = 0;
static volatile uint16_t lastDisconnectReason = 0;
static uint16_t seenStaConnected = 0, seenStaDisconnected = 0, seenApStop = 0;
static uint32_t lastApCheckMs = 0, lastRestartMs = 0;
static const uint32_t RESTART_SETTLE_MS = 3000;  // our own restart also fires AP_STOP: ignore it
static bool hadViewer = false;

static void onWifiEvent(arduino_event_id_t event, arduino_event_info_t info) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_AP_STACONNECTED: evStaConnected++; break;
    case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
      lastDisconnectReason = info.wifi_ap_stadisconnected.reason;
      evStaDisconnected++;
      break;
    case ARDUINO_EVENT_WIFI_AP_STOP: evApStop++; break;
    default: break;
  }
}

static void startAp() {
  WiFi.softAP(WIFI_LOG_SSID, WIFI_LOG_PASSWORD, WIFI_LOG_CHANNEL);
  WiFi.setTxPower(WIFI_POWER_13dBm);
  WiFi.setSleep(false);
}

// The AP restart runs in its own task on core 0 (where Wi-Fi lives), never in the
// loop: 2026-11-20 a restart while driving hung inside the Wi-Fi driver for > 5 s,
// the loop watchdog reset the robot and it stopped. While it runs, the loop skips
// all Wi-Fi work and keeps driving.
static volatile bool restartRunning = false;
static volatile uint32_t restartTookMs = 0;      // set by the task when done
static uint32_t restartStartMs = 0;
static bool restartSlowLogged = false;

static void apRestartTask(void *) {
  uint32_t t = millis();
  WiFi.softAPdisconnect(true);
  vTaskDelay(pdMS_TO_TICKS(50));
  WiFi.mode(WIFI_AP);
  startAp();
  restartTookMs = millis() - t;
  restartRunning = false;
  vTaskDelete(NULL);
}

static void restartAp(const char *why) {
  if (restartRunning) return;
  lastRestartMs = millis();
  restartStartMs = lastRestartMs;
  restartSlowLogged = false;
  restartTookMs = 0;
  logEvent("Wi-Fi AP restart (%s), heap=%lu", why, (unsigned long)ESP.getFreeHeap());
  restartRunning = true;
  if (xTaskCreatePinnedToCore(apRestartTask, "ap_restart", 4096, nullptr, 1, nullptr, 0) != pdPASS) {
    restartRunning = false;
    logEvent("Wi-Fi AP restart task could not start");
  }
}

// Called from the loop: log Wi-Fi events, keep the access point alive
static void wifiHousekeeping() {
  if (restartRunning) {
    if (!restartSlowLogged && millis() - restartStartMs > 10000) {
      restartSlowLogged = true;
      logEvent("Wi-Fi AP restart still running after 10 s (driver stuck?) - loop keeps driving");
    }
    return;
  }
  if (restartTookMs) {
    logEvent("Wi-Fi AP restart done in %lu ms", (unsigned long)restartTookMs);
    restartTookMs = 0;
  }
  if (evStaConnected != seenStaConnected) {
    seenStaConnected = evStaConnected;
    logEvent("Wi-Fi: laptop/station joined (%d connected)", WiFi.softAPgetStationNum());
  }
  if (evStaDisconnected != seenStaDisconnected) {
    seenStaDisconnected = evStaDisconnected;
    logEvent("Wi-Fi: station left, reason %u", lastDisconnectReason);
  }
  uint32_t now = millis();
  if (evApStop != seenApStop) {
    seenApStop = evApStop;
    if (now - lastRestartMs > RESTART_SETTLE_MS) {  // not caused by our own restart
      logEvent("Wi-Fi: AP STOPPED");
      restartAp("AP stopped");
      return;
    }
  }
  if (now - lastRestartMs < RESTART_SETTLE_MS) return;
  if (now - lastApCheckMs < 5000) return;
  lastApCheckMs = now;
  if (!(WiFi.getMode() & WIFI_MODE_AP) || WiFi.softAPIP() == IPAddress(0, 0, 0, 0)) {
    restartAp("AP not running");
    return;
  }
  // Laptop was talking to us, then went silent for 30 s: restart the AP once
  // (~0.1 s). Do NOT require "no stations": 2026-11-20 the radio died while driving
  // (Wi-Fi and BLE at the same moment), the program kept running and the dead
  // laptop link still counted as a connected station, so the AP was never restarted.
  if (wifiLog_hasViewer()) hadViewer = true;
  if (hadViewer && wifiLog_msSinceViewer() > 30000 && now - lastRestartMs > 60000) {
    hadViewer = false;
    char why[64];
    snprintf(why, sizeof(why), "viewer silent 30 s, %d station(s) listed", WiFi.softAPgetStationNum());
    restartAp(why);
  }
}

// Collects printed text and sends it as UDP packets (for long outputs like "log")
class UdpPrinter : public Print {
 public:
  size_t write(uint8_t c) override {
    buf[len++] = c;
    if (len >= sizeof(buf)) flush();
    return 1;
  }
  void flush() override {
    if (len == 0) return;
    if (!restartRunning && wifiLog_hasViewer()) {
      // Only used for command output while stopped (e.g. "log"). Sending fast fills
      // the Wi-Fi buffers and the heap (LOW HEAP 13 KB seen) -> small packets, pace
      // them, and wait while memory is low.
      for (int i = 0; i < 50 && ESP.getFreeHeap() < 25000; i++) delay(10);
      udp.beginPacket(viewerIp, viewerPort);
      udp.write(buf, len);
      udp.endPacket();
      delay(15);
    }
    len = 0;
  }
 private:
  uint8_t buf[512];
  size_t len = 0;
};
static UdpPrinter printer;

void wifiLog_begin() {
  WiFi.onEvent(onWifiEvent);
  WiFi.mode(WIFI_AP);
  // Reduced TX power: full power means ~250 mA current spikes on the 3.3 V rail.
  // 8.5 dBm lost the link on the far side of the track (2026-11-04), 13 dBm ~2x range.
  startAp();
  udp.begin(WIFI_LOG_PORT);
  started = true;
  Serial.printf("Wi-Fi log: join '%s' (pw %s), then run tools/wifi_log.py  [robot %s:%d]\n",
                WIFI_LOG_SSID, WIFI_LOG_PASSWORD, WiFi.softAPIP().toString().c_str(), WIFI_LOG_PORT);
}

bool wifiLog_hasViewer() {
  return started && viewerPort != 0 && (millis() - viewerLastSeenMs) < VIEWER_TIMEOUT_MS;
}

int wifiLog_stations() {
  return started ? WiFi.softAPgetStationNum() : 0;
}

uint32_t wifiLog_msSinceViewer() {
  if (!started || viewerPort == 0) return UINT32_MAX;
  return millis() - viewerLastSeenMs;
}

void wifiLog_send(const char *line) {
  if (restartRunning || !wifiLog_hasViewer()) return;
  udp.beginPacket(viewerIp, viewerPort);
  udp.write((const uint8_t *)line, strlen(line));
  udp.write('\n');
  udp.endPacket();  // UDP: never waits for the laptop
}

const char *wifiLog_poll() {
  if (!started) return nullptr;
  wifiHousekeeping();
  if (restartRunning) return nullptr;
  int n = udp.parsePacket();
  if (n <= 0) return nullptr;
  int len = udp.read(cmdBuf, sizeof(cmdBuf) - 1);
  if (len < 0) len = 0;
  cmdBuf[len] = 0;
  while (len > 0 && (cmdBuf[len - 1] == '\n' || cmdBuf[len - 1] == '\r' || cmdBuf[len - 1] == ' ')) cmdBuf[--len] = 0;

  bool wasConnected = wifiLog_hasViewer();
  viewerIp = udp.remoteIP();
  viewerPort = udp.remotePort();
  viewerLastSeenMs = millis();
  if (!wasConnected) wifiLog_send("# robot: viewer registered");

  if (strcmp(cmdBuf, "hello") == 0 || len == 0) return nullptr;  // keepalive only
  return cmdBuf;
}

Print &wifiLog_printer() { return printer; }
void wifiLog_flush() { printer.flush(); }

#else  // WIFI_LOG_ENABLED == 0

class NullPrinter : public Print {
 public:
  size_t write(uint8_t) override { return 1; }
};
static NullPrinter nullPrinter;
void wifiLog_begin() {}
void wifiLog_send(const char *) {}
bool wifiLog_hasViewer() { return false; }
uint32_t wifiLog_msSinceViewer() { return UINT32_MAX; }
int wifiLog_stations() { return 0; }
const char *wifiLog_poll() { return nullptr; }
Print &wifiLog_printer() { return nullPrinter; }
void wifiLog_flush() {}

#endif
