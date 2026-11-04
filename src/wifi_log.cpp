#include "wifi_log.h"

#if WIFI_LOG_ENABLED

#include <WiFi.h>
#include <WiFiUdp.h>

static WiFiUDP udp;
static bool started = false;
static IPAddress viewerIp;
static uint16_t viewerPort = 0;
static uint32_t viewerLastSeenMs = 0;
static const uint32_t VIEWER_TIMEOUT_MS = 10000;

static char cmdBuf[32];

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
    if (wifiLog_hasViewer()) {
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
  WiFi.mode(WIFI_AP);
  // Reduced TX power: full power means ~250 mA current spikes on the 3.3 V rail.
  // 8.5 dBm lost the link on the far side of the track (2026-11-04), 13 dBm ~2x range.
  WiFi.softAP(WIFI_LOG_SSID, WIFI_LOG_PASSWORD, WIFI_LOG_CHANNEL);
  WiFi.setTxPower(WIFI_POWER_13dBm);
  WiFi.setSleep(false);
  udp.begin(WIFI_LOG_PORT);
  started = true;
  Serial.printf("Wi-Fi log: join '%s' (pw %s), then run tools/wifi_log.py  [robot %s:%d]\n",
                WIFI_LOG_SSID, WIFI_LOG_PASSWORD, WiFi.softAPIP().toString().c_str(), WIFI_LOG_PORT);
}

bool wifiLog_hasViewer() {
  return started && viewerPort != 0 && (millis() - viewerLastSeenMs) < VIEWER_TIMEOUT_MS;
}

uint32_t wifiLog_msSinceViewer() {
  if (!started || viewerPort == 0) return UINT32_MAX;
  return millis() - viewerLastSeenMs;
}

void wifiLog_send(const char *line) {
  if (!wifiLog_hasViewer()) return;
  udp.beginPacket(viewerIp, viewerPort);
  udp.write((const uint8_t *)line, strlen(line));
  udp.write('\n');
  udp.endPacket();  // UDP: never waits for the laptop
}

const char *wifiLog_poll() {
  if (!started) return nullptr;
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
const char *wifiLog_poll() { return nullptr; }
Print &wifiLog_printer() { return nullPrinter; }
void wifiLog_flush() {}

#endif
