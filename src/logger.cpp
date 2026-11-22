#include "logger.h"
#include <LittleFS.h>
#include <Preferences.h>
#include <esp_system.h>
#include <esp_core_dump.h>
#include <esp_task_wdt.h>
#include <stdarg.h>
#include "wifi_log.h"
#include "distance_sensors.h"  // printSensorDiagnostics
#include "robot_config.h"      // ROBOT_NAME
#include "lap_detect.h"        // lap count in the status line
#include "speed_map.h"         // map commands

// ============================================================================
// SETTINGS
// ============================================================================
static const char    *LOG_FILE        = "/log.txt";
static const char    *LOG_FILE_OLD    = "/log_old.txt";
static const size_t   LOG_MAX_BYTES   = 64 * 1024;  // rotate to log_old.txt after this
// Round diary (2026-11-22): only what matters for the next round (drives, laps, speed
// map, marks, wrong way, grip) - survives a whole competition day, unlike the log
// above that heartbeats fill up. Read with "rounds" (dashboard: Robot's diary).
static const char    *ROUNDS_FILE     = "/rounds.txt";
static const char    *ROUNDS_OLD      = "/rounds_old.txt";
static const size_t   ROUNDS_MAX      = 48 * 1024;
static int64_t clockBase = 0;   // unix time at millis() = 0, set by the laptop ("clock")
static int32_t clockTz = 0;
static const char *DIARY[] = {"LAPSUM", "SPEEDMAP", "MARK", "WRONG WAY", "right way again", "grip",
                              "wheels cleaned", "track ->", "track renamed", "speed map", "driving started", "driving stopped",
                              "laps this run", "remote: start", "remote run", "remote: stop",
                              "app: main switch", "start button", "NOT MOVING", "BLIND", "new lap mark",
                              "lap mark", "settings saved", "remote: set", "laps ", "identity"};
static bool isDiary(const char *m) {
  for (const char *p : DIARY) if (strncmp(m, p, strlen(p)) == 0) return true;
  return false;
}
// Date from unix time without gmtime/strftime (those cost ~7 KB of heap - measured
// on Sonic 2026-11-22). Days -> y/m/d: H. Hinnant's civil_from_days.
static void stampOf(uint32_t ms, uint32_t boot, char *buf, size_t len) {
  if (clockBase) {
    int64_t t = clockBase + clockTz + ms / 1000;
    int64_t days = t / 86400;
    int32_t sec = (int32_t)(t % 86400);
    days += 719468;
    int64_t era = days / 146097;
    int32_t doe = (int32_t)(days - era * 146097);
    int32_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    int32_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    int32_t mp = (5 * doy + 2) / 153;
    int32_t d = doy - (153 * mp + 2) / 5 + 1;
    int32_t m = mp < 10 ? mp + 3 : mp - 9;
    int32_t y = (int32_t)(yoe + era * 400) + (m <= 2);
    snprintf(buf, len, "%04ld-%02ld-%02ld %02ld:%02ld:%02ld", (long)y, (long)m, (long)d,
             (long)(sec / 3600), (long)(sec / 60 % 60), (long)(sec % 60));
  } else {
    snprintf(buf, len, "boot#%lu +%.1fs", (unsigned long)boot, ms / 1000.0);
  }
}
static const uint32_t HB_DRIVING_MS   = 5000;       // heartbeat to flash while driving
static const uint32_t HB_IDLE_MS      = 30000;      // heartbeat to flash while idle
static const uint32_t FLUSH_IDLE_MS   = 1000;       // flush events to flash when not driving
static const uint16_t SLOW_LOOP_MS    = 100;        // log loops slower than this
static const uint32_t LOW_HEAP_BYTES  = 20000;      // log once if free heap drops below
static const uint32_t WIFI_STATUS_MS  = 1000 / WIFI_LOG_STATUS_HZ;

// ============================================================================
// RTC BLACK BOX (survives software resets / watchdog / panic, not power loss)
// ============================================================================
#define BB_MAGIC   0x50C1C0DEu
#define BB_EVENTS  32
#define BB_MSG_LEN 96   // was 56: lap summaries need more (RTC: 32 x 100 B)

struct BbEvent {
  uint32_t ms;
  char msg[BB_MSG_LEN];
};

struct BlackBox {
  uint32_t magic;
  uint32_t size;
  uint32_t bootNo;
  uint32_t uptimeMs;
  uint32_t loops;
  uint8_t  stage;
  uint16_t maxCycleMs;
  uint32_t minFreeHeap;
  uint32_t freeHeap;     // current free heap (last loop)
  uint32_t i2cErrors;
  LogSnapshot snap;
  uint32_t evTotal;    // events logged this session
  uint32_t evFlushed;  // events already written to flash
  BbEvent  ev[BB_EVENTS];
};

RTC_NOINIT_ATTR static BlackBox bb;

static const char *STAGE_NAMES[STAGE_COUNT] = {
  "setup", "loop_start", "remotexy", "sensor_FFL", "sensor_FL", "sensor_L", "sensor_M",
  "sensor_R", "sensor_FR", "sensor_FFR", "button", "drive", "color", "outputs", "save",
  "logger", "loop_end"
};

static bool fsOk = false;
static bool lastResetAbnormal = false;
static uint32_t bootNo = 0;

static const char *stageName(uint8_t s) {
  return s < STAGE_COUNT ? STAGE_NAMES[s] : "?";
}

static const char *resetReasonStr(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "POWERON (power was off / battery or wiring)";
    case ESP_RST_EXT:       return "EXTERNAL (EN pin)";
    case ESP_RST_SW:        return "SOFTWARE (esp_restart)";
    case ESP_RST_PANIC:     return "PANIC (software crash)";
    case ESP_RST_INT_WDT:   return "INTERRUPT WATCHDOG (hang with interrupts off)";
    case ESP_RST_TASK_WDT:  return "TASK WATCHDOG (loop hung >5 s)";
    case ESP_RST_WDT:       return "OTHER WATCHDOG";
    case ESP_RST_DEEPSLEEP: return "DEEP SLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT (supply voltage dipped)";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "UNKNOWN";
  }
}

// ============================================================================
// FLASH FILE
// ============================================================================
static void rotateIfNeeded() {
  File f = LittleFS.open(LOG_FILE, FILE_READ);
  if (!f) return;
  size_t sz = f.size();
  f.close();
  if (sz < LOG_MAX_BYTES) return;
  LittleFS.remove(LOG_FILE_OLD);
  LittleFS.rename(LOG_FILE, LOG_FILE_OLD);
}

// Write one line to the flash log (and optionally echo to Serial)
static void appendLine(const char *line, bool echo = true) {
  if (echo) {
    Serial.println(line);
    wifiLog_send(line);
  }
  if (!fsOk) return;
  rotateIfNeeded();
  File f = LittleFS.open(LOG_FILE, FILE_APPEND);
  if (!f) return;
  f.println(line);
  f.close();
}

static void appendf(bool echo, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void appendf(bool echo, const char *fmt, ...) {
  char buf[256];
  int n = snprintf(buf, sizeof(buf), "[#%lu %7.3fs] ", (unsigned long)bootNo, millis() / 1000.0);
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf + n, sizeof(buf) - n, fmt, ap);
  va_end(ap);
  appendLine(buf, echo);
}

// Move not-yet-saved RTC events into the flash file. sessionBoot is the boot
// number the events belong to (differs from bootNo when saving the previous
// session's events at startup).
static void flushEvents(uint32_t sessionBoot) {
  if (bb.evFlushed >= bb.evTotal) return;
  if (!fsOk) { bb.evFlushed = bb.evTotal; return; }
  rotateIfNeeded();
  File f = LittleFS.open(LOG_FILE, FILE_APPEND);
  if (!f) return;
  uint32_t first = bb.evFlushed;
  if (bb.evTotal - first > BB_EVENTS) {
    f.printf("[#%lu] (%lu events lost, ring buffer overflow)\n",
             (unsigned long)sessionBoot, (unsigned long)(bb.evTotal - first - BB_EVENTS));
    first = bb.evTotal - BB_EVENTS;
  }
  File d;
  for (uint32_t i = first; i < bb.evTotal; i++) {
    const BbEvent &e = bb.ev[i % BB_EVENTS];
    char msg[BB_MSG_LEN + 1];
    memcpy(msg, e.msg, BB_MSG_LEN);
    msg[BB_MSG_LEN] = 0;  // RTC data may be garbage after a weird reset
    f.printf("[#%lu %7.3fs] %s\n", (unsigned long)sessionBoot, e.ms / 1000.0, msg);
    if (isDiary(msg)) {
      if (!d) {
        File r = LittleFS.open(ROUNDS_FILE, FILE_READ);
        size_t sz = r ? r.size() : 0;
        if (r) r.close();
        if (sz >= ROUNDS_MAX) { LittleFS.remove(ROUNDS_OLD); LittleFS.rename(ROUNDS_FILE, ROUNDS_OLD); }
        d = LittleFS.open(ROUNDS_FILE, FILE_APPEND);
      }
      if (d) {
        char st[32];
        stampOf(e.ms, sessionBoot, st, sizeof(st));
        d.printf("%s %s\n", st, msg);
      }
    }
  }
  if (d) d.close();
  f.close();
  bb.evFlushed = bb.evTotal;
}

static void formatSnapshot(char *buf, size_t len, const BlackBox &b) {
  const LogSnapshot &s = b.snap;
  snprintf(buf, len,
           "up=%.1fs loops=%lu stage=%s cyc=%ums max=%ums sw=%u drive=%u man=%u ble=%u rev=%u "
           "spd=%d str=%d dist=[%d %d %d %d %d %d %d] heap=%lu heapMin=%lu i2cErr=%lu sta=%d viewer=%lus lap=%d/%.1fs trk=%s ww=%d smap=%d",
           b.uptimeMs / 1000.0, (unsigned long)b.loops, stageName(b.stage), s.cycleMs, b.maxCycleMs,
           s.startSwitch, s.driving, s.manual, s.connected, s.reversing, s.speedUs, s.steerUs,
           s.dist[0], s.dist[1], s.dist[2], s.dist[3], s.dist[4], s.dist[5], s.dist[6],
           (unsigned long)b.freeHeap, (unsigned long)b.minFreeHeap, (unsigned long)b.i2cErrors,
           wifiLog_stations(), (unsigned long)min(wifiLog_msSinceViewer() / 1000, (uint32_t)9999),
           lap_count(), lap_last_s(), speedmap_track_name(), lap_wrong_way(), speedmap_mode());
}

static void logCoreDumpSummary() {
  esp_core_dump_summary_t *sum = (esp_core_dump_summary_t *)malloc(sizeof(esp_core_dump_summary_t));
  if (!sum) return;
  if (esp_core_dump_get_summary(sum) == ESP_OK) {
    appendf(true, "CRASH in task '%s' pc=0x%08lx cause=%lu vaddr=0x%08lx",
            sum->exc_task, (unsigned long)sum->exc_pc,
            (unsigned long)sum->ex_info.exc_cause, (unsigned long)sum->ex_info.exc_vaddr);
    char bt[200];
    int n = snprintf(bt, sizeof(bt), "backtrace%s:", sum->exc_bt_info.corrupted ? " (corrupted)" : "");
    for (uint32_t i = 0; i < sum->exc_bt_info.depth && i < 16 && n < (int)sizeof(bt) - 12; i++) {
      n += snprintf(bt + n, sizeof(bt) - n, " 0x%08lx", (unsigned long)sum->exc_bt_info.bt[i]);
    }
    appendf(true, "%s", bt);
    appendf(true, "decode: xtensa-esp32-elf-addr2line -pfiaC -e firmware.elf <addresses>");
  } else {
    appendf(true, "no readable core dump in flash");
  }
  free(sum);
}

// ============================================================================
// PUBLIC API
// ============================================================================
void logger_begin() {
  esp_reset_reason_t reason = esp_reset_reason();
  lastResetAbnormal = (reason == ESP_RST_PANIC || reason == ESP_RST_INT_WDT ||
                       reason == ESP_RST_TASK_WDT || reason == ESP_RST_WDT ||
                       reason == ESP_RST_BROWNOUT);

  Preferences p;
  p.begin("logger", false);
  bootNo = p.getUInt("boot", 0) + 1;
  p.putUInt("boot", bootNo);
  p.end();

  fsOk = LittleFS.begin(true);  // formats the partition on first ever use (takes a few s once)
  if (!fsOk) Serial.println("LOGGER: LittleFS mount failed, logging to Serial only");

  appendLine("", false);
  appendf(true, "===== %s BOOT #%lu  reset reason: %s =====", robotName(), (unsigned long)bootNo, resetReasonStr(reason));

  bool prevValid = (bb.magic == BB_MAGIC && bb.size == sizeof(BlackBox) && bb.evFlushed <= bb.evTotal);
  if (prevValid) {
    flushEvents(bb.bootNo);  // events the previous session did not get to save
    char snap[300];
    formatSnapshot(snap, sizeof(snap), bb);
    appendf(true, "previous session #%lu last state: %s", (unsigned long)bb.bootNo, snap);
    if (reason == ESP_RST_POWERON) {
      // A real power-off wipes RTC memory. If it survived, the reset was a short
      // dip / glitch on the 3.3V supply or EN pin (or the EN/RST button, or USB auto-reset).
      appendf(true, "NOTE: black box survived -> short supply/EN glitch, not a long power-off");
    }
  } else if (reason != ESP_RST_POWERON) {
    appendf(true, "previous session black box not available");
  }

  if (reason == ESP_RST_PANIC || reason == ESP_RST_INT_WDT ||
      reason == ESP_RST_TASK_WDT || reason == ESP_RST_WDT) {
    logCoreDumpSummary();
  }

  // Start a fresh black box for this session
  memset(&bb, 0, sizeof(bb));
  bb.magic = BB_MAGIC;
  bb.size = sizeof(BlackBox);
  bb.bootNo = bootNo;
  bb.stage = STAGE_SETUP;
  bb.minFreeHeap = ESP.getFreeHeap();

  Serial.println("Logger ready. Serial commands: log, status, clear, help");
}

void logger_setupDone() {
  // the loop watchdog is already enabled early in setup() (Sonic_X.ino)
  logEvent("setup done, heap=%lu", (unsigned long)ESP.getFreeHeap());
}

bool logger_lastResetAbnormal() {
  return lastResetAbnormal;
}

void logStage(LogStage stage) {
  bb.stage = stage;
}

void logEvent(const char *fmt, ...) {
  BbEvent &e = bb.ev[bb.evTotal % BB_EVENTS];
  e.ms = millis();
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(e.msg, BB_MSG_LEN, fmt, ap);
  va_end(ap);
  bb.evTotal++;
  char line[BB_MSG_LEN + 24];
  snprintf(line, sizeof(line), "[LOG %7.3fs] %.*s", e.ms / 1000.0, BB_MSG_LEN, e.msg);
  Serial.println(line);
  wifiLog_send(line);
}

void logI2CError(const char *sensorName, uint8_t status) {
  static uint32_t lastLogMs = 0;
  bb.i2cErrors++;
  uint32_t now = millis();
  if (bb.i2cErrors == 1 || now - lastLogMs >= 1000) {  // max 1 line per second
    lastLogMs = now;
    logEvent("I2C error %s status=%u (total %lu)", sensorName, status, (unsigned long)bb.i2cErrors);
  }
}

// ---------------------------------------------------------------------------
// Serial commands (non-blocking)
// ---------------------------------------------------------------------------
static void printFile(const char *path, Print &out) {
  File f = LittleFS.open(path, FILE_READ);
  if (!f) return;
  out.printf("----- %s (%u bytes) -----\n", path, (unsigned)f.size());
  uint8_t buf[256];
  while (f.available()) {
    size_t n = f.read(buf, sizeof(buf));
    out.write(buf, n);
    esp_task_wdt_reset();  // printing 64 KB takes ~6 s, keep the loop watchdog happy
  }
  f.close();
}

static LoggerCommandHook commandHook = nullptr;
void logger_setCommandHook(LoggerCommandHook hook) { commandHook = hook; }

// out = Serial or the Wi-Fi printer, depending on where the command came from
static void handleCommand(const char *cmd, bool driving, Print &out) {
  if (commandHook && commandHook(cmd, driving, out)) return;
  if (speedmap_command(cmd, driving, out)) return;
  if (lap_command(cmd, out)) return;
  if (strcmp(cmd, "log") == 0) {
    if (driving) { out.println("Not while driving (turn main switch OFF)"); return; }
    flushEvents(bootNo);
    printFile(LOG_FILE_OLD, out);
    printFile(LOG_FILE, out);
    out.println("----- end of log -----");
  } else if (strcmp(cmd, "status") == 0) {
    char snap[300];
    formatSnapshot(snap, sizeof(snap), bb);
    out.printf("boot #%lu: %s\n", (unsigned long)bootNo, snap);
  } else if (strcmp(cmd, "scan") == 0) {
    i2cScan(out);
  } else if (strcmp(cmd, "sensors") == 0) {
    printSensorDiagnostics(out);
  } else if (strncmp(cmd, "on ", 3) == 0 || strncmp(cmd, "off ", 4) == 0 || strcmp(cmd, "fault") == 0) {
    if (driving) { out.println("Not while driving"); return; }
    sensorPowerCommand(cmd, out);
    esp_task_wdt_reset();  // re-init can take a moment
  } else if (strcmp(cmd, "rounds") == 0) {
    if (driving) { out.println("Not while driving (turn main switch OFF)"); return; }
    flushEvents(bootNo);
    printFile(ROUNDS_OLD, out);
    printFile(ROUNDS_FILE, out);
    out.println("----- end of rounds -----");
  } else if (strcmp(cmd, "rounds clear") == 0) {
    if (driving) { out.println("Not while driving"); return; }
    LittleFS.remove(ROUNDS_FILE);
    LittleFS.remove(ROUNDS_OLD);
    out.println("rounds cleared");
  } else if (strncmp(cmd, "clock ", 6) == 0) {
    char *end = nullptr;
    long long unix = strtoll(cmd + 6, &end, 10);
    long tz = end ? strtol(end, nullptr, 10) : 0;
    if (unix > 1700000000LL) {
      clockBase = unix - millis() / 1000;
      clockTz = tz;
      char st[32];
      stampOf(millis(), bootNo, st, sizeof(st));
      out.printf("clock set: %s\n", st);
    }
  } else if (strcmp(cmd, "clear") == 0) {
    if (driving) { out.println("Not while driving"); return; }
    LittleFS.remove(LOG_FILE);
    LittleFS.remove(LOG_FILE_OLD);
    out.println("Log cleared");
  } else if (cmd[0] != 0) {
    out.println("Commands: log (print flash log), status (live state), sensors (raw sensor data), on/off <FFL|FL|L|M|R|FR|FFR|all>, scan (I2C bus), params, set kp|ki|kd|speed <v>, start <s>, stop, map [on|dynamic|off|reset], track [name|rename <new>], mark <text>, save, rounds [clear], name [robot], clear (delete log)");
  }
}

static void pollSerial(bool driving) {
  static char line[32];
  static uint8_t len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      line[len] = 0;
      handleCommand(line, driving, Serial);
      len = 0;
    } else if (len < sizeof(line) - 1) {
      line[len++] = c;
    }
  }
}

void logger_loop(const LogSnapshot &s) {
  static bool first = true;
  static LogSnapshot prev;
  static uint32_t lastHbMs = 0, lastFlushMs = 0, lastSlowLogMs = 0;
  static bool lowHeapLogged = false;

  logStage(STAGE_LOGGER);
  uint32_t now = millis();

  bb.uptimeMs = now;
  bb.loops++;
  bb.snap = s;
  if (s.cycleMs > bb.maxCycleMs) bb.maxCycleMs = s.cycleMs;

  uint32_t heap = ESP.getFreeHeap();
  bb.freeHeap = heap;
  if (heap < bb.minFreeHeap) bb.minFreeHeap = heap;
  if (heap < LOW_HEAP_BYTES && !lowHeapLogged) {
    lowHeapLogged = true;
    logEvent("LOW HEAP %lu bytes", (unsigned long)heap);
  }

  if (s.cycleMs >= SLOW_LOOP_MS && now - lastSlowLogMs >= 1000) {
    lastSlowLogMs = now;
    logEvent("slow loop %u ms", s.cycleMs);
  }

  // State changes
  if (first) {
    logEvent("state: sw=%u man=%u ble=%u", s.startSwitch, s.manual, s.connected);
  } else {
    if (s.connected != prev.connected)     logEvent(s.connected ? "BLE app connected" : "BLE app DISCONNECTED");
    if (s.startSwitch != prev.startSwitch) logEvent("main switch %s", s.startSwitch ? "ON" : "OFF");
    if (s.manual != prev.manual)           logEvent("mode -> %s", s.manual ? "manual" : "autonomous");
    if (s.driving != prev.driving) {
      if (s.driving) logEvent("driving started (track '%s')", speedmap_track_name());
      else logEvent("driving stopped");
    }
    // Start/stop events go to flash at once: a power-off right after a stop must not
    // erase why it stopped (2026-11-18: a stop 5 s into a drive left no trace)
    if (s.driving != prev.driving || s.startSwitch != prev.startSwitch) flushEvents(bootNo);
    if (s.reversing != prev.reversing)     logEvent(s.reversing ? "reversing" : "reverse done");
  }
  first = false;
  prev = s;

  // Heartbeat: "still alive" line in flash, so a total power loss still shows
  // roughly when it happened and what the robot was doing
  if (now - lastHbMs >= (s.driving ? HB_DRIVING_MS : HB_IDLE_MS)) {
    lastHbMs = now;
    flushEvents(bootNo);
    char snap[300];
    formatSnapshot(snap, sizeof(snap), bb);
    appendf(false, "HB %s", snap);
    bb.maxCycleMs = 0;  // max is per heartbeat interval
  }

  // While stopped, save events promptly (flash writes can stall the loop a bit,
  // so while driving they wait for the heartbeat)
  if (!s.driving && bb.evFlushed < bb.evTotal && now - lastFlushMs >= FLUSH_IDLE_MS) {
    lastFlushMs = now;
    flushEvents(bootNo);
  }

  pollSerial(s.driving);

  // Wi-Fi: commands from the laptop + live status stream
  const char *cmd = wifiLog_poll();
  if (cmd) {
    char ack[48];
    snprintf(ack, sizeof(ack), "# robot got command '%s'", cmd);
    wifiLog_send(ack);
    handleCommand(cmd, s.driving, wifiLog_printer());
    wifiLog_flush();
  }
  static uint32_t lastWifiStatusMs = 0;
  if (wifiLog_hasViewer() && now - lastWifiStatusMs >= WIFI_STATUS_MS) {
    lastWifiStatusMs = now;
    char line[320];
    int n = snprintf(line, sizeof(line), "S #%lu ", (unsigned long)bootNo);
    formatSnapshot(line + n, sizeof(line) - n, bb);
    wifiLog_send(line);
  }
}
