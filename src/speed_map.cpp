#include "speed_map.h"
#include "lap_detect.h"
#include "logger.h"
#include <Preferences.h>
#include <math.h>

static float factor[SPEEDMAP_PARTS];
static bool trouble[SPEEDMAP_PARTS];   // in the current lap
static bool enabled = true;
static bool dirty = false;             // learned something since the last save
static bool wasDriving = false;
static int currentPart = -1;
static float mapLapS = 0;              // typical lap time the map was learned with
static bool trackChecked = false;      // compared with mapLapS in this drive

static void resetMap() {
  for (int i = 0; i < SPEEDMAP_PARTS; i++) factor[i] = 1.0f;
}

static void save() {
  Preferences p;
  p.begin("speedmap", false);
  p.putBytes("f", factor, sizeof(factor));
  p.putBool("on", enabled);
  p.putFloat("lap", mapLapS);
  p.end();
  dirty = false;
}

void speedmap_begin() {
  resetMap();
  Preferences p;
  p.begin("speedmap", true);
  if (p.getBytesLength("f") == sizeof(factor)) p.getBytes("f", factor, sizeof(factor));
  enabled = p.getBool("on", true);
  mapLapS = p.getFloat("lap", 0);
  p.end();
  for (int i = 0; i < SPEEDMAP_PARTS; i++) {  // sanity
    if (!(factor[i] >= SPEEDMAP_MIN_FACTOR && factor[i] <= 1.0f)) factor[i] = 1.0f;
  }
  for (int i = 0; i < SPEEDMAP_PARTS; i++) trouble[i] = false;
}

static void describe(char *buf, size_t len) {
  int n = snprintf(buf, len, "SPEEDMAP %s:", enabled ? "on" : "OFF");
  bool any = false;
  for (int i = 0; i < SPEEDMAP_PARTS && n < (int)len - 10; i++) {
    if (factor[i] < 0.995f) {
      n += snprintf(buf + n, len - n, " %d:%.2f", i + 1, factor[i]);
      any = true;
    }
  }
  if (!any) snprintf(buf + n, len - n, " no slow parts");
}

// called by lap_detect.cpp when a lap is completed
void speedmap_lap_done(bool consistent) {
  float med = lap_median_s();
  if (consistent && med > 0 && !trackChecked) {   // first regular laps of this drive
    trackChecked = true;
    bool learned = false;
    for (int i = 0; i < SPEEDMAP_PARTS; i++) learned |= factor[i] < 0.995f;
    if (learned && mapLapS > 0 && fabsf(med - mapLapS) > SPEEDMAP_NEW_TRACK * mapLapS) {
      logEvent("new track? lap %.2f s vs map %.2f s -> speed map reset", med, mapLapS);
      resetMap();
    }
    if (!learned || mapLapS <= 0 || fabsf(med - mapLapS) > SPEEDMAP_NEW_TRACK * mapLapS) mapLapS = med;
    dirty = true;
  }
  if (consistent && med > 0) mapLapS = 0.9f * mapLapS + 0.1f * med;  // follow slow drift
  if (consistent) {
    for (int i = 0; i < SPEEDMAP_PARTS; i++) {
      if (trouble[i]) factor[i] -= SPEEDMAP_STEP_DOWN;
      else factor[i] += SPEEDMAP_STEP_UP;
      if (factor[i] < SPEEDMAP_MIN_FACTOR) factor[i] = SPEEDMAP_MIN_FACTOR;
      if (factor[i] > 1.0f) factor[i] = 1.0f;
    }
    dirty = true;
    char buf[96];
    describe(buf, sizeof(buf));
    logEvent("%s", buf);
  }
  for (int i = 0; i < SPEEDMAP_PARTS; i++) trouble[i] = false;
}

void speedmap_update(bool driving, int minSideCm, bool reversing, bool trapped, bool notMoving) {
  if (!driving) {
    if (wasDriving && dirty) save();  // keep what was learned (battery swap, power-off)
    wasDriving = false;
    trackChecked = false;
    currentPart = -1;
    return;
  }
  wasDriving = true;
  float phase;
  if (!lap_position(&phase)) { currentPart = -1; return; }
  currentPart = (int)(phase * SPEEDMAP_PARTS);
  if (currentPart >= SPEEDMAP_PARTS) currentPart = SPEEDMAP_PARTS - 1;
  if (minSideCm < SPEEDMAP_NEAR_CM || reversing || trapped || notMoving) trouble[currentPart] = true;
}

float speedmap_factor() {
  if (!enabled || currentPart < 0) return 1.0f;
  float f = factor[currentPart];
  float next = factor[(currentPart + 1) % SPEEDMAP_PARTS];   // brake BEFORE the spot
  return f < next ? f : next;
}

int speedmap_part() { return currentPart; }

bool speedmap_command(const char *cmd, Print &out) {
  if (strncmp(cmd, "map", 3) != 0 || (cmd[3] != 0 && cmd[3] != ' ')) return false;
  const char *arg = cmd[3] ? cmd + 4 : "";
  if (strcmp(arg, "on") == 0) { enabled = true; save(); logEvent("speed map ON"); }
  else if (strcmp(arg, "off") == 0) { enabled = false; save(); logEvent("speed map OFF"); }
  else if (strcmp(arg, "reset") == 0) { resetMap(); mapLapS = 0; save(); logEvent("speed map reset"); }
  else if (arg[0]) { out.println("map | map on | map off | map reset"); return true; }
  char buf[96];
  describe(buf, sizeof(buf));
  out.println(buf);
  out.print("factors:");
  for (int i = 0; i < SPEEDMAP_PARTS; i++) out.printf(" %.2f", factor[i]);
  out.printf("\nmap learned at lap %.2f s; now: part %d, factor %.2f\n", mapLapS, currentPart + 1, speedmap_factor());
  return true;
}
