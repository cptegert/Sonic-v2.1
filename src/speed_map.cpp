#include "speed_map.h"
#include "lap_detect.h"
#include "logger.h"
#include <Preferences.h>
#include <math.h>

// ---- current track (one of TRACK_SLOTS saved ones) ----
static char trackName[TRACK_NAME_LEN] = "default";
static int slot = 0;
static float factor[SPEEDMAP_PARTS];
static float mapLapS = 0;              // typical lap time the map was learned with
static float tmpl[LAP_WINDOW];         // lap fingerprint of this track
static bool haveTmpl = false;

static bool trouble[SPEEDMAP_PARTS];   // in the current lap
static bool enabled = true;
static bool dynamicUp = false;          // "map dynamic": also faster where it is safe
static float ceilF[SPEEDMAP_PARTS];     // dynamic: factor where trouble happened (minus a bit)
static bool dirty = false;             // learned something since the last save
static bool wasDriving = false;
static int currentPart = -1;
static bool trackChecked = false;      // compared with mapLapS in this drive
// Grip (2026-11-22): dirty wheels slide wider -> the same corner gets closer to the
// wall lap after lap. Per part: closest side distance this lap, and its usual value.
static int partMin[SPEEDMAP_PARTS];     // this lap, 999 = no data
static float partBase[SPEEDMAP_PARTS];  // usual closest distance (0 = not known yet)
static void resetPartMin() { for (int i = 0; i < SPEEDMAP_PARTS; i++) partMin[i] = 999; }

static void key(char *buf, char c, int i) { snprintf(buf, 8, "%c%d", c, i); }

static void resetMap() {
  for (int i = 0; i < SPEEDMAP_PARTS; i++) { factor[i] = 1.0f; ceilF[i] = SPEEDMAP_MAX_FACTOR; }
}

static bool learnedAny() {
  for (int i = 0; i < SPEEDMAP_PARTS; i++) if (fabsf(factor[i] - 1.0f) > 0.005f) return true;
  return false;
}

static void saveSlot() {
  Preferences p;
  p.begin("speedmap", false);
  char k[8];
  uint32_t use = p.getUInt("use", 0) + 1;
  p.putUInt("use", use);
  key(k, 'n', slot); p.putString(k, trackName);
  key(k, 'f', slot); p.putBytes(k, factor, sizeof(factor));
  key(k, 'l', slot); p.putFloat(k, mapLapS);
  key(k, 't', slot);
  if (haveTmpl) p.putBytes(k, tmpl, sizeof(tmpl)); else p.remove(k);
  key(k, 'u', slot); p.putUInt(k, use);
  p.putUChar("cur", slot);
  p.putBool("on", enabled);
  p.putBool("up", dynamicUp);
  p.end();
  dirty = false;
}

static void loadSlot(Preferences &p, int i) {
  char k[8];
  slot = i;
  resetMap();
  key(k, 'n', i);
  String n = p.getString(k, "");
  strlcpy(trackName, n.length() ? n.c_str() : "default", sizeof(trackName));
  key(k, 'f', i);
  if (p.getBytesLength(k) == sizeof(factor)) p.getBytes(k, factor, sizeof(factor));
  for (int j = 0; j < SPEEDMAP_PARTS; j++)  // sanity
    if (!(factor[j] >= SPEEDMAP_MIN_FACTOR && factor[j] <= SPEEDMAP_MAX_FACTOR)) factor[j] = 1.0f;
  key(k, 'l', i);
  mapLapS = p.getFloat(k, 0);
  key(k, 't', i);
  haveTmpl = p.getBytesLength(k) == sizeof(tmpl) && p.getBytes(k, tmpl, sizeof(tmpl)) == sizeof(tmpl);
  for (int j = 0; j < SPEEDMAP_PARTS; j++) { trouble[j] = false; partBase[j] = 0; }
  dirty = false;
}

void speedmap_begin() {
  resetPartMin();
  for (int i = 0; i < SPEEDMAP_PARTS; i++) partBase[i] = 0;
  Preferences p;
  p.begin("speedmap", false);
  enabled = p.getBool("on", true);
  dynamicUp = p.getBool("up", false);
  // firmware before named tracks: one map in "f"/"lap" -> becomes track "default"
  if (!p.isKey("n0") && p.isKey("f")) {
    float f[SPEEDMAP_PARTS];
    if (p.getBytesLength("f") == sizeof(f)) {
      p.getBytes("f", f, sizeof(f));
      p.putBytes("f0", f, sizeof(f));
    }
    p.putFloat("l0", p.getFloat("lap", 0));
    p.putString("n0", "default");
    p.remove("f");
    p.remove("lap");
  }
  int cur = p.getUChar("cur", 0);
  if (cur >= TRACK_SLOTS) cur = 0;
  loadSlot(p, cur);
  p.end();
}

const char *speedmap_track_name() { return trackName; }
int speedmap_mode() { return !enabled ? 0 : dynamicUp ? 2 : 1; }

// ---- lap fingerprint (lap_detect.cpp) ----
bool speedmap_saved_template(float *out) {
  if (!haveTmpl) return false;
  memcpy(out, tmpl, sizeof(tmpl));
  return true;
}

void speedmap_template_learned(const float *t) {
  // a NEW lap mark: the parts of the old map are counted from the old mark, so the
  // learned slow-downs would land in the wrong places -> start over
  if (haveTmpl && learnedAny()) {
    resetMap();
    logEvent("new lap mark on track '%s' -> its speed map starts over", trackName);
  }
  memcpy(tmpl, t, sizeof(tmpl));
  haveTmpl = true;
  dirty = true;
}

static void describe(char *buf, size_t len) {
  int n = snprintf(buf, len, "SPEEDMAP %s [%s]:", !enabled ? "OFF" : dynamicUp ? "dynamic" : "on", trackName);
  bool any = false;
  for (int i = 0; i < SPEEDMAP_PARTS && n < (int)len - 10; i++) {
    if (fabsf(factor[i] - 1.0f) > 0.005f) {
      n += snprintf(buf + n, len - n, " %d:%.2f", i + 1, factor[i]);
      any = true;
    }
  }
  if (!any) snprintf(buf + n, len - n, " nothing learned yet");
}

// called by lap_detect.cpp when a lap is completed
void speedmap_lap_done(bool consistent) {
  float med = lap_median_s();
  if (consistent && med > 0 && !trackChecked) {   // first regular laps of this drive
    trackChecked = true;
    bool learned = learnedAny();
    // (until 2026-11-22 this reset the map - but crash-heavy laps did that too;
    //  tracks are named now, so only a hint)
    if (learned && mapLapS > 0 && fabsf(med - mapLapS) > SPEEDMAP_NEW_TRACK * mapLapS)
      logEvent("laps %.2f s vs %.2f s when the map was learned: other track? (track <name>)", med, mapLapS);
    if (!learned || mapLapS <= 0 || fabsf(med - mapLapS) > SPEEDMAP_NEW_TRACK * mapLapS) mapLapS = med;
    dirty = true;
  }
  if (consistent && med > 0) mapLapS = 0.9f * mapLapS + 0.1f * med;  // follow slow drift
  if (consistent) {
    char drift[48] = "";
    int dn = 0;
    for (int i = 0; i < SPEEDMAP_PARTS; i++) {
      bool slid = false;
      if (partMin[i] < 999) {
        if (partBase[i] <= 0) partBase[i] = partMin[i];
        else {
          slid = !trouble[i] && partMin[i] < partBase[i] - GRIP_DROP_CM;
          partBase[i] += GRIP_BASE_ALPHA * (partMin[i] - partBase[i]);
        }
      }
      if (trouble[i]) {
        // too fast here: slow down; if it had been sped up, remember not to go that high again
        if (factor[i] > 1.0f) ceilF[i] = fmaxf(1.0f, factor[i] - SPEEDMAP_CEIL_MARGIN);
        factor[i] -= SPEEDMAP_STEP_DOWN;
      }
      else if (slid) {
        factor[i] -= GRIP_STEP_DOWN;   // getting closer to the wall than usual: less grip
        if (dn < 40) dn += snprintf(drift + dn, sizeof(drift) - dn, " %d", i + 1);
      }
      else if (factor[i] < 1.0f) factor[i] += SPEEDMAP_STEP_UP;   // recovering
      else if (dynamicUp && partMin[i] < 999 && partMin[i] >= SPEEDMAP_SAFE_CM && factor[i] < ceilF[i])
        factor[i] += SPEEDMAP_BOOST_STEP;                          // clean with room to spare: a bit faster
      if (factor[i] < SPEEDMAP_MIN_FACTOR) factor[i] = SPEEDMAP_MIN_FACTOR;
      float top = dynamicUp ? ceilF[i] : 1.0f;
      if (factor[i] > top) factor[i] = top;     // slow-down-only mode: never above 1.0
      if (factor[i] > SPEEDMAP_MAX_FACTOR) factor[i] = SPEEDMAP_MAX_FACTOR;
    }
    dirty = true;
    if (dn) logEvent("grip: closer to the wall than usual in part%s -> slower there", drift);
    char buf[120];
    describe(buf, sizeof(buf));
    logEvent("%s", buf);
  }
  for (int i = 0; i < SPEEDMAP_PARTS; i++) trouble[i] = false;
  resetPartMin();
}

int speedmap_lap_margin() {
  int sum = 0, n = 0;
  for (int i = 0; i < SPEEDMAP_PARTS; i++)
    if (partMin[i] < 999) { sum += partMin[i]; n++; }
  return n >= SPEEDMAP_PARTS / 2 ? sum / n : -1;
}

// "mark cleaned": wheels cleaned -> grip is back: slow parts recover most of the way
// at once (they learn again if a part is still a problem), usual distances relearned
void speedmap_wheels_cleaned() {
  for (int i = 0; i < SPEEDMAP_PARTS; i++) {
    if (factor[i] < 1.0f) factor[i] += GRIP_CLEAN_RECOVER * (1.0f - factor[i]);
    partBase[i] = 0;
  }
  dirty = true;
  char buf[120];
  describe(buf, sizeof(buf));
  logEvent("wheels cleaned -> %s", buf);
}

void speedmap_update(bool driving, int minSideCm, bool reversing, bool trapped, bool notMoving) {
  if (!driving) {
    if (wasDriving && dirty) saveSlot();  // keep what was learned (battery swap, power-off)
    wasDriving = false;
    trackChecked = false;
    currentPart = -1;
    return;
  }
  wasDriving = true;
  float phase;
  if (lap_wrong_way() || !lap_position(&phase)) { currentPart = -1; return; }
  currentPart = (int)(phase * SPEEDMAP_PARTS);
  if (currentPart >= SPEEDMAP_PARTS) currentPart = SPEEDMAP_PARTS - 1;
  if (minSideCm < SPEEDMAP_NEAR_CM || reversing || trapped || notMoving) trouble[currentPart] = true;
  if (!reversing && minSideCm < partMin[currentPart]) partMin[currentPart] = minSideCm;
}

float speedmap_factor() {
  if (!enabled || currentPart < 0) return 1.0f;
  float f = factor[currentPart];
  float next = factor[(currentPart + 1) % SPEEDMAP_PARTS];   // brake BEFORE the spot
  return f < next ? f : next;
}

int speedmap_part() { return currentPart; }

// ---- track command ----
static void listTracks(Print &out) {
  Preferences p;
  p.begin("speedmap", true);
  out.printf("track: %s  (lap %.2f s, fingerprint %s, map %s)\nsaved tracks:", trackName, mapLapS,
             haveTmpl ? "yes" : "no", learnedAny() ? "learned" : "empty");
  char k[8];
  for (int i = 0; i < TRACK_SLOTS; i++) {
    key(k, 'n', i);
    String n = p.getString(k, "");
    if (n.length()) out.printf(" %s%s", n.c_str(), i == slot ? "*" : "");
  }
  out.println();
  p.end();
}

static bool validName(const char *n) {
  if (!n[0] || strlen(n) >= TRACK_NAME_LEN) return false;
  for (const char *c = n; *c; c++)
    if (!isalnum((unsigned char)*c) && *c != '-' && *c != '_') return false;
  return true;
}

static void selectTrack(const char *name, Print &out) {
  if (strcmp(name, trackName) == 0) { listTracks(out); return; }
  if (dirty) saveSlot();
  Preferences p;
  p.begin("speedmap", false);
  char k[8];
  int found = -1, empty = -1, lru = 0;
  uint32_t lruUse = UINT32_MAX;
  for (int i = 0; i < TRACK_SLOTS; i++) {
    key(k, 'n', i);
    String n = p.getString(k, "");
    if (n == name) found = i;
    else if (!n.length() && empty < 0) empty = i;
    key(k, 'u', i);
    uint32_t u = p.getUInt(k, 0);
    if (i != slot && u < lruUse) { lruUse = u; lru = i; }
  }
  if (found >= 0) {
    loadSlot(p, found);
    p.end();
    logEvent("track -> '%s' (known: lap %.2f s, fingerprint %s)", trackName, mapLapS, haveTmpl ? "yes" : "no");
  } else {
    int s = empty >= 0 ? empty : lru;
    key(k, 'n', s);
    String old = p.getString(k, "");
    p.end();
    slot = s;
    strlcpy(trackName, name, sizeof(trackName));
    resetMap();
    mapLapS = 0;
    haveTmpl = false;
    for (int j = 0; j < SPEEDMAP_PARTS; j++) { trouble[j] = false; partBase[j] = 0; }
    if (old.length()) logEvent("track memory full: '%s' forgotten", old.c_str());
    logEvent("track -> '%s' (new: learning from scratch)", trackName);
  }
  saveSlot();  // also remembers it as the current track
  listTracks(out);
}

bool speedmap_command(const char *cmd, bool driving, Print &out) {
  if (strncmp(cmd, "track", 5) == 0 && (cmd[5] == 0 || cmd[5] == ' ')) {
    const char *arg = cmd[5] ? cmd + 6 : "";
    if (!arg[0]) { listTracks(out); return true; }
    if (driving) { out.println("track: not while driving"); return true; }
    if (strncmp(arg, "rename ", 7) == 0) {
      const char *nn = arg + 7;
      if (!validName(nn)) { out.printf("track: name = letters, digits, - or _, max %d\n", TRACK_NAME_LEN - 1); return true; }
      Preferences p;
      p.begin("speedmap", true);
      char k[8];
      bool taken = false;
      for (int i = 0; i < TRACK_SLOTS; i++) {
        key(k, 'n', i);
        if (i != slot && p.getString(k, "") == nn) taken = true;
      }
      p.end();
      if (taken) { out.printf("track: '%s' exists already\n", nn); return true; }
      logEvent("track renamed '%s' -> '%s'", trackName, nn);
      strlcpy(trackName, nn, sizeof(trackName));
      saveSlot();
      listTracks(out);
      return true;
    }
    if (!validName(arg)) { out.printf("track: name = letters, digits, - or _, max %d\n", TRACK_NAME_LEN - 1); return true; }
    selectTrack(arg, out);
    return true;
  }
  if (strncmp(cmd, "map", 3) != 0 || (cmd[3] != 0 && cmd[3] != ' ')) return false;
  const char *arg = cmd[3] ? cmd + 4 : "";
  if (strcmp(arg, "on") == 0) {
    enabled = true; dynamicUp = false;
    for (int i = 0; i < SPEEDMAP_PARTS; i++) if (factor[i] > 1.0f) factor[i] = 1.0f;   // no boosts
    saveSlot(); logEvent("speed map ON (slow down only)");
  }
  else if (strcmp(arg, "dynamic") == 0) { enabled = true; dynamicUp = true; saveSlot(); logEvent("speed map DYNAMIC (slower + faster)"); }
  else if (strcmp(arg, "off") == 0) { enabled = false; saveSlot(); logEvent("speed map OFF"); }
  else if (strcmp(arg, "reset") == 0) {
    if (driving) { out.println("map reset: not while driving"); return true; }
    resetMap(); mapLapS = 0; haveTmpl = false; saveSlot();
    for (int i = 0; i < SPEEDMAP_PARTS; i++) partBase[i] = 0;
    logEvent("speed map + lap mark of track '%s' reset", trackName);
  }
  else if (arg[0]) { out.println("map | map on | map dynamic | map off | map reset | track [name] | track rename <new>"); return true; }
  char buf[120];
  describe(buf, sizeof(buf));
  out.println(buf);
  out.print("factors:");
  for (int i = 0; i < SPEEDMAP_PARTS; i++) out.printf(" %.2f", factor[i]);
  out.printf("\nmap learned at lap %.2f s; now: part %d, factor %.2f\n", mapLapS, currentPart + 1, speedmap_factor());
  return true;
}
