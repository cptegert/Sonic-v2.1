#include "lap_detect.h"
#include "logger.h"
#include <math.h>

static float buf[LAP_WINDOW];         // last LAP_WINDOW samples of (right - left), ring
static bool  bufDisturbed[LAP_WINDOW];
static int   bufPos = 0, bufCount = 0;
static float tmpl[LAP_WINDOW];         // fingerprint (normalized: mean 0, length 1)
static bool  haveTemplate = false;
static uint32_t driveStartMs = 0, lastSampleMs = 0, lastLapMs = 0;
static float prevC = 0, prevPrevC = 0;
static uint32_t prevSampleMs = 0;
static int laps = 0;
static float lastLapS = 0;
static bool wasDriving = false;
static float recent[5];       // last lap times for the median
static int recentN = 0;
static bool lastConsistent = false;
void speedmap_lap_done(bool consistent);   // speed_map.cpp

static float medianRecent() {
  float t[5];
  int n = recentN < 5 ? recentN : 5;
  for (int i = 0; i < n; i++) t[i] = recent[i];
  for (int i = 1; i < n; i++) for (int j = i; j > 0 && t[j] < t[j - 1]; j--) { float x = t[j]; t[j] = t[j - 1]; t[j - 1] = x; }
  return n ? t[n / 2] : 0;
}

// Per-lap statistics -> "LAPSUM" line at every lap
struct LapStats {
  uint32_t loops, lockLoops, minSpeedLoops;
  uint64_t speedSum;
  int reverses, trapped, notMoving, blind, minWall;
};
static LapStats st;
static bool prevReversing = false, prevTrapped = false;
static void resetStats() { memset(&st, 0, sizeof(st)); st.minWall = 999; }

void lap_note(LapEvent ev) {
  if (ev == LAP_EV_NOT_MOVING) st.notMoving++;
  else if (ev == LAP_EV_BLIND) st.blind++;
}

static void logLapSummary(int lapNo, float lapS) {
  uint32_t n = st.loops ? st.loops : 1;
  logEvent("LAPSUM %d %.2fs rev%d trap%d notmov%d blind%d minwall%dcm lock%d%% minspd%d%% spd%dus",
           lapNo, lapS, st.reverses, st.trapped, st.notMoving, st.blind, st.minWall,
           (int)(100 * st.lockLoops / n), (int)(100 * st.minSpeedLoops / n), (int)(st.speedSum / n));
}

// window in chronological order, normalized (mean 0, unit length); false if flat
static bool normalizedWindow(float *out) {
  float mean = 0;
  for (int i = 0; i < LAP_WINDOW; i++) {
    out[i] = buf[(bufPos + i) % LAP_WINDOW];  // bufPos = oldest
    mean += out[i];
  }
  mean /= LAP_WINDOW;
  float ss = 0;
  for (int i = 0; i < LAP_WINDOW; i++) { out[i] -= mean; ss += out[i] * out[i]; }
  if (ss < 1e-6f) return false;
  float inv = 1.0f / sqrtf(ss);
  for (int i = 0; i < LAP_WINDOW; i++) out[i] *= inv;
  return true;
}

static void learnTemplate(uint32_t now) {
  bool disturbed = false;
  for (int i = 0; i < LAP_WINDOW; i++) disturbed |= bufDisturbed[i];
  if (disturbed) return;
  float w[LAP_WINDOW];
  float mean = 0, ss = 0;
  for (int i = 0; i < LAP_WINDOW; i++) mean += buf[i];
  mean /= LAP_WINDOW;
  for (int i = 0; i < LAP_WINDOW; i++) ss += (buf[i] - mean) * (buf[i] - mean);
  if (sqrtf(ss / LAP_WINDOW) < LAP_MIN_STD_CM) return;  // too flat to recognise
  if (!normalizedWindow(w)) return;
  memcpy(tmpl, w, sizeof(tmpl));
  haveTemplate = true;
  lastLapMs = now;
  resetStats();  // lap statistics count from the fingerprint point
  prevC = prevPrevC = 0;
  logEvent("lap fingerprint learned (laps so far %d)", laps);
}

void lap_update(const LapSample &s) {
  bool driving = s.driving;
  float leftCm = s.leftCm, rightCm = s.rightCm;
  bool disturbed = s.disturbed;
  uint32_t now = millis();
  if (driving) {  // statistics every loop
    st.loops++;
    if (s.fullLock) st.lockLoops++;
    if (s.minSpeed) st.minSpeedLoops++;
    st.speedSum += s.speedUs;
    if (s.minDistCm < st.minWall) st.minWall = s.minDistCm;
    if (s.reversing && !prevReversing) st.reverses++;
    if (s.trapped && !prevTrapped) st.trapped++;
    prevReversing = s.reversing;
    prevTrapped = s.trapped;
  }
  if (!driving) {
    if (wasDriving && laps > 0) logEvent("laps this run: %d, last %.2f s", laps, lastLapS);
    wasDriving = false;
    return;
  }
  if (!wasDriving) {  // new run: start over
    wasDriving = true;
    driveStartMs = now;
    lastSampleMs = now - LAP_SAMPLE_MS;
    bufPos = bufCount = 0;
    haveTemplate = false;
    laps = 0;
    lastLapS = 0;
    prevC = prevPrevC = 0;
    recentN = 0;
    lastConsistent = false;
    resetStats();
  }
  if (now - lastSampleMs < LAP_SAMPLE_MS) return;
  lastSampleMs = now;

  buf[bufPos] = rightCm - leftCm;
  bufDisturbed[bufPos] = disturbed;
  bufPos = (bufPos + 1) % LAP_WINDOW;
  if (bufCount < LAP_WINDOW) { bufCount++; return; }

  // (re)learn the fingerprint: after the start, and when no lap for a long time
  if (!haveTemplate || now - lastLapMs > LAP_RELEARN_MS) {
    if (now - driveStartMs >= LAP_SETTLE_MS) {
      bool had = haveTemplate;
      haveTemplate = false;
      learnTemplate(now);
      if (!haveTemplate && had) { haveTemplate = true; lastLapMs = now; }  // keep old one, retry later
    }
    if (!haveTemplate || lastLapMs == now) return;
  }

  float w[LAP_WINDOW];
  float c = 0;
  if (normalizedWindow(w)) {
    for (int i = 0; i < LAP_WINDOW; i++) c += w[i] * tmpl[i];
  }
  // peak of the match one sample ago -> lap at that moment
  if (prevC > LAP_THRESHOLD && prevC >= c && prevC > prevPrevC && prevSampleMs - lastLapMs >= LAP_MIN_MS) {
    lastLapS = (prevSampleMs - lastLapMs) / 1000.0f;
    lastLapMs = prevSampleMs;
    laps++;
    float med = medianRecent();
    lastConsistent = recentN >= LAP_POS_MIN_LAPS - 1 && med > 0 &&
                     fabsf(lastLapS - med) <= LAP_POS_TOLERANCE * med;
    recent[recentN % 5] = lastLapS;
    recentN++;
    logEvent("LAP %d: %.2f s (match %.2f)", laps, lastLapS, prevC);
    logLapSummary(laps, lastLapS);
    resetStats();
    speedmap_lap_done(lastConsistent);
  }
  prevPrevC = prevC;
  prevC = c;
  prevSampleMs = now;
}

int lap_count() { return laps; }
float lap_last_s() { return lastLapS; }

bool lap_position(float *phase) {
  if (!wasDriving || !haveTemplate || laps < LAP_POS_MIN_LAPS || !lastConsistent) return false;
  float med = medianRecent();
  if (med <= 0) return false;
  float p = (millis() - lastLapMs) / (med * 1000.0f);
  if (p > 1.0f + LAP_POS_TOLERANCE) return false;   // overdue: lost track of the position
  *phase = p > 0.999f ? 0.999f : p;
  return true;
}

bool lap_last_consistent() { return lastConsistent; }

float lap_median_s() { return recentN >= LAP_POS_MIN_LAPS ? medianRecent() : 0; }
