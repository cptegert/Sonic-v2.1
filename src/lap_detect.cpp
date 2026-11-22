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
  prevC = prevPrevC = 0;
  logEvent("lap fingerprint learned (laps so far %d)", laps);
}

void lap_update(bool driving, float leftCm, float rightCm, bool disturbed) {
  uint32_t now = millis();
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
    logEvent("LAP %d: %.2f s (match %.2f)", laps, lastLapS, prevC);
  }
  prevPrevC = prevC;
  prevC = c;
  prevSampleMs = now;
}

int lap_count() { return laps; }
float lap_last_s() { return lastLapS; }
