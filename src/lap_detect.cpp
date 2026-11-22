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
static bool synced = true;    // false = saved fingerprint loaded, lap mark not found yet
static bool wrongWay = false; // set by you ("mark wrong"): nothing is learned meanwhile
static bool tmplProven = false; // fingerprint found again / gave 3 laps: worth searching for
static int tmplLaps = 0;
void speedmap_lap_done(bool consistent, bool troubleOnly);   // speed_map.cpp
float speedmap_saved_lap_s();                          // typical lap time saved with the track
static float typicalLapS();
bool speedmap_saved_template(float *out);               // fingerprint of this track
void speedmap_template_learned(const float *t);
const char *speedmap_track_name();
int speedmap_lap_margin();                              // average closest side distance per part, -1 = n/a
void speedmap_wheels_cleaned();

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
  char margin[16] = "";
  int m = speedmap_lap_margin();
  if (m >= 0) snprintf(margin, sizeof(margin), " margin%dcm", m);
  logEvent("LAPSUM %d %.2fs rev%d trap%d notmov%d blind%d minwall%dcm lock%d%% minspd%d%% spd%dus%s",
           lapNo, lapS, st.reverses, st.trapped, st.notMoving, st.blind, st.minWall,
           (int)(100 * st.lockLoops / n), (int)(100 * st.minSpeedLoops / n), (int)(st.speedSum / n), margin);
}

// ---------------------------------------------------------------------------
// Whole-lap signature (2026-11-22): steering + (right - left) of a regular lap,
// averaged over laps, 100 ms steps. Two trackers follow the robot along it: one
// assumes it drives forwards (its place moves forward 0-2 steps per sample), one
// assumes it drives the track BACKWARDS (time reversed, left/right swapped, place
// moves backwards). Each blends in how well the last 1.5 s fit there. Backwards
// clearly better than forwards for 0.5 s = "WRONG WAY?". Also gives the position
// from the sensors alone. LOGGING ONLY: nothing in the driving uses it (yet).
// Note: on a figure-8 the turns come in the same order backwards -> can't tell.
// ---------------------------------------------------------------------------
#define SIG_MAX 200            // samples = 20 s lap max
#define WW_WIN 15              // 1.5 s compared each step
#define WW_ALPHA 0.08f
// stored as int8 (steering x100, right-left in cm) to save RAM (heap is tight with
// Wi-Fi + Bluetooth): ~1.4 KB in all
static int8_t sigS[SIG_MAX], sigR[SIG_MAX];
static int sigN = 0;
static int8_t recS[SIG_MAX], recR[SIG_MAX];
static int recN = 0;
static bool recOk = false;
static float histS[WW_WIN], histR[WW_WIN];
static int histPos = 0, histN = 0;
static int16_t trF[SIG_MAX], trB[SIG_MAX];     // tracker scores x10000
static inline int8_t q8(float v) { return (int8_t)constrain(lroundf(v), -127, 127); }
static int trWarm = 0;
static int wwFwd = 0, wwRev = 0, wwPos = -1, wwCount = 0;
static bool wwDetected = false;

static void sigLapStart() { recN = 0; recOk = true; }

static void sigRecord(float st, float rl, bool disturbed) {
  if (!recOk) return;
  if (disturbed || recN >= SIG_MAX) { recOk = false; return; }
  recS[recN] = q8(st * 100);
  recR[recN] = q8(rl * 100);
  recN++;
}

static void trackerReset() {
  for (int k = 0; k < SIG_MAX; k++) trF[k] = trB[k] = 0;
  trWarm = 0;
  histN = 0;
  wwDetected = false;
  wwCount = 0;
}

static void sigLapDone(bool consistent) {
  if (consistent && recOk && recN >= 3 * WW_WIN) {
    if (sigN == 0 || abs(recN - sigN) > sigN / 6) {
      bool first = sigN == 0;
      memcpy(sigS, recS, recN);
      memcpy(sigR, recR, recN);
      sigN = recN;
      trackerReset();
      if (first) logEvent("lap signature learned (%d samples): wrong-way check on", sigN);
    } else {
      for (int i = 0; i < sigN; i++) {   // blend in, stretched to the signature length
        float x = i * (recN - 1) / (float)(sigN - 1);
        int a = (int)x;
        float f = x - a;
        int b = a + 1 < recN ? a + 1 : a;
        sigS[i] = q8(0.7f * sigS[i] + 0.3f * (recS[a] * (1 - f) + recS[b] * f));
        sigR[i] = q8(0.7f * sigR[i] + 0.3f * (recR[a] * (1 - f) + recR[b] * f));
      }
    }
  }
  sigLapStart();
}

// cosine similarity of the last WW_WIN samples with the signature at place k
static float segCos(int k, bool backwards) {
  float dot = 0, nw = 0, ns = 0;
  for (int j = 0; j < WW_WIN; j++) {
    int h = (histPos + j) % WW_WIN;                     // oldest first
    int idx = backwards ? (k + WW_WIN - 1 - j) : (k + j);
    idx %= sigN;
    float s1 = (backwards ? -sigS[idx] : sigS[idx]) * 0.01f;
    float s2 = (backwards ? -sigR[idx] : sigR[idx]) * 0.01f;
    dot += histS[h] * s1 + histR[h] * s2;
    nw += histS[h] * histS[h] + histR[h] * histR[h];
    ns += s1 * s1 + s2 * s2;
  }
  return (nw > 1e-6f && ns > 1e-6f) ? dot / sqrtf(nw * ns) : 0;
}

static void wrongWayCheck(float st, float rl, bool disturbed) {
  histS[histPos] = st;
  histR[histPos] = rl;
  histPos = (histPos + 1) % WW_WIN;
  if (histN < WW_WIN) histN++;
  if (sigN < 3 * WW_WIN || histN < WW_WIN) { wwFwd = wwRev = 0; wwPos = -1; return; }
  if (disturbed) return;                                // reversing / stuck: hold
  int n = sigN;
  int bfi = -32768, bbi = -32768, bk = 0;
  // forward tracker: new F[k] from old F[k], F[k-1], F[k-2] -> ascending, keep the old values
  int16_t o1 = trF[n - 1], o2 = trF[n - 2];          // old F[k-1], F[k-2] (wrap)
  for (int k = 0; k < n; k++) {
    int16_t old = trF[k];
    int16_t p = old > o1 ? old : o1;
    if (o2 > p) p = o2;
    trF[k] = (int16_t)((1 - WW_ALPHA) * p + WW_ALPHA * 10000 * segCos(k, false));
    if (trF[k] > bfi) { bfi = trF[k]; bk = k; }
    o2 = o1;
    o1 = old;
  }
  // backward tracker: new B[k] from old B[k], B[k+1], B[k+2] -> descending
  int16_t q1 = trB[0], q2 = trB[1];                   // old B[k+1], B[k+2] for k = n-1 (wrap)
  for (int k = n - 1; k >= 0; k--) {
    int16_t old = trB[k];
    int16_t p = old > q1 ? old : q1;
    if (q2 > p) p = q2;
    trB[k] = (int16_t)((1 - WW_ALPHA) * p + WW_ALPHA * 10000 * segCos(k, true));
    if (trB[k] > bbi) bbi = trB[k];
    q2 = q1;
    q1 = old;
  }
  float bf = bfi / 10000.0f, bb = bbi / 10000.0f;
  wwFwd = (int)(bf * 100);
  wwRev = (int)(bb * 100);
  wwPos = ((bk + WW_WIN - 1) % n) * 1000 / n;           // end of the window = now
  if (trWarm < 30) { trWarm++; return; }                // 3 s to settle
  if (!wwDetected) {
    wwCount = (bb > bf + WW_MARGIN) ? wwCount + 1 : 0;
    if (wwCount >= WW_SAMPLES) {
      wwDetected = true;
      wwCount = 0;
      logEvent("WRONG WAY? detected (back %.2f / fwd %.2f)", bb, bf);
    }
  } else {
    wwCount = (bf > bb + WW_MARGIN) ? wwCount + 1 : 0;
    if (wwCount >= WW_SAMPLES) {
      wwDetected = false;
      wwCount = 0;
      logEvent("right way again (detected: fwd %.2f / back %.2f)", bf, bb);
    }
  }
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
  synced = true;
  tmplProven = false;
  tmplLaps = 0;
  sigN = 0;            // new lap mark: the signature starts at the old one -> learn it again
  trackerReset();
  sigLapStart();
  lastLapMs = now;
  resetStats();  // lap statistics count from the fingerprint point
  prevC = prevPrevC = 0;
  logEvent("lap fingerprint learned (laps so far %d)", laps);
  speedmap_template_learned(tmpl);  // saved with the track: same lap mark next drive
}

void lap_update(const LapSample &s) {
  bool driving = s.driving;
  float leftCm = s.leftCm, rightCm = s.rightCm;
  bool disturbed = s.disturbed || wrongWay;
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
    wrongWay = false;
    trackerReset();
    recOk = false;
    // Fingerprint saved with this track: look for the SAME lap mark as last time,
    // so the speed map (parts counted from the lap mark) lines up again. Not found
    // within LAP_RELEARN_MS -> a new fingerprint is learned (and the map resets).
    haveTemplate = speedmap_saved_template(tmpl);
    synced = !haveTemplate;
    tmplProven = false;
    tmplLaps = 0;
    lastLapMs = now;
    if (haveTemplate) logEvent("looking for the saved lap mark of track '%s'", speedmap_track_name());
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
  wrongWayCheck(s.steer, (rightCm - leftCm) / 100.0f, s.disturbed);
  if (synced) sigRecord(s.steer, (rightCm - leftCm) / 100.0f, disturbed || wwDetected);

  // (re)learn the fingerprint: after the start, and when no lap for a long time
  // No lap for a while with a fingerprint that is known to work (pushed, turned
  // around, stuck): keep searching for the same lap mark before learning a new one,
  // because a new one moves the lap mark and the speed map has to start over.
  if (haveTemplate && synced && tmplProven && now - lastLapMs > LAP_RELEARN_MS) {
    synced = false;
    logEvent("lap mark lost (no lap for %lu s), looking for it", (unsigned long)((now - lastLapMs) / 1000));
  }
  if (!haveTemplate || now - lastLapMs > (tmplProven ? LAP_RELEARN_PROVEN_MS : LAP_RELEARN_MS)) {
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
  if (!synced && prevC > LAP_THRESHOLD && prevC >= c && prevC > prevPrevC) {
    synced = true;  // (saved / lost) lap mark found: laps count from here
    tmplProven = true;
    lastLapMs = prevSampleMs;
    resetStats();
    sigLapStart();
    logEvent("lap mark found (match %.2f)", prevC);
  } else if (synced && prevC > LAP_THRESHOLD && prevC >= c && prevC > prevPrevC && prevSampleMs - lastLapMs >= LAP_MIN_MS) {
    lastLapS = (prevSampleMs - lastLapMs) / 1000.0f;
    lastLapMs = prevSampleMs;
    laps++;
    if (++tmplLaps >= 3) tmplProven = true;
    float med = medianRecent();
    float typ = typicalLapS();     // before this lap counts
    if (typ > 0) lastConsistent = fabsf(lastLapS - typ) <= LAP_POS_TOLERANCE * typ;
    else lastConsistent = recentN >= LAP_POS_MIN_LAPS - 1 && med > 0 &&
                          fabsf(lastLapS - med) <= LAP_POS_TOLERANCE * med;
    // a lap made longer by trouble still tells WHERE the trouble was (the position is
    // measured from the lap mark): learn "slow down there", but nothing else
    bool troubleOnly = !lastConsistent && typ > 0 && lastLapS > (1.0f + LAP_POS_TOLERANCE) * typ;
    recent[recentN % 5] = lastLapS;
    recentN++;
    logEvent("LAP %d: %.2f s (match %.2f)", laps, lastLapS, prevC);
    logLapSummary(laps, lastLapS);
    resetStats();
    sigLapDone(lastConsistent);
    speedmap_lap_done(lastConsistent, troubleOnly);
  }
  prevPrevC = prevC;
  prevC = c;
  prevSampleMs = now;
}

int lap_count() { return laps; }
float lap_last_s() { return lastLapS; }

// Typical lap time: the recent laps if they agree with the time saved for this track
// (or there is none saved), else the saved one (crash laps make the recent ones long).
static float typicalLapS() {
  float saved = speedmap_saved_lap_s();
  if (recentN >= LAP_POS_MIN_LAPS) {
    float med = medianRecent();
    // the last 3 laps agree with each other -> that IS the lap time now (also after a
    // speed change); crash laps hardly ever agree with each other
    bool agree = true;
    for (int k = 1; k <= 3; k++) {
      float t = recent[(recentN - k) % 5];
      if (fabsf(t - med) > 0.15f * med) agree = false;
    }
    if (agree || saved <= 0 || fabsf(med - saved) <= 0.25f * saved) return med;
  }
  return saved > 0 ? saved : 0;
}

// Position = time since the lap mark / typical lap time. The lap mark is a precise
// anchor, so this works from the first lap mark on once the typical lap time is known
// (saved for the track) - it no longer needs 3 regular laps in a row (2026-11-22:
// crash-heavy drives never learned anything).
bool lap_position(float *phase) {
  if (!wasDriving || !haveTemplate || !synced || wrongWay) return false;
  float med = typicalLapS();
  if (med <= 0) return false;
  float p = (millis() - lastLapMs) / (med * 1000.0f);
  if (p > 1.0f + LAP_POS_TOLERANCE) return false;   // overdue: lost track of the position
  *phase = p > 0.999f ? 0.999f : p;
  return true;
}

bool lap_last_consistent() { return lastConsistent; }

float lap_median_s() { return recentN >= LAP_POS_MIN_LAPS ? medianRecent() : 0; }

bool lap_wrong_way() { return wrongWay; }

void lap_ww(int *fwd100, int *back100, int *pos1000, bool *detected) {
  *fwd100 = wwFwd; *back100 = wwRev; *pos1000 = wwPos; *detected = wwDetected;
}

int lap_phase_permille() {
  float ph;
  return lap_position(&ph) ? (int)(ph * 1000) : -1;
}

// "mark <text>": your note in the robot's log (from the laptop dashboard buttons).
// "mark wrong..." = driving the wrong way: no lap learning / speed map learning
// until "mark ok" (or "mark right...") or the next start.
bool lap_command(const char *cmd, Print &out) {
  if (strncmp(cmd, "mark", 4) != 0 || (cmd[4] != 0 && cmd[4] != ' ')) return false;
  const char *text = cmd[4] ? cmd + 5 : "";
  if (strncmp(text, "wrong", 5) == 0) wrongWay = true;
  else if (strncmp(text, "clean", 5) == 0) speedmap_wheels_cleaned();
  else if (strncmp(text, "ok", 2) == 0 || strncmp(text, "right", 5) == 0) wrongWay = false;
  logEvent("MARK %s%s", text[0] ? text : "-", wrongWay ? " (wrong way: not learning)" : "");
  out.printf("marked: %s\n", text);
  return true;
}
