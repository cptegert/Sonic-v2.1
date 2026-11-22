// Lap detection from sensor data only (for lap times in the log)
// -----------------------------------------------------------------------------
// 4 s after driving starts, the robot stores a 2.5 s "fingerprint" of its
// surroundings (right - left blended distance, sampled every 100 ms). From then
// on it compares the last 2.5 s with it (normalized cross-correlation). A strong
// match peak (> LAP_THRESHOLD) at least LAP_MIN_MS after the previous lap = one
// lap. No lap for LAP_RELEARN_MS -> learn a new fingerprint.
// Tuned 2026-11-22 on all recorded drives: ~2 % false laps, laps > 9 s are mostly
// real slow laps (stuck / pushed) or a missed match. For logging, not for
// "stop after N laps".
// -----------------------------------------------------------------------------
#ifndef LAP_DETECT_H
#define LAP_DETECT_H

#include <Arduino.h>

#define LAP_SAMPLE_MS    100
#define LAP_WINDOW       25      // samples = 2.5 s fingerprint
#define LAP_SETTLE_MS    4000    // skip the start
#define LAP_THRESHOLD    0.70f
#define LAP_MIN_MS       2500
#define LAP_RELEARN_MS   20000
#define LAP_MIN_STD_CM   4.0f    // fingerprint must have some shape

// Everything the lap module looks at, once per loop
struct LapSample {
  bool driving;
  float leftCm, rightCm;   // blended distances (fingerprint)
  bool disturbed;          // reversing / trapped / manual: no fingerprint taken then
  bool reversing;          // for counting reverse manoeuvres
  bool trapped;            // TRAPPED this loop
  int minDistCm;           // closest of the 7 raw sensors
  bool fullLock;           // steering at full lock
  bool minSpeed;           // speed command at the minimum
  int speedUs;             // motor command
};
void lap_update(const LapSample &s);

// Events from elsewhere that go into the lap summary (LAPSUM)
enum LapEvent : uint8_t { LAP_EV_NOT_MOVING, LAP_EV_BLIND };
void lap_note(LapEvent ev);
int lap_count();
float lap_last_s();     // last lap time in s, 0 = none yet

#endif
