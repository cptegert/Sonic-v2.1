#include "battery.h"
#include "logger.h"
#include <Preferences.h>

static float calFactor = 1.0f;   // measured / computed, stored with "batcal"
static bool enabled = true;      // "battery off" for a robot without the divider (saved)
static float filtered = 0.0f;    // volts, low-pass filtered
static uint32_t lastPinMv = 0;
static uint32_t lastSampleMs = 0;
static bool lowLogged = false, criticalLogged = false;

static float pinToBattery(uint32_t pinMv) {
  return pinMv / 1000.0f * (BATTERY_R_TOP + BATTERY_R_BOTTOM) / BATTERY_R_BOTTOM * calFactor;
}

static uint32_t samplePinMv() {
  uint32_t sum = 0;
  for (int i = 0; i < 8; i++) sum += analogReadMilliVolts(BATTERY_PIN);  // factory-calibrated ADC
  return sum / 8;
}

void battery_begin() {
  analogSetPinAttenuation(BATTERY_PIN, ADC_11db);
  Preferences p;
  p.begin("battery", true);
  calFactor = p.getFloat("cal", 1.0f);
  enabled = p.getBool("enabled", true);
  p.end();
  if (!enabled) {
    filtered = 0.0f;
    logEvent("battery measurement OFF (no divider on this robot)");
    return;
  }
  if (calFactor < 0.8f || calFactor > 1.2f) calFactor = 1.0f;  // sanity
  lastPinMv = samplePinMv();
  filtered = pinToBattery(lastPinMv);
  if (battery_present()) logEvent("battery %.2f V", filtered);
  else logEvent("no battery voltage (USB only?) pin=%lu mV", (unsigned long)lastPinMv);
}

void battery_loop() {
  if (!enabled) return;
  uint32_t now = millis();
  if (now - lastSampleMs < 100) return;
  lastSampleMs = now;
  lastPinMv = samplePinMv();
  float v = pinToBattery(lastPinMv);
  // Low-pass: motor load makes the voltage dip briefly; ~1 s time constant
  filtered += 0.1f * (v - filtered);
  if (fabsf(v - filtered) > 3.0f) filtered = v;  // battery plugged in / out: jump

  if (!battery_present()) {
    lowLogged = criticalLogged = false;
    return;
  }
  // Log once per crossing, with 0.3 V hysteresis
  if (filtered < BATTERY_LOW_V && !lowLogged) {
    lowLogged = true;
    logEvent("LOW BATTERY %.2f V (%.2f V/cell)", filtered, filtered / 3);
  } else if (filtered > BATTERY_LOW_V + 0.3f) {
    lowLogged = false;
  }
  if (filtered < BATTERY_CRITICAL_V && !criticalLogged) {
    criticalLogged = true;
    logEvent("CRITICAL BATTERY %.2f V - charge it!", filtered);
  } else if (filtered > BATTERY_CRITICAL_V + 0.3f) {
    criticalLogged = false;
  }
}

float battery_volts() { return battery_present() ? filtered : 0.0f; }
bool battery_present() { return enabled && filtered > BATTERY_PRESENT_V; }
bool battery_low() { return battery_present() && filtered < BATTERY_LOW_V; }
bool battery_critical() { return battery_present() && filtered < BATTERY_CRITICAL_V; }

bool batteryCommand(const char *cmd, Print &out) {
  float measured;
  if (strcmp(cmd, "battery off") == 0 || strcmp(cmd, "battery on") == 0) {
    enabled = (cmd[9] == 'n');
    Preferences p;
    p.begin("battery", false);
    p.putBool("enabled", enabled);
    p.end();
    filtered = 0.0f;
    if (enabled) { lastPinMv = samplePinMv(); filtered = pinToBattery(lastPinMv); }
    logEvent("battery measurement %s (saved)", enabled ? "ON" : "OFF");
    out.printf("battery measurement %s (saved on this robot)\n", enabled ? "ON" : "OFF");
    return true;
  }
  if (strcmp(cmd, "battery") == 0) {
    if (!enabled) { out.println("battery: measurement OFF on this robot (battery on = enable)"); return true; }
    out.printf("battery: %.2f V (%.2f V/cell)  pin=%lu mV  cal=%.4f  %s\n",
               filtered, filtered / 3, (unsigned long)lastPinMv, calFactor,
               !battery_present() ? "NOT PRESENT (USB only?)" : battery_critical() ? "CRITICAL"
               : battery_low() ? "LOW" : "ok");
    return true;
  }
  if (sscanf(cmd, "batcal %f", &measured) == 1) {
    float computed = pinToBattery(lastPinMv) / calFactor;  // uncalibrated value
    if (computed < BATTERY_PRESENT_V || measured < 6 || measured > 13.5f) {
      out.println("batcal: battery must be connected, give the measured voltage, e.g. batcal 12.31");
      return true;
    }
    float f = measured / computed;
    if (f < 0.8f || f > 1.2f) {
      out.printf("batcal: factor %.3f out of range - check the wiring / resistor values\n", f);
      return true;
    }
    calFactor = f;
    Preferences p;
    p.begin("battery", false);
    p.putFloat("cal", calFactor);
    p.end();
    filtered = measured;
    logEvent("battery calibrated: factor %.4f", calFactor);
    out.printf("batcal: factor %.4f saved\n", calFactor);
    return true;
  }
  return false;
}
