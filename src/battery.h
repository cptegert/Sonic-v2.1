// Battery voltage monitoring (3S LiPo) through a resistor divider on GPIO 34
// -----------------------------------------------------------------------------
//   Battery + (after main switch) --[82k]--+--[10k]-- GND
//                                          +--[1uF]-- GND
//                                          +--------- GPIO 34 (ADC1, input only)
// 12.6 V (full) -> ~1.37 V at the pin, 9 V -> ~0.98 V.
// Calibrate once against a multimeter / LiPo checker:  batcal <measured volts>
// -----------------------------------------------------------------------------
#ifndef BATTERY_H
#define BATTERY_H

#include <Arduino.h>

#define BATTERY_PIN        34
#define BATTERY_R_TOP      82000.0f  // ohm (battery side)
#define BATTERY_R_BOTTOM   10000.0f  // ohm (GND side)
#define BATTERY_LOW_V      10.5f     // ~3.5 V/cell: warning
#define BATTERY_CRITICAL_V 9.9f      // ~3.3 V/cell: remote runs refused / stopped
#define BATTERY_PRESENT_V  7.0f      // below this = no battery (USB only: ~5 V leaks in), no warnings

void battery_begin();
void battery_loop();                    // call every loop (samples every 100 ms)
float battery_volts();                  // filtered battery voltage (0 if not present)
bool battery_present();
bool battery_low();                     // below BATTERY_LOW_V (and present)
bool battery_critical();                // below BATTERY_CRITICAL_V (and present)
bool batteryCommand(const char *cmd, Print &out);  // "battery", "batcal <V>"

#endif
