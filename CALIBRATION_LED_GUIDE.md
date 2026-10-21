# MPU6050 & Compass Calibration LED Guide

This guide explains the **Neopixel LED color patterns** (WS2812 addressable LED on pin 14) during sensor calibration.

## Starting Calibration

**How to start:** Press the "Calibrate mpu/compass" button in the RemoteXY app

⚠️ **IMPORTANT:** Calibration takes ~35 seconds and blocks all other operations. Always calibrate with:
- Main switch **OFF** (robot stationary)
- Robot on a flat, stable surface
- No active driving or sensor operations

If you try to calibrate while the robot is active, you'll see an error message.

---

## LED Color Pattern Reference

### 1️⃣ MPU6050 Calibration (Accelerometer & Gyroscope)

#### **🔵 BLUE - 3 Slow Blinks** (800ms on, 200ms off)
- **Color:** Blue
- **Meaning:** Countdown before calibration starts
- **Action Required:** Place robot on a FLAT, LEVEL surface and keep it COMPLETELY STILL

#### **🐬 CYAN - Solid ON** (continuous)
- **Color:** Cyan (light blue)
- **Meaning:** Collecting 1000 sensor samples (~2 seconds)
- **Action Required:** DO NOT MOVE THE ROBOT!

#### **🟢 GREEN - 2 Quick Blinks** (200ms on, 200ms off)
- **Color:** Green
- **Meaning:** MPU6050 calibration complete ✓
- **Next:** Compass calibration will begin automatically

#### **🔴 RED - 10 Rapid Blinks** (100ms on/off)
- **Color:** Red
- **Meaning:** ERROR - MPU6050 sensor not detected
- **Action Required:** Check sensor wiring and restart

---

### 2️⃣ Compass Calibration (Magnetometer)

#### **🔵 BLUE - 3 Slow Blinks** (800ms on, 200ms off)
- **Color:** Blue
- **Meaning:** Countdown before calibration starts
- **Action Required:** Get ready to rotate the robot

#### **💜 MAGENTA - Fast Pulsing** (300ms cycle)
- **Color:** Magenta (purple/pink)
- **Meaning:** Calibration in progress (30 seconds)
- **Action Required:** Rotate robot slowly in ALL directions:
  - 🔄 Rotate 360° horizontally (yaw)
  - ⬆️⬇️ Tilt forward/backward (pitch)
  - ⬅️➡️ Tilt left/right (roll)
- **Progress:** Watch Serial Monitor for countdown and sample count

#### **🟢 GREEN - 3 Quick Blinks** (200ms on, 200ms off)
- **Color:** Green
- **Meaning:** GOOD calibration quality! All axes > 5000 range
- **Result:** Calibration values are reliable

#### **🟠 ORANGE - 5 Slow Blinks** (500ms on, 300ms off)
- **Color:** Orange
- **Meaning:** POOR calibration quality - insufficient rotation
- **Action Required:** Consider recalibrating with more movement
- **Note:** Values are still saved but may be less accurate

---

### 3️⃣ Save to EEPROM

#### **🟡 YELLOW - Solid ON** (brief)
- **Color:** Yellow
- **Meaning:** Saving calibration values to EEPROM
- **Action:** Wait for completion pattern

#### **🟢 GREEN - Long-Short-Long Pattern**
- **Color:** Green
- **Pattern:** ON (1s) → OFF (0.2s) → ON (1s) → OFF
- **Meaning:** All calibration values saved to EEPROM ✓
- **Result:** Values will auto-load on every boot

---

## Color Summary

| Color | Meaning |
|-------|---------|
| 🔵 **Blue** | Countdown / Get Ready |
| 🐬 **Cyan** | Collecting MPU6050 data (stay still!) |
| 💜 **Magenta** | Collecting compass data (rotate!) |
| 🟢 **Green** | Success / Complete |
| 🟡 **Yellow** | Saving to EEPROM |
| 🟠 **Orange** | Warning / Poor quality |
| 🔴 **Red** | Error

---

## Calibration Checklist

### MPU6050 (Keep Still)
- [ ] Robot on flat, level surface
- [ ] No vibrations
- [ ] Wait for 3 slow blinks
- [ ] DO NOT MOVE during solid LED

### Compass (Move in All Directions)
- [ ] Wait for 3 slow blinks
- [ ] Rotate horizontally 360° slowly
- [ ] Tilt forward and backward
- [ ] Tilt left and right
- [ ] Continue for full 30 seconds (watch LED pulsing)
- [ ] Check Serial Monitor for quality report

### Verification
- [ ] Final long-short-long LED pattern seen
- [ ] Serial Monitor shows "Calibration Complete!"
- [ ] Check Serial Monitor for quality metrics:
  - X, Y, Z ranges should be > 5000 for "OK"
  - Lower values = "POOR" = needs recalibration

---

## Troubleshooting

| LED Pattern | Problem | Solution |
|------------|---------|----------|
| 10 rapid blinks | MPU6050 not detected | Check I2C wiring (SDA/SCL) |
| No LED activity | Button not working | Check RemoteXY connection |
| 5 slow blinks after compass | Poor calibration | Recalibrate with more rotation |
| LED stuck solid | Code frozen | Reset ESP32 |

---

## Serial Monitor Output

Connect to Serial Monitor (115200 baud) for detailed information:
- Real-time calibration progress
- Calculated offset values
- Compass quality metrics (X/Y/Z ranges)
- Sample counts
- Success/error messages

---

## Notes

- **All timing uses `millis()` - non-blocking**
- **Calibration takes ~35 seconds total**
  - MPU6050: ~5 seconds
  - Compass: ~30 seconds
- **Values persist across reboots**
- **Recalibrate anytime by pressing the button again**
- **Different magnetic environments may require recalibration**

---

*Generated for Sonic-X Racing Robot - ESP32 Platform*
