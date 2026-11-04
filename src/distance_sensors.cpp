#include "distance_sensors.h"
#include "globals.h"  // For signal_rate_minimal
#include <Arduino.h>
#include <VL53L1X.h>
#include "logger.h"

// ============================================================================
// SENSOR OBJECTS
// ============================================================================

VL53L1X sensorFarFarLeft, sensorFarLeft, sensorLeft, sensorMiddle, sensorRight, sensorFarRight, sensorFarFarRight;

// Init result per sensor [FFL, FL, L, M, R, FR, FFR]. A sensor that failed to
// start is not read (would only cost I2C time + errors) and reports maxDistance.
static bool sensorOk[7] = {false, false, false, false, false, false, false};

// Health tracking (see distance_sensors.h)
static uint32_t lastNewDataMs[7] = {0, 0, 0, 0, 0, 0, 0};
static bool failingNow[7] = {false, false, false, false, false, false, false};
static int failingCount = 0;
static int badLoops = 0;

// Which sensors to use [FFL, FL, L, M, R, FR, FFR]. A disabled sensor is kept
// powered down (XSHUT LOW), so it is completely off the I2C bus, and reads maxDistance.
// Changeable at runtime with the "on <name>" / "off <name>" commands (not saved).
static bool sensorEnabled[7] = {true, true, true, true, true, true, true};
// Test switch: initialize the sensor (on the bus, has its address) but stop its laser.
// 2026-11-04: FFR only ranges correctly while FFL's laser is off -> FFL's laser
// pulses disturb FFR (optical, or supply noise via shared power/ground wiring).
static const bool sensorLaserOff[7] = {false, false, false, false, false, false, false};

// ============================================================================
// CALIBRATION DATA
// ============================================================================

const float scalingFactors[7] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
const int offsets[7] = {0, 0, 0, 0, 0, 0, 0};

// ============================================================================
// DISTANCE DATA
// ============================================================================

int distances[3] = {0, 0, 0};         // Blended output: [left, middle, right]
int distancesAll[7] = {0,0,0,0,0,0,0}; // All 7 raw sensor readings

// Maximum distance threshold: readings beyond this are capped
// Set to 130cm (1.3m) as requested - reasonable for indoor robot navigation
int maxDistance = 130;

// ============================================================================
// SIGNAL QUALITY METRICS
// ============================================================================

float signalRate = 0.0;        // Current sensor signal strength (MCPS)
float middle_signal_rate = 0.0; // Middle sensor signal rate (for display)
// Note: signal_rate_minimal is defined in globals.cpp

// ============================================================================
// INITIALIZATION FUNCTIONS
// ============================================================================

void initialize_Distance_Sensors() {
    Serial.println("🔧 Initializing distance sensors...");

    // Configure all XSHUT pins as outputs
    pinMode(XSHUT_FAR_FAR_LEFT, OUTPUT);
    pinMode(XSHUT_FAR_LEFT,     OUTPUT);
    pinMode(XSHUT_LEFT,         OUTPUT);
    pinMode(XSHUT_MIDDLE,       OUTPUT);
    pinMode(XSHUT_RIGHT,        OUTPUT);
    pinMode(XSHUT_FAR_RIGHT,    OUTPUT);
    pinMode(XSHUT_FAR_FAR_RIGHT, OUTPUT);

    // Shut down all sensors to reset I2C addresses
    digitalWrite(XSHUT_FAR_FAR_LEFT, LOW);
    digitalWrite(XSHUT_FAR_LEFT,     LOW);
    digitalWrite(XSHUT_LEFT,         LOW);
    digitalWrite(XSHUT_MIDDLE,       LOW);
    digitalWrite(XSHUT_RIGHT,        LOW);
    digitalWrite(XSHUT_FAR_RIGHT,    LOW);
    digitalWrite(XSHUT_FAR_FAR_RIGHT, LOW);
    delay(20);

    Serial.println("  Initializing VL53L1X sensors...");
    VL53L1X *objs[7] = {&sensorFarFarLeft, &sensorFarLeft, &sensorLeft, &sensorMiddle,
                        &sensorRight, &sensorFarRight, &sensorFarFarRight};
    const int xshut[7] = {XSHUT_FAR_FAR_LEFT, XSHUT_FAR_LEFT, XSHUT_LEFT, XSHUT_MIDDLE,
                          XSHUT_RIGHT, XSHUT_FAR_RIGHT, XSHUT_FAR_FAR_RIGHT};
    const uint8_t addr[7] = {ADDRESS_FAR_FAR_LEFT, ADDRESS_FAR_LEFT, ADDRESS_LEFT, ADDRESS_MIDDLE,
                             ADDRESS_RIGHT, ADDRESS_FAR_RIGHT, ADDRESS_FAR_FAR_RIGHT};
    const char *names[7] = {"FFL", "FL", "L", "M", "R", "FR", "FFR"};
    for (int i = 0; i < 7; i++) {
        if (!sensorEnabled[i]) {
            sensorOk[i] = false;  // XSHUT stays LOW -> powered down, off the bus
            logEvent("sensor %s DISABLED in code (powered down)", names[i]);
            continue;
        }
        sensorOk[i] = initSensor(*objs[i], xshut[i], addr[i]);
        lastNewDataMs[i] = millis();
        if (!sensorOk[i]) logEvent("sensor %s FAILED to init -> reads as %d cm", names[i], maxDistance);
        if (sensorOk[i] && sensorLaserOff[i]) {
            objs[i]->stopContinuous();  // on the bus, but not emitting
            sensorOk[i] = false;        // not read, reports maxDistance
            logEvent("sensor %s on bus, laser OFF (test)", names[i]);
        }
    }

#if SENSOR_IRQ_ONLY
    // Configure IRQ pins only if using interrupt mode
    pinMode(IRQ_FAR_FAR_LEFT_PIN, INPUT_PULLUP);
    pinMode(IRQ_FAR_LEFT_PIN, INPUT_PULLUP);
    pinMode(IRQ_LEFT_PIN, INPUT_PULLUP);
    pinMode(IRQ_MIDDLE_PIN, INPUT_PULLUP);
    pinMode(IRQ_RIGHT_PIN, INPUT_PULLUP);
    pinMode(IRQ_FAR_RIGHT_PIN, INPUT_PULLUP);
    pinMode(IRQ_FAR_FAR_RIGHT_PIN, INPUT_PULLUP);

    // Attach interrupts
    attachInterrupt(digitalPinToInterrupt(IRQ_FAR_FAR_LEFT_PIN),  [](){ irq_flag_far_far_left = true; }, FALLING);
    attachInterrupt(digitalPinToInterrupt(IRQ_FAR_LEFT_PIN),      [](){ irq_flag_far_left = true; }, FALLING);
    attachInterrupt(digitalPinToInterrupt(IRQ_LEFT_PIN),          [](){ irq_flag_left = true; }, FALLING);
    attachInterrupt(digitalPinToInterrupt(IRQ_MIDDLE_PIN),        [](){ irq_flag_middle = true; }, FALLING);
    attachInterrupt(digitalPinToInterrupt(IRQ_RIGHT_PIN),         [](){ irq_flag_right = true; }, FALLING);
    attachInterrupt(digitalPinToInterrupt(IRQ_FAR_RIGHT_PIN),     [](){ irq_flag_far_right = true; }, FALLING);
    attachInterrupt(digitalPinToInterrupt(IRQ_FAR_FAR_RIGHT_PIN), [](){ irq_flag_far_far_right = true; }, FALLING);
    Serial.println("✓ IRQ mode enabled");
#else
    Serial.println("✓ Polling mode enabled (faster cycle time)");
#endif

    Serial.println("✅ Sensor init complete");
}

bool initSensor(VL53L1X &sensor, int xshutPin, uint8_t newAddress) {
    digitalWrite(xshutPin, HIGH);
    delay(10);

    // Retry initialization up to 3 times
    int retries = 3;
    bool success = false;
    while (retries > 0 && !success) {
        if (sensor.init()) {
            success = true;
            sensor.setAddress(newAddress);
            sensor.setDistanceMode(DISTANCE_MODE);
            sensor.setMeasurementTimingBudget(TIMING_BUDGET);
            sensor.startContinuous(TIMING_BUDGET / 1000);  // Convert µs to ms
            Serial.printf("✓ VL53L1X sensor initialized on pin %d (addr 0x%02X)\n", xshutPin, newAddress);
        } else {
            retries--;
            Serial.printf("⚠️  VL53L1X init failed (pin %d), retries left: %d\n", xshutPin, retries);
            delay(100);
        }
    }

    if (!success) {
        Serial.printf("❌ VL53L1X sensor on pin %d failed - using fallback readings\n", xshutPin);
        return false;
    }

    return true;
}

// ============================================================================
// SENSOR READING FUNCTIONS
// ============================================================================

void read_Distance_Sensors(int distances[]) {
    float tempSignalRate = 0.0;  // Temporary storage for middle sensor signal rate
    for (int i = 0; i < 7; i++) failingNow[i] = false;

#if SENSOR_IRQ_ONLY
    // IRQ-driven mode: only read sensors that have signaled DATA_READY
    noInterrupts();
    bool fffl = irq_flag_far_far_left; if (fffl) irq_flag_far_far_left = false;
    bool ffl  = irq_flag_far_left;     if (ffl)  irq_flag_far_left = false;
    bool fl   = irq_flag_left;         if (fl)   irq_flag_left = false;
    bool m    = irq_flag_middle;       if (m)    irq_flag_middle = false;
    bool r    = irq_flag_right;        if (r)    irq_flag_right = false;
    bool fr   = irq_flag_far_right;    if (fr)   irq_flag_far_right = false;
    bool frr  = irq_flag_far_far_right; if (frr) irq_flag_far_far_right = false;
    interrupts();

    if (fffl) distancesAll[0] = getCalibratedDistance(sensorFarFarLeft,     "FFL", scalingFactors[0], offsets[0]);
    if (ffl)  distancesAll[1] = getCalibratedDistance(sensorFarLeft,        "FL",  scalingFactors[1], offsets[1]);
    if (fl)   distancesAll[2] = getCalibratedDistance(sensorLeft,           "L",   scalingFactors[2], offsets[2]);
    if (m)    {
        distancesAll[3] = getCalibratedDistance(sensorMiddle, "M", scalingFactors[3], offsets[3]);
        tempSignalRate = signalRate;  // Capture middle sensor's signal rate
    }
    if (r)    distancesAll[4] = getCalibratedDistance(sensorRight,          "R",   scalingFactors[4], offsets[4]);
    if (fr)   distancesAll[5] = getCalibratedDistance(sensorFarRight,       "FR",  scalingFactors[5], offsets[5]);
    if (frr)  distancesAll[6] = getCalibratedDistance(sensorFarFarRight,    "FFR", scalingFactors[6], offsets[6]);
#else
    // Polling mode: read all sensors every cycle (faster, no IRQ wiring needed)
    // Sensors that failed to init read as maxDistance (see sensorOk)
    logStage(STAGE_SENSOR_FFL);
    distancesAll[0] = sensorOk[0] ? getCalibratedDistance(sensorFarFarLeft,  "FFL", scalingFactors[0], offsets[0]) : maxDistance;
    logStage(STAGE_SENSOR_FL);
    distancesAll[1] = sensorOk[1] ? getCalibratedDistance(sensorFarLeft,     "FL",  scalingFactors[1], offsets[1]) : maxDistance;
    logStage(STAGE_SENSOR_L);
    distancesAll[2] = sensorOk[2] ? getCalibratedDistance(sensorLeft,        "L",   scalingFactors[2], offsets[2]) : maxDistance;

    // Read middle sensor and capture its signal rate
    logStage(STAGE_SENSOR_M);
    distancesAll[3] = sensorOk[3] ? getCalibratedDistance(sensorMiddle,      "M",   scalingFactors[3], offsets[3]) : maxDistance;
    tempSignalRate = signalRate;  // Capture ONLY middle sensor's signal rate

    logStage(STAGE_SENSOR_R);
    distancesAll[4] = sensorOk[4] ? getCalibratedDistance(sensorRight,       "R",   scalingFactors[4], offsets[4]) : maxDistance;
    logStage(STAGE_SENSOR_FR);
    distancesAll[5] = sensorOk[5] ? getCalibratedDistance(sensorFarRight,    "FR",  scalingFactors[5], offsets[5]) : maxDistance;
    logStage(STAGE_SENSOR_FFR);
    distancesAll[6] = sensorOk[6] ? getCalibratedDistance(sensorFarFarRight, "FFR", scalingFactors[6], offsets[6]) : maxDistance;

    //all sensors max distance (for fast acceleration)

    // distancesAll[0] = maxDistance;
    // distancesAll[1] = maxDistance;
    // distancesAll[2] = maxDistance;
    // distancesAll[3] = maxDistance;
    // distancesAll[4] = maxDistance; 
    // distancesAll[5] = maxDistance;
    // distancesAll[6] = maxDistance;




#endif

    // Update middle sensor signal rate for display
    middle_signal_rate = tempSignalRate;

    // Health: how many working sensors failed this cycle
    failingCount = 0;
    for (int i = 0; i < 7; i++) if (sensorOk[i] && failingNow[i]) failingCount++;
    badLoops = (failingCount >= SENSOR_BLIND_MIN_FAILING) ? badLoops + 1 : 0;

#if SENSOR_MAX_RISE_CM > 0
    // Rise-rate limit (see distance_sensors.h): spread sudden signal-loss jumps
    // over a few loops, pass decreases instantly.
    static int limited[7] = {130, 130, 130, 130, 130, 130, 130};
    for (int i = 0; i < 7; i++) {
        if (distancesAll[i] > limited[i] + SENSOR_MAX_RISE_CM) {
            limited[i] += SENSOR_MAX_RISE_CM;
        } else {
            limited[i] = distancesAll[i];
        }
        distancesAll[i] = limited[i];
    }
#endif

    // Blend sensor readings for better curve detection
    // LEFT: 60% near sensor + 40% angled sensors
    float left_near = distancesAll[2];
    float left_far = min(distancesAll[0], distancesAll[1]);
    distances[0] = (int)(0.6 * left_near + 0.4 * left_far);

    // MIDDLE: direct reading
    distances[1] = distancesAll[3];

    // RIGHT: 60% near sensor + 40% angled sensors
    float right_near = distancesAll[4];
    float right_far = min(distancesAll[5], distancesAll[6]);
    distances[2] = (int)(0.6 * right_near + 0.4 * right_far);
}

// Last raw reading per sensor, for the "sensors" diagnostics command
struct SensorDiag { uint16_t rawMm; float rate; uint8_t rangeStatus; uint8_t i2cStatus; };
static SensorDiag sensorDiag[7];

// Last distance per sensor, returned while a sensor has no new measurement yet
static int lastDistance[7] = {130, 130, 130, 130, 130, 130, 130};

static int sensorIndex(const VL53L1X &s) {
    const VL53L1X *all[7] = {&sensorFarFarLeft, &sensorFarLeft, &sensorLeft, &sensorMiddle,
                             &sensorRight, &sensorFarRight, &sensorFarFarRight};
    for (int i = 0; i < 7; i++) if (all[i] == &s) return i;
    return -1;
}

static const char *SENSOR_NAMES[7] = {"FFL", "FL", "L", "M", "R", "FR", "FFR"};

// "on FFL" / "off M" / "on all" / "off all": power sensors up/down at runtime (testing).
// Returns false if the command is not a sensor command.
bool sensorPowerCommand(const char *cmd, Print &out) {
    if (strcmp(cmd, "fault") == 0) {
        // Test of the sensor failsafe: power down L and R behind the code's back
        // (like a loose wire) -> I2C errors -> blind -> bus recovery should fix it
        digitalWrite(XSHUT_LEFT, LOW);
        digitalWrite(XSHUT_RIGHT, LOW);
        logEvent("TEST: sensors L and R powered down (simulated dropout)");
        out.println("fault: L and R powered down, watch the log for BLIND / recovery");
        return true;
    }
    bool on;
    if (strncmp(cmd, "on ", 3) == 0) on = true;
    else if (strncmp(cmd, "off ", 4) == 0) on = false;
    else return false;
    const char *name = cmd + (on ? 3 : 4);

    VL53L1X *objs[7] = {&sensorFarFarLeft, &sensorFarLeft, &sensorLeft, &sensorMiddle,
                        &sensorRight, &sensorFarRight, &sensorFarFarRight};
    const int xshut[7] = {XSHUT_FAR_FAR_LEFT, XSHUT_FAR_LEFT, XSHUT_LEFT, XSHUT_MIDDLE,
                          XSHUT_RIGHT, XSHUT_FAR_RIGHT, XSHUT_FAR_FAR_RIGHT};
    const uint8_t addr[7] = {ADDRESS_FAR_FAR_LEFT, ADDRESS_FAR_LEFT, ADDRESS_LEFT, ADDRESS_MIDDLE,
                             ADDRESS_RIGHT, ADDRESS_FAR_RIGHT, ADDRESS_FAR_FAR_RIGHT};
    bool all = strcasecmp(name, "all") == 0;
    bool found = false;
    for (int i = 0; i < 7; i++) {
        if (!all && strcasecmp(name, SENSOR_NAMES[i]) != 0) continue;
        found = true;
        if (on) {
            if (sensorEnabled[i] && sensorOk[i]) continue;  // already running
            digitalWrite(xshut[i], LOW);  // clean restart (chip address resets to 0x29)
            delay(5);
            *objs[i] = VL53L1X();  // library object back to default address/state too
            sensorEnabled[i] = true;
            lastDistance[i] = maxDistance;
            sensorOk[i] = initSensor(*objs[i], xshut[i], addr[i]);
            lastNewDataMs[i] = millis();
            if (sensorOk[i] && sensorLaserOff[i]) { objs[i]->stopContinuous(); sensorOk[i] = false; }
            out.printf("%s %s\n", SENSOR_NAMES[i], sensorOk[i] ? "ON" : "ON but init FAILED");
        } else {
            digitalWrite(xshut[i], LOW);  // powered down, off the bus
            sensorEnabled[i] = false;
            sensorOk[i] = false;
            distancesAll[i] = maxDistance;
            out.printf("%s OFF\n", SENSOR_NAMES[i]);
        }
        logEvent("sensor %s switched %s by command", SENSOR_NAMES[i], on ? "on" : "off");
    }
    if (!found) out.println("unknown sensor, use FFL FL L M R FR FFR or all");
    return true;
}

// "scan": list every I2C address that answers (expected: 0x30-0x36, 0x29 = a sensor
// that is powered but was never given its address, e.g. XSHUT not connected)
void i2cScan(Print &out) {
    int found = 0;
    for (uint8_t a = 1; a < 127; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) {
            const char *who = "";
            const uint8_t addrs[7] = {ADDRESS_FAR_FAR_LEFT, ADDRESS_FAR_LEFT, ADDRESS_LEFT, ADDRESS_MIDDLE,
                                      ADDRESS_RIGHT, ADDRESS_FAR_RIGHT, ADDRESS_FAR_FAR_RIGHT};
            for (int i = 0; i < 7; i++) if (addrs[i] == a) who = SENSOR_NAMES[i];
            if (a == 0x29) who = "default address (unconfigured sensor!)";
            out.printf("0x%02X %s\n", a, who);
            found++;
        }
    }
    out.printf("%d device(s) found\n", found);
}

void printSensorDiagnostics(Print &out) {
    const char *names[7] = {"FFL", "FL", "L", "M", "R", "FR", "FFR"};
    out.printf("min signal rate = %.2f MCPS, max distance = %d cm\n", signal_rate_minimal, maxDistance);
    for (int i = 0; i < 7; i++) {
        const SensorDiag &d = sensorDiag[i];
        // rangeStatus: 0 = valid, 1 = sigma fail, 2 = signal fail, 4 = out of bounds,
        // 7 = wrap-around, 255 = none (see VL53L1X::RangeStatus)
        out.printf("%-3s init=%s raw=%4umm rate=%6.2f range_status=%3u i2c=%u -> %d cm\n",
                   names[i], !sensorEnabled[i] ? "OFF " : sensorLaserOff[i] ? "NOLZ" : sensorOk[i] ? "ok  " : "FAIL", d.rawMm, d.rate, d.rangeStatus,
                   d.i2cStatus, distancesAll[i]);
    }
}

int getCalibratedDistance(VL53L1X &sensor, const char* sensorName, float scalingFactor, int offset) {
    int idx = sensorIndex(sensor);

    // Only read when a NEW measurement is ready. Reading earlier is not just stale:
    // the library calibrates the sensor (phasecal) on the first read() after start,
    // and if that happens before the first measurement finished, the calibration is
    // garbage and the sensor reports ~2.4 m "out of bounds" (status 4) until restarted.
    // That is what broke the last-initialized sensor (FFR) at boot. Fixed 2026-11-04.
    if (!sensor.dataReady()) {
        if (sensor.last_status != 0) {
            logI2CError(sensorName, sensor.last_status);
            if (idx >= 0) failingNow[idx] = true;
            return maxDistance;
        }
        if (idx >= 0 && millis() - lastNewDataMs[idx] > SENSOR_STALE_MS) {
            failingNow[idx] = true;  // alive on the bus but no new data: stuck sensor
            return maxDistance;
        }
        return (idx >= 0) ? lastDistance[idx] : maxDistance;  // no new data yet
    }
    if (idx >= 0) lastNewDataMs[idx] = millis();

    uint16_t rawDistance = sensor.read(false);
    signalRate = sensor.ranging_data.peak_signal_count_rate_MCPS;

    if (idx >= 0) {
        sensorDiag[idx] = {rawDistance, signalRate, (uint8_t)sensor.ranging_data.range_status, sensor.last_status};
    }

    // I2C failure (sensor lost power / reset to default address / bus stuck)
    if (sensor.last_status != 0) {
        logI2CError(sensorName, sensor.last_status);
        if (idx >= 0) failingNow[idx] = true;
        return maxDistance;
    }

    // Reject weak signals or invalid readings
    int result;
    if (signalRate < signal_rate_minimal || rawDistance == 0) {
        result = maxDistance;
    } else {
        // Convert mm to cm and apply calibration
        int calibratedDistance = (rawDistance + offset) * scalingFactor / 10;
        result = (calibratedDistance > maxDistance) ? maxDistance : calibratedDistance;
    }
    if (idx >= 0) lastDistance[idx] = result;
    return result;
}

// ============================================================================
// DEBUG FUNCTIONS
// ============================================================================

void debugSensors() {
    Serial.print("Sensors [cm] | ");
    Serial.print("FFL:"); Serial.print(distancesAll[0]); Serial.print(" ");
    Serial.print("FL:");  Serial.print(distancesAll[1]); Serial.print(" ");
    Serial.print("L:");   Serial.print(distancesAll[2]); Serial.print(" ");
    Serial.print("M:");   Serial.print(distancesAll[3]); Serial.print(" ");
    Serial.print("R:");   Serial.print(distancesAll[4]); Serial.print(" ");
    Serial.print("FR:");  Serial.print(distancesAll[5]); Serial.print(" ");
    Serial.print("FFR:"); Serial.print(distancesAll[6]);
    Serial.print(" | Signal:"); Serial.println(middle_signal_rate);
}


// ============================================================================
// SENSOR HEALTH + I2C BUS RECOVERY
// ============================================================================
bool sensors_blind() { return badLoops >= SENSOR_BLIND_LOOPS; }
int sensors_failing() { return failingCount; }

// Free a stuck I2C bus (a slave holding SDA low) and re-initialise all sensors.
// Blocks ~0.1-1 s: the caller must put the motor in neutral first.
int sensors_recover() {
    Wire.end();
    // Clock out a stuck slave: up to 9 SCL pulses until SDA is released, then STOP
    pinMode(SDA, INPUT_PULLUP);
    pinMode(SCL, OUTPUT_OPEN_DRAIN);
    digitalWrite(SCL, HIGH);
    for (int i = 0; i < 9 && digitalRead(SDA) == LOW; i++) {
        digitalWrite(SCL, LOW);  delayMicroseconds(5);
        digitalWrite(SCL, HIGH); delayMicroseconds(5);
    }
    pinMode(SDA, OUTPUT_OPEN_DRAIN);
    digitalWrite(SDA, LOW);  delayMicroseconds(5);
    digitalWrite(SCL, HIGH); delayMicroseconds(5);
    digitalWrite(SDA, HIGH); delayMicroseconds(5);
    Wire.begin();
    Wire.setClock(400000);

    VL53L1X *objs[7] = {&sensorFarFarLeft, &sensorFarLeft, &sensorLeft, &sensorMiddle,
                        &sensorRight, &sensorFarRight, &sensorFarFarRight};
    const int xshut[7] = {XSHUT_FAR_FAR_LEFT, XSHUT_FAR_LEFT, XSHUT_LEFT, XSHUT_MIDDLE,
                          XSHUT_RIGHT, XSHUT_FAR_RIGHT, XSHUT_FAR_FAR_RIGHT};
    const uint8_t addr[7] = {ADDRESS_FAR_FAR_LEFT, ADDRESS_FAR_LEFT, ADDRESS_LEFT, ADDRESS_MIDDLE,
                             ADDRESS_RIGHT, ADDRESS_FAR_RIGHT, ADDRESS_FAR_FAR_RIGHT};
    for (int i = 0; i < 7; i++) digitalWrite(xshut[i], LOW);  // all off -> default address
    delay(10);
    int ok = 0;
    for (int i = 0; i < 7; i++) {
        if (!sensorEnabled[i]) continue;
        *objs[i] = VL53L1X();
        sensorOk[i] = initSensor(*objs[i], xshut[i], addr[i]);
        if (sensorOk[i] && sensorLaserOff[i]) { objs[i]->stopContinuous(); sensorOk[i] = false; }
        lastDistance[i] = maxDistance;
        lastNewDataMs[i] = millis();
        if (sensorOk[i]) ok++;
    }
    badLoops = 0;
    failingCount = 0;
    return ok;
}
