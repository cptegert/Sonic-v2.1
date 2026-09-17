#include "distance_sensors.h"
#include "globals.h"  // For signal_rate_minimal
#include <Arduino.h>
#include <VL53L1X.h>
#include <VL53L0X.h>

// ============================================================================
// SENSOR OBJECTS
// ============================================================================

VL53L1X sensorFarLeft, sensorLeft, sensorMiddle, sensorRight, sensorFarRight;
VL53L0X sensorFarFarLeft, sensorFarFarRight;

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
    initSensor(sensorFarLeft,  XSHUT_FAR_LEFT,  ADDRESS_FAR_LEFT);
    initSensor(sensorLeft,     XSHUT_LEFT,      ADDRESS_LEFT);
    initSensor(sensorMiddle,   XSHUT_MIDDLE,    ADDRESS_MIDDLE);
    initSensor(sensorRight,    XSHUT_RIGHT,     ADDRESS_RIGHT);
    initSensor(sensorFarRight, XSHUT_FAR_RIGHT, ADDRESS_FAR_RIGHT);

    Serial.println("  Initializing VL53L0X sensors...");
    initSensorL0X(sensorFarFarLeft,  XSHUT_FAR_FAR_LEFT,  ADDRESS_FAR_FAR_LEFT);
    initSensorL0X(sensorFarFarRight, XSHUT_FAR_FAR_RIGHT, ADDRESS_FAR_FAR_RIGHT);

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

bool initSensorL0X(VL53L0X &sensor, int xshutPin, uint8_t newAddress) {
    digitalWrite(xshutPin, HIGH);
    delay(10);

    // Retry mechanism for VL53L0X (same as VL53L1X)
    int retries = 3;
    bool success = false;

    while (retries > 0 && !success) {
        sensor.setTimeout(50);
        if (sensor.init()) {
            success = true;
            sensor.setAddress(newAddress);
            sensor.setMeasurementTimingBudget(TIMING_BUDGET);  // Match VL53L1X timing
            sensor.startContinuous(TIMING_BUDGET / 1000);      // Convert µs to ms

            // Verify sensor responds after init
            delay(60);
            uint16_t testRead = sensor.readRangeContinuousMillimeters();
            if (sensor.timeoutOccurred()) {
                success = false;
                retries--;
                Serial.printf("⚠️ VL53L0X post-init test failed (pin %d), retries left: %d\n", xshutPin, retries);
                sensor.stopContinuous();
                delay(100);
            } else {
                Serial.printf("✓ VL53L0X sensor initialized on pin %d (addr 0x%02X)\n", xshutPin, newAddress);
            }
        } else {
            retries--;
            Serial.printf("⚠️ VL53L0X init failed (pin %d), retries left: %d\n", xshutPin, retries);
            delay(100);
        }
    }

    if (!success) {
        Serial.printf("❌ VL53L0X sensor on pin %d failed - using fallback readings\n", xshutPin);
        return false;
    }

    return true;
}

// ============================================================================
// SENSOR READING FUNCTIONS
// ============================================================================

void read_Distance_Sensors(int distances[]) {
    float tempSignalRate = 0.0;  // Temporary storage for middle sensor signal rate

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

    if (fffl) distancesAll[0] = getCalibratedDistanceL0X(sensorFarFarLeft,  "FFL", scalingFactors[0], offsets[0]);
    if (ffl)  distancesAll[1] = getCalibratedDistance(sensorFarLeft,        "FL",  scalingFactors[1], offsets[1]);
    if (fl)   distancesAll[2] = getCalibratedDistance(sensorLeft,           "L",   scalingFactors[2], offsets[2]);
    if (m)    {
        distancesAll[3] = getCalibratedDistance(sensorMiddle, "M", scalingFactors[3], offsets[3]);
        tempSignalRate = signalRate;  // Capture middle sensor's signal rate
    }
    if (r)    distancesAll[4] = getCalibratedDistance(sensorRight,          "R",   scalingFactors[4], offsets[4]);
    if (fr)   distancesAll[5] = getCalibratedDistance(sensorFarRight,       "FR",  scalingFactors[5], offsets[5]);
    if (frr)  distancesAll[6] = getCalibratedDistanceL0X(sensorFarFarRight, "FFR", scalingFactors[6], offsets[6]);
#else
    // Polling mode: read all sensors every cycle (faster, no IRQ wiring needed)
    //distancesAll[0] = getCalibratedDistanceL0X(sensorFarFarLeft,  "FFL", scalingFactors[0], offsets[0]);
    distancesAll[0] = maxDistance;
    distancesAll[1] = getCalibratedDistance(sensorFarLeft,        "FL",  scalingFactors[1], offsets[1]);
    distancesAll[2] = getCalibratedDistance(sensorLeft,           "L",   scalingFactors[2], offsets[2]);

    // Read middle sensor and capture its signal rate
    distancesAll[3] = getCalibratedDistance(sensorMiddle,         "M",   scalingFactors[3], offsets[3]);
    tempSignalRate = signalRate;  // Capture ONLY middle sensor's signal rate

    distancesAll[4] = getCalibratedDistance(sensorRight,          "R",   scalingFactors[4], offsets[4]);
    distancesAll[5] = getCalibratedDistance(sensorFarRight,       "FR",  scalingFactors[5], offsets[5]);
    //distancesAll[6] = getCalibratedDistanceL0X(sensorFarFarRight, "FFR", scalingFactors[6], offsets[6]);
    distancesAll[6] = maxDistance;

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

int getCalibratedDistance(VL53L1X &sensor, const char* sensorName, float scalingFactor, int offset) {
    // Quick check if data is already ready (most common case after startup)
    if (!sensor.dataReady()) {
        // Wait for data ready with timeout protection
        unsigned long waitStart = millis();
    }

    // Data is ready - read it immediately
    uint16_t rawDistance = sensor.read(false);
    signalRate = sensor.ranging_data.peak_signal_count_rate_MCPS;

    // Reject weak signals or invalid readings
    if (signalRate < signal_rate_minimal || rawDistance == 0) {
        return maxDistance;
    }

    // Convert mm to cm and apply calibration
    int calibratedDistance = (rawDistance + offset) * scalingFactor / 10;
    return (calibratedDistance > maxDistance) ? maxDistance : calibratedDistance;
}

int getCalibratedDistanceL0X(VL53L0X &sensor, const char* name, float scalingFactor, int offset) {
    unsigned long startTime = millis();
    uint16_t rawDistance = 0;



    if (sensor.timeoutOccurred() || rawDistance == 0) {
        Serial.printf("⚠️ VL53L0X %s timeout/fault -> using maxDistance\n", name);
        return maxDistance;
    }

    // Convert mm to cm and apply calibration
    int calibratedDistance = (rawDistance + offset) * scalingFactor / 10;
    return (calibratedDistance > maxDistance) ? maxDistance : calibratedDistance;
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

