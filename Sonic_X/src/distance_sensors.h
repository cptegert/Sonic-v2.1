//A .h file, or header file, contains declarations—like a blueprint. These declarations inform the program what functions, variables, or classes exist, without actually defining how they work.
//Think of a .h file as a menu in a restaurant. It lists all the dishes (functions, variables) available but doesn't tell you how to cook them (that's in the .cpp file).

#ifndef DISTANCE_SENSORS_H
#define DISTANCE_SENSORS_H

#include <Wire.h>
#include <VL53L1X.h> //Need to install VL53L1X library, bt Pololu
#include <VL53L0X.h>

// ============================================================================
// SENSOR PIN CONFIGURATION
// ============================================================================
// XSHUT pins control sensor power-up for I2C address assignment
// Safe GPIO pins selected to avoid ADC2 (BLE conflict) and strapping pins

#define XSHUT_FAR_FAR_LEFT   32   // VL53L0X - Leftmost sensor
#define XSHUT_FAR_LEFT       15   // VL53L1X
#define XSHUT_LEFT           13   // VL53L1X
#define XSHUT_MIDDLE         12   // VL53L1X - Center sensor
#define XSHUT_RIGHT          4    // VL53L1X
#define XSHUT_FAR_RIGHT      16   // VL53L1X
#define XSHUT_FAR_FAR_RIGHT  33   // VL53L0X - Rightmost sensor

// ============================================================================
// I2C ADDRESS CONFIGURATION
// ============================================================================
// Unique I2C addresses assigned to each sensor during initialization

#define ADDRESS_FAR_FAR_LEFT   0x35   // VL53L0X
#define ADDRESS_FAR_LEFT       0x33   // VL53L1X
#define ADDRESS_LEFT           0x30   // VL53L1X
#define ADDRESS_MIDDLE         0x31   // VL53L1X
#define ADDRESS_RIGHT          0x32   // VL53L1X
#define ADDRESS_FAR_RIGHT      0x34   // VL53L1X
#define ADDRESS_FAR_FAR_RIGHT  0x36   // VL53L0X

// ============================================================================
// SENSOR MEASUREMENT CONFIGURATION
// ============================================================================

#define DISTANCE_MODE VL53L1X::Long   // Long distance mode (up to 4m)
#define TIMING_BUDGET 20000           // Measurement timing budget in microseconds (20ms)
                                       // Lower = faster, higher = more accurate
                                       // 20ms is a good balance for stable fast readings

#define SENSOR_WAIT_TIMEOUT 25        // Max wait time in ms for sensor data ready
                                       // Should be slightly > TIMING_BUDGET/1000
                                       // Keep this tight to avoid long waits

#define SENSOR_READ_TIMEOUT_MS 100    // Timeout for sensor reads (prevents infinite blocking)

extern int maxDistance;                // Maximum valid distance in cm (runtime adjustable)

// ============================================================================
// SENSOR OBJECTS
// ============================================================================

extern VL53L1X sensorFarLeft, sensorLeft, sensorMiddle, sensorRight, sensorFarRight;
extern VL53L0X sensorFarFarLeft, sensorFarFarRight;

// ============================================================================
// IRQ CONFIGURATION (Optional - for interrupt-driven sensor reading)
// ============================================================================
// IRQ pins can be connected to sensor GPIO1 pins for DATA_READY interrupts
// If not using IRQs, set SENSOR_IRQ_ONLY to 0 for polling mode

#define SENSOR_IRQ_ONLY 0             // 0 = Polling mode (recommended if IRQ pins not wired)
                                       // 1 = IRQ-only mode (requires physical wiring)

#define IRQ_FAR_FAR_LEFT_PIN  34
#define IRQ_FAR_LEFT_PIN      35
#define IRQ_LEFT_PIN          36
#define IRQ_MIDDLE_PIN        39
#define IRQ_RIGHT_PIN         25
#define IRQ_FAR_RIGHT_PIN     26
#define IRQ_FAR_FAR_RIGHT_PIN 27

// ============================================================================
// CALIBRATION
// ============================================================================

extern const float scalingFactors[7];  // Per-sensor scaling factors
extern const int offsets[7];           // Per-sensor offset corrections

// ============================================================================
// DISTANCE DATA ARRAYS
// ============================================================================

extern int distances[3];      // Blended output: [0]=left, [1]=middle, [2]=right
extern int distancesAll[7];   // Raw readings from all 7 sensors

// ============================================================================
// SIGNAL QUALITY METRICS
// ============================================================================

// Note: signal_rate_minimal is defined in globals.h/cpp
extern float middle_signal_rate;   // Current signal rate of middle sensor (defined in distance_sensors.cpp)

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

// Initialization (bool return for health monitoring)
void initialize_Distance_Sensors();
bool initSensor(VL53L1X &sensor, int xshutPin, uint8_t newAddress);
bool initSensorL0X(VL53L0X &sensor, int xshutPin, uint8_t newAddress);

// Reading
void read_Distance_Sensors(int distances[]);
int getCalibratedDistance(VL53L1X &sensor, const char* sensorName, float scalingFactor, int offset);
int getCalibratedDistanceL0X(VL53L0X &sensor, const char* name, float scaling, int offset);

// Debug
void debugSensors();

#endif
