#include <Arduino.h>  //library for serial printing - MUST be first
#include <Wire.h>  //i2c jaoks?
#include <ESP32Servo.h>  //for Servo control
#include "RemoteXY_Config.h" //RemoteXY configuration and variables
#include "distance_sensors.h"  // Include your distance sensor header file
#include "color_sensor.h"  // Include the color sensor header file
#include "autonomousDriving.h"
#include "manualDriving.h"
#include "globals.h"  // Include globals for access to all global variables
#include <Preferences.h> //
#include <Adafruit_NeoPixel.h>  // For addressable LED 


Preferences preferences;

Servo motor;
Servo servo;

// PIN DEFINITIONS
const int MOTOR_PWM_PIN    = 17;  // DC motor driver PWM signal
const int SERVO_PIN        = 5;   // Servo PWM signal
const int NEOPIXEL_PIN     = 14;  // WS2812 addressable LED data pin
const int NEOPIXEL_COUNT   = 1;   // Number of LEDs (1 for status indicator)

// Create Neopixel object for calibration LED
Adafruit_NeoPixel pixel(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// SERVO SETTINGS (50 Hz for standard servos)
const int SERVO_FREQ_HZ  = 90; //50
const int SERVO_RES_BITS = 16;  // High resolution for fine control

// Servo pulse widths in microseconds
const int SERVO_MAX_US     = 2125;  // Full left WITH SMALLER NUMBER THAN 2260 THE ROD WILL HIT the cardan shaft BUT with this turning would be unsymmetrical, could turn more to the left to the right, so 2125 it is
const int SERVO_CENTER_US  = 1500;  // Center position
const int SERVO_MIN_US     = 875;  // Full right WITH SMALLER NUMBER THE ROD WILL HIT the cardan shaft

// Wide scan range for testing?? min and max pwm for motor
const int PWM_MIN = 1000;
const int PWM_MAX = 2000;

// The 3A brushed ESC typically works with 1000-2000µs pulses like a servo
// 1000µs = reverse, 1500µs = stopped/neutral, 2000µs = full forward
const int MOTOR_FREQ_HZ  = 50;     // 50 Hz like servo
const int MOTOR_RES_BITS = 16;     // 16-bit resolution

//if defined here, not in setup, then global variable
unsigned long startMillis = 0;  // Variable to store the time when the Start_switch is turned on
bool waitingForStart = false;   // Flag to check if we are waiting for the delay
float steer_output = 1500;  // Initialize to center position

// Non-blocking Neopixel test state machine
enum NeopixelTestState {
  NEOPIXEL_OFF,      // Test not running
  NEOPIXEL_RED,
  NEOPIXEL_GREEN,
  NEOPIXEL_BLUE,
  NEOPIXEL_PURPLE,
  NEOPIXEL_COMPLETE
};
NeopixelTestState neopixelState = NEOPIXEL_RED;
unsigned long neopixelLastChange = 0;
const unsigned long NEOPIXEL_DELAY = 500;  // 500ms per color
// middle_signal_rate is now defined in distance_sensors.cpp and declared in distance_sensors.h
float final_speed = 1500; //if defined here, not in setup, then global variable (1500 = stopped)
unsigned long savedIndicationStartTime = 0; // Time when "SAVED" indication started

// Global copies of RemoteXY fields for use in other .cpp files (avoids multiple RemoteXY includes)
int16_t STUCK_DELAY_MS = 900;
int16_t REVERSE_DURATION_MS = 350;
float CLOSE_DISTANCE_CM = 20.0;
float gyro_threshold = 10.0;
float angle_change_threshold = 30.0;
float DOWNHILL_MULTIPLIER = 0.8;  // Default: reduce speed by 20% on downhill
float UPHILL_MULTIPLIER = 1.2;    // Default: increase speed by 20% on uphill



void setup() {
  RemoteXY_Init (); 

  // Load saved settings from Preferences (EEPROM)
  preferences.begin("sonic-x", false); // namespace, read-only=false

  // Load PID float values from preferences and convert to slider positions (0-100)
  kp = preferences.getFloat("kp", 5.0);
  ki = preferences.getFloat("ki", 0.01);
  kd = preferences.getFloat("kd", 4.5);

  // Convert PID floats back to slider values for display in app
  RemoteXY.PID_P = (int8_t)(kp * 100.0 / 15.0);  // kp range: 0-15.0 → slider: 0-100
  RemoteXY.PID_I = (int8_t)(ki * 100.0 / 0.1);   // ki range: 0-0.1 → slider: 0-100
  RemoteXY.PID_D = (int8_t)(kd * 100.0 / 15.0);  // kd range: 0-15.0 → slider: 0-100

  // Load other settings to global variables and RemoteXY
  speed_limit = preferences.getFloat("max_speed", 64.0);
  RemoteXY.MAX_SPEED = speed_limit;

  speed_reduction = preferences.getFloat("curve_reduce", 0.55);
  RemoteXY.CURVE_SPEED_REDUCTION = speed_reduction;

  signal_rate_minimal = preferences.getFloat("signal_rate", 5.0);
  RemoteXY.MIN_SIGNAL_RATE = signal_rate_minimal;

  red_min = preferences.getFloat("red_min", 150.0);
  RemoteXY.red_min = red_min;

  green_min = preferences.getFloat("green_min", 100.0);
  RemoteXY.green_min = green_min;

  color_detection_on = preferences.getUChar("color_detect", 1);
  RemoteXY.color_detection_on = color_detection_on;

  min_speed = preferences.getFloat("min_speed", 20.0);
  RemoteXY.MIN_SPEED = min_speed;

  STUCK_DELAY_MS = preferences.getShort("stuck_delay", 900);
  RemoteXY.STUCK_DELAY_MS = STUCK_DELAY_MS;

  REVERSE_DURATION_MS = preferences.getShort("rev_duration", 350);
  RemoteXY.REVERSE_DURATION_MS = REVERSE_DURATION_MS;

  CLOSE_DISTANCE_CM = preferences.getFloat("close_dist", 20.0);
  RemoteXY.CLOSE_DISTANCE_CM = CLOSE_DISTANCE_CM;

  gyro_threshold = preferences.getFloat("gyro_thresh", 5.0);
  RemoteXY.gyro_threshold = gyro_threshold;

  angle_change_threshold = preferences.getFloat("angle_thresh", 30.0);
  RemoteXY.ANGLE_CHANGE_THRESHOLD = angle_change_threshold;

  DOWNHILL_MULTIPLIER = preferences.getFloat("downhill_mult", 0.8);
  RemoteXY.DOWNHILL_MULTIPLIER = DOWNHILL_MULTIPLIER;

  UPHILL_MULTIPLIER = preferences.getFloat("uphill_mult", 1.2);
  RemoteXY.UPHILL_MULTIPLIER = UPHILL_MULTIPLIER;

  preferences.end();

  // Initialize output display variables
  RemoteXY.p_value_show = kp;
  RemoteXY.i_value_show = ki;
  RemoteXY.d_value_show = kd;

  RemoteXY.Start_switch = 1; //for having esp reset button as main power on button
  


  Serial.begin(115200);

  // Initialize Neopixel LED for calibration indication
  pixel.begin();
  pixel.show(); // Initialize all pixels to 'off'
  pixel.setBrightness(50); // Set brightness (0-255, 50 = moderate brightness)

  // Start non-blocking Neopixel test (will run in loop)
  Serial.println("======================================");
  Serial.println("Testing Neopixel LED on pin " + String(NEOPIXEL_PIN) + "...");
  Serial.println("You should see: Red -> Green -> Blue -> Purple");
  Serial.println("(Non-blocking test - will complete in 2 seconds)");
  Serial.println("======================================");

  // Initialize state machine
  neopixelState = NEOPIXEL_RED;
  neopixelLastChange = millis();
  pixel.setPixelColor(0, pixel.Color(255, 0, 0));  // Start with red
  pixel.show();

  motor.setPeriodHertz(50);   // Standard 50 Hz ESC frequency
  servo.setPeriodHertz(50);   // Standard 50 Hz ESC frequency
  motor.attach(MOTOR_PWM_PIN, PWM_MIN, PWM_MAX);
  servo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  
  Serial.println("Arming ESC (sending neutral 1500 µs)...");
  motor.writeMicroseconds(1500);
  servo.writeMicroseconds(1500);
  RemoteXY_delay(500); // give ESC time to arm

  Wire.begin();  // Initialize I2C
  Wire.setClock(400000); // use 400 kHz I2C, vl53l1x librarys oli, sellega anduri mõõtmised palju stabiilsemad

  initialize_Distance_Sensors();// Initialize distance sensors (if applicable)

}

void loop() {
  // Measuˇre cycle time - start
  unsigned long cycleStartTime = micros();
  RemoteXYEngine.handler ();
  read_Distance_Sensors(distances);

  // debugSensors();
  
//if WRONG Max_speed is set:
 if (RemoteXY.MAX_SPEED > 100) { RemoteXY.MAX_SPEED = 100; }
 //if WRONG speed_reduction is set:
 if (RemoteXY.CURVE_SPEED_REDUCTION > 1) { RemoteXY.CURVE_SPEED_REDUCTION = RemoteXY.CURVE_SPEED_REDUCTION/100; }
 if (RemoteXY.CURVE_SPEED_REDUCTION > 1) { RemoteXY.CURVE_SPEED_REDUCTION = 1; }
 if (RemoteXY.CURVE_SPEED_REDUCTION < 0) { RemoteXY.CURVE_SPEED_REDUCTION = 0; }

 // === SYNC RemoteXY → Global Variables ===
 // Convert PID slider values (0-100) to actual PID coefficients
 kp = RemoteXY.PID_P * 15.0 / 100.0; // 0-15.0
 ki = RemoteXY.PID_I * 0.1 / 100.0; // 0-0.1
 kd = RemoteXY.PID_D * 15.0 / 100.0; // 0-15.0

 // Sync all other RemoteXY input variables to global variables
 speed_limit = RemoteXY.MAX_SPEED;
 speed_reduction = RemoteXY.CURVE_SPEED_REDUCTION;
 signal_rate_minimal = RemoteXY.MIN_SIGNAL_RATE;
 red_min = RemoteXY.red_min;
 green_min = RemoteXY.green_min;
 color_detection_on = RemoteXY.color_detection_on;
 min_speed = RemoteXY.MIN_SPEED;
 manual_mode_switch = RemoteXY.manual_mode_switch;
 Start_switch = RemoteXY.Start_switch;
 joystick_steering_x = RemoteXY.joystick_steering_x;
 joystick_speed_y = RemoteXY.joystick_speed_y;
 green_first = RemoteXY.green_first;

 // Sync autonomous driving parameters
 STUCK_DELAY_MS = RemoteXY.STUCK_DELAY_MS;
 REVERSE_DURATION_MS = RemoteXY.REVERSE_DURATION_MS;
 CLOSE_DISTANCE_CM = RemoteXY.CLOSE_DISTANCE_CM;
 gyro_threshold = RemoteXY.gyro_threshold;
 DOWNHILL_MULTIPLIER = RemoteXY.DOWNHILL_MULTIPLIER;
 UPHILL_MULTIPLIER = RemoteXY.UPHILL_MULTIPLIER;

//if MAIN_SWITCH ON
  if (Start_switch==1) {
    // If not already waiting for the 5-second delay
    if (waitingForStart == false) {
      startMillis = millis();  // Record the time when the main switch was turned on
      waitingForStart = true;  // Set the flag to true, indicating we're waiting

      if (RemoteXY.FIVE_SEC_WAIT == 1) {
        Serial.println("5-second wait BYPASSED - starting immediately!");
      } else {
        Serial.println("Waiting for 5 sec to start...");
      }
    }

    // Check if we should start: either bypass enabled OR 5 seconds have passed
    bool shouldStart = (RemoteXY.FIVE_SEC_WAIT == 1) || (millis() - startMillis >= 2900); // to account for esp bootup time, when start is given via esp reset.


    if (shouldStart) {
      //if MANUAL_MODE
      if (manual_mode_switch==1) {
        manual_mode();
        //Serial.println("manual mode");
      //if AUTONOMOUS_MODE
      } else{
        autonomous_mode();
      }
    }
  //if MAIN_SWITCH OFF
  } else{
    waitingForStart = false; // Reset the waiting flag
    motor.writeMicroseconds(1500);
    servo.writeMicroseconds(1500);
  }


// Update output variables for RemoteXY display
  RemoteXY.linearbar_01 = map(distancesAll[0], 0, maxDistance, 0, 100);
  RemoteXY.linearbar_02 = map(distancesAll[1], 0, maxDistance, 0, 100);
  RemoteXY.linearbar_03 = map(distancesAll[2], 0, maxDistance, 0, 100);
  RemoteXY.linearbar_04 = map(distancesAll[3], 0, maxDistance, 0, 100);
  RemoteXY.linearbar_05 = map(distancesAll[4], 0, maxDistance, 0, 100);
  RemoteXY.linearbar_06 = map(distancesAll[5], 0, maxDistance, 0, 100);
  RemoteXY.linearbar_07 = map(distancesAll[6], 0, maxDistance, 0, 100);

  RemoteXY.distance_middle_show = distancesAll[3]; // Middle sensor raw distance
  RemoteXY.steering = steer_output;
  RemoteXY.motor_speed = final_speed;
  RemoteXY.signal_rate_graph = middle_signal_rate;

  // Update PID value displays to show actual float values
  RemoteXY.p_value_show = kp;
  RemoteXY.i_value_show = ki;
  RemoteXY.d_value_show = kd;

  // Update speed_linearbar: convert motor speed (1000-2000µs) to percentage (0-100)
  // 1500 = 50% (center/stopped), 2000 = 100% (full forward), 1000 = 0% (full reverse)
  RemoteXY.speed_linearbar = map((int)final_speed, 1000, 2000, 0, 100);

  // Update steer_linearbar: convert steering (875-2125µs) to percentage (0-100)
  // 1500 = 50% (center), 875 = 0% (full right), 2125 = 100% (full left)
  RemoteXY.steer_linearbar = map((int)steer_output, 2125, 875, 0, 100);

  // Save button event handling
  if (RemoteXY.Save_EEPROM_button == 1) { // If Save button pressed
    saveAllSettings();
    RemoteXY.Save_EEPROM_button = 0; // Reset button
    Serial.println("Settings saved to EEPROM.");
  }

  // Calibration button event handling
  // WARNING: Calibration blocks for ~35 seconds and will cause I2C errors if sensors are active
  // Only calibrate when robot is stationary with main switch OFF
  if (RemoteXY.CALIBRATE_MPU_COMPASS == 1 && Start_switch == 0) {
    Serial.println("==============================================");
    Serial.println("Calibration button pressed!");
    Serial.println("IMPORTANT: Robot must be stationary!");
    Serial.println("==============================================");

    // MPU6050 calibration removed - no gyro implementation

    Serial.println("=== Calibration skipped (no gyro) ===");

    // Reset button
    RemoteXY.CALIBRATE_MPU_COMPASS = 0;
  } else if (RemoteXY.CALIBRATE_MPU_COMPASS == 1 && Start_switch == 1) {
    Serial.println("ERROR: Cannot calibrate while robot is active!");
    Serial.println("Turn OFF main switch before calibrating.");
    RemoteXY.CALIBRATE_MPU_COMPASS = 0;
  }

  // Reset "SAVED" indication after 0.7 seconds
  if (RemoteXY.Saved_string == 1 && (millis() - savedIndicationStartTime) >= 700) {
    RemoteXY.Saved_string = 0;
  }



  // Measure cycle time - end (convert microseconds to milliseconds)
  unsigned long cycleEndTime = micros();
  float cycleTimeMs = (cycleEndTime - cycleStartTime) / 1000.0;
  // Note: If you need to display cycle time, map it to CYCLE_TIME_SHOW or add a cycle time variable to RemoteXY
  RemoteXY.CYCLE_TIME_SHOW = cycleTimeMs;
} 

