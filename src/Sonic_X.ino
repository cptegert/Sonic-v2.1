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
#include "logger.h"  // black-box / crash logging
#include "wifi_log.h"  // Wi-Fi live log for laptop
#include "battery.h"   // battery voltage on GPIO 34
extern bool reversing;  // autonomousDriving.cpp, for the logger
void sendTelemetry(bool driving, float cycleTimeMs);
bool remoteCommand(const char *cmd, bool driving, Print &out);
void remoteRunWatchdog();
bool sensorFailsafe();


Preferences preferences;

Servo motor;
Servo servo;

// PIN DEFINITIONS
const int MOTOR_PWM_PIN    = 17;  // DC motor driver PWM signal
const int SERVO_PIN        = 5;   // Servo PWM signal
const int NEOPIXEL_PIN     = 14;  // WS2812 addressable LED data pin
const int NEOPIXEL_COUNT   = 1;   // Number of LEDs (1 for status indicator)
const int START_BUTTON_PIN = 18;  // Physical start button: GPIO18 -> button -> GND (active-low, internal pull-up)

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
bool startedByButton = false;   // true if the physical button turned the main switch ON (forces full 5 s wait)
float steer_output = 1500;  // Initialize to center position

// Physical start button debounce (non-blocking)
const unsigned long START_BUTTON_DEBOUNCE_MS = 50;
const unsigned long START_DELAY_MS = 5000;  // full 5 s countdown required by competition rules (button and app)
bool startButtonStableState = HIGH;     // debounced state (HIGH = released, pull-up)
bool startButtonLastReading = HIGH;     // last raw reading
unsigned long startButtonLastChange = 0; // time of last raw change

// Status LED (Neopixel):
//   main switch OFF + start button held -> solid blue (waiting for the starting whistle)
//   countdown after release             -> blue blink, once every second (0.5 s on, 0.5 s off)
//   driving (autonomous)                -> solid green
//   driving (manual)                    -> solid purple
//   main switch OFF, button not held    -> dim white (ready)
uint32_t statusLedColor = 0;  // last color sent to the LED (0 = off)
void setStatusLed(uint32_t color) {
  if (color == statusLedColor) return;  // only update on change, show() takes time
  pixel.setPixelColor(0, color);
  pixel.show();
  statusLedColor = color;
}
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
  // Serial + LED first: LED goes red while booting, so a robot stuck in a reset
  // loop (brownout) no longer keeps showing the last color (e.g. green)
  Serial.begin(115200);
  pixel.begin();
  pixel.setBrightness(50); // Set brightness (0-255, 50 = moderate brightness)
  setStatusLed(pixel.Color(255, 0, 0));
  logger_begin();  // logs reset reason + previous session black box
  battery_begin();

  RemoteXY_Init (); 
  wifiLog_begin();  // Wi-Fi access point for laptop live logs (BLE stays for RemoteXY)

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

  RemoteXY.Start_switch = 0; //When code starts, set main switch to OFF (0) to avoid accidental start
  


  pinMode(START_BUTTON_PIN, INPUT_PULLUP); // button to GND, no external resistor needed

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

  logger_setCommandHook(remoteCommand);  // params / set / start / stop over serial + Wi-Fi
  logger_setupDone();  // enables loop watchdog (loop stuck >5 s -> reset + logged)
}

void loop() {
  // Measuˇre cycle time - start
  unsigned long cycleStartTime = micros();
  logStage(STAGE_REMOTEXY);
  RemoteXYEngine.handler ();
  battery_loop();
  read_Distance_Sensors(distances);  // sets STAGE_SENSOR_* itself
  bool sensorsBlind = sensorFailsafe();  // I2C bus dead -> motor neutral + bus recovery

  // debugSensors();
  
//if WRONG Max_speed is set:
 if (RemoteXY.MAX_SPEED > 100) { RemoteXY.MAX_SPEED = 100; }
 //if WRONG speed_reduction is set:
 if (RemoteXY.CURVE_SPEED_REDUCTION > 1) { RemoteXY.CURVE_SPEED_REDUCTION = RemoteXY.CURVE_SPEED_REDUCTION/100; }
 if (RemoteXY.CURVE_SPEED_REDUCTION > 1) { RemoteXY.CURVE_SPEED_REDUCTION = 1; }
 if (RemoteXY.CURVE_SPEED_REDUCTION < 0) { RemoteXY.CURVE_SPEED_REDUCTION = 0; }

 // === PHYSICAL START BUTTON → toggles the same main switch as the app ===
 logStage(STAGE_BUTTON);
 bool startButtonReading = digitalRead(START_BUTTON_PIN);
 if (startButtonReading != startButtonLastReading) {
   startButtonLastChange = millis();  // raw change, restart debounce timer
   startButtonLastReading = startButtonReading;
 }
 if ((millis() - startButtonLastChange) >= START_BUTTON_DEBOUNCE_MS && startButtonReading != startButtonStableState) {
   startButtonStableState = startButtonReading;
   if (startButtonStableState == HIGH) {  // released (trigger once per press, on release, so the hand is away before the timer starts)
     RemoteXY.Start_switch = !RemoteXY.Start_switch;
     startedByButton = (RemoteXY.Start_switch == 1);
     logEvent(RemoteXY.Start_switch ? "start button: main switch ON" : "start button: main switch OFF");
   }
 }

 // Remote (laptop) run: time limit + stop if the laptop link is lost
 remoteRunWatchdog();

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
  logStage(STAGE_DRIVE);
  bool driving = false;  // for the logger: countdown finished, motors under control
  if (Start_switch==1) {
    // If not already waiting for the 5-second delay
    if (waitingForStart == false) {
      startMillis = millis();  // Record the time when the main switch was turned on
      waitingForStart = true;  // Set the flag to true, indicating we're waiting

      if (RemoteXY.FIVE_SEC_WAIT == 1 && !startedByButton) {
        Serial.println("5-second wait BYPASSED - starting immediately!");
      } else {
        Serial.println("Waiting for 5 sec to start...");
      }
    }

    // Check if we should start: either bypass enabled OR 5 seconds have passed
    // Button start always waits the full 5 s (bypass only works for app starts, for testing)
    bool shouldStart = (RemoteXY.FIVE_SEC_WAIT == 1 && !startedByButton) || (millis() - startMillis >= START_DELAY_MS);


    if (!shouldStart) {
      // Countdown: blue for the first half of every second, off for the second half
      bool blinkOn = ((millis() - startMillis) % 1000) < 500;
      setStatusLed(blinkOn ? pixel.Color(0, 0, 255) : 0);
    }

    if (shouldStart && sensorsBlind && manual_mode_switch == 0) {
      // Sensors dead: do NOT drive blind. Motor neutral (already set by the
      // failsafe), LED red. Driving continues by itself once sensors are back.
      driving = true;
      motor.writeMicroseconds(1500);
      servo.writeMicroseconds(1500);
      final_speed = 1500;
      setStatusLed(pixel.Color(255, 0, 0));
    } else if (shouldStart) {
      driving = true;
      // driving: purple = manual, green = autonomous
      setStatusLed(manual_mode_switch == 1 ? pixel.Color(160, 0, 255) : pixel.Color(0, 255, 0));
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
    startedByButton = false; // next start (app or button) decides its own delay
    motor.writeMicroseconds(1500);
    servo.writeMicroseconds(1500);
    // Button held while stopped = waiting for the starting whistle
    // Otherwise dim white = ready, or orange = ready but last reset was abnormal
    // (brownout / crash / watchdog) -> read the log over USB: type "log"
    uint32_t idleColor = logger_lastResetAbnormal() ? pixel.Color(255, 80, 0) : pixel.Color(120, 120, 120);
    setStatusLed(startButtonStableState == LOW ? pixel.Color(0, 0, 255) : idleColor);
  }


// Update output variables for RemoteXY display
  logStage(STAGE_OUTPUTS);
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
  logStage(STAGE_SAVE);
  if (RemoteXY.Save_EEPROM_button == 1) { // If Save button pressed
    saveAllSettings();
    RemoteXY.Save_EEPROM_button = 0; // Reset button
    logEvent("settings saved");
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

  // Black box snapshot + heartbeat + serial commands (log / status / clear)
  LogSnapshot snap;
  snap.cycleMs = (uint16_t)min(cycleTimeMs, 65535.0f);
  for (int i = 0; i < 7; i++) snap.dist[i] = distancesAll[i];
  snap.speedUs = (int16_t)final_speed;
  snap.steerUs = (int16_t)steer_output;
  snap.startSwitch = Start_switch;
  snap.manual = manual_mode_switch;
  snap.driving = driving;
  snap.connected = RemoteXY.connect_flag;
  snap.reversing = reversing;
  snap.battCv = (uint16_t)(battery_volts() * 100);
  logger_loop(snap);
  sendTelemetry(driving, cycleTimeMs);
  logStage(STAGE_LOOP_END);
}

// ============================================================================
// PID TELEMETRY (Wi-Fi): one CSV row per loop while driving, for PID tuning.
// Only sent while a laptop viewer is connected (tools/wifi_log.py saves it to
// crashlogs/run_*.csv). UDP, never blocks. Columns must match TELEMETRY_HEADER.
// ============================================================================
static const char *TELEMETRY_HEADER =
  "t_ms,loop,cyc_ms,manual,rev,trapped,FFL,FL,L,M,R,FR,FFR,left,mid,right,"
  "bias,error,p,i,d,pid,steer_us,speed_us,steer_mult,brake,kp,ki,kd,speed_limit,ble,batt_v";

void sendTelemetry(bool driving, float cycleTimeMs) {
  static bool wasDriving = false;
  static uint16_t runNo = 0;
  static uint32_t loopNo = 0, lastHeaderMs = 0;
  if (!wifiLog_hasViewer()) { wasDriving = driving; return; }

  if (driving && !wasDriving) { runNo++; loopNo = 0; lastHeaderMs = 0; }
  wasDriving = driving;
  if (!driving) return;

  char line[360];
  uint32_t now = millis();
  // Header with run number at start and every 2 s (viewer may join mid-run)
  if (lastHeaderMs == 0 || now - lastHeaderMs >= 2000) {
    lastHeaderMs = now;
    snprintf(line, sizeof(line), "TH,%u,%s", runNo, TELEMETRY_HEADER);
    wifiLog_send(line);
  }
  const PidTelemetry &t = pidTel;
  snprintf(line, sizeof(line),
           "T,%lu,%lu,%.1f,%u,%u,%u,%d,%d,%d,%d,%d,%d,%d,%.0f,%.0f,%.0f,"
           "%.1f,%.1f,%.1f,%.2f,%.1f,%.1f,%.0f,%.0f,%.2f,%.2f,%.2f,%.3f,%.2f,%.0f,%u,%.2f",
           (unsigned long)now, (unsigned long)loopNo++, cycleTimeMs, manual_mode_switch,
           t.reversing, t.trapped,
           distancesAll[0], distancesAll[1], distancesAll[2], distancesAll[3],
           distancesAll[4], distancesAll[5], distancesAll[6],
           t.left, t.middle, t.right,
           t.bias, t.error, t.pTerm, t.iTerm, t.dTerm, t.pid,
           manual_mode_switch ? steer_output : t.steerUs,
           manual_mode_switch ? final_speed : t.speedUs,
           t.steerMult, t.brake, kp, ki, kd, speed_limit, RemoteXY.connect_flag, battery_volts());
  wifiLog_send(line);
} 


// ============================================================================
// REMOTE RUN CONTROL (laptop over Wi-Fi or USB serial), for automated PID tests
//   params                 show current values
//   set kp|ki|kd|speed <v> change a value (not while driving; same as the app sliders)
//   start <seconds>        full 5 s countdown, then drive for max <seconds> (1..120)
//   stop                   main switch OFF
// Safety: a remote-started run ALWAYS stops by itself when its time is up, and
// stops within REMOTE_LINK_TIMEOUT_MS if the laptop stops sending (link lost).
// The physical button and the phone app can stop it at any time.
// ============================================================================
static const uint32_t REMOTE_LINK_TIMEOUT_MS = 1000;
static const uint32_t REMOTE_MAX_RUN_S = 120;
static bool remoteRun = false;
static uint32_t remoteRunEndMs = 0;

static void printParams(Print &out) {
  out.printf("params: kp=%.2f ki=%.3f kd=%.2f speed=%.0f main_switch=%u manual=%u five_sec_bypass=%u remote_run=%u battery=%.2fV\n",
             RemoteXY.PID_P * 15.0 / 100.0, RemoteXY.PID_I * 0.1 / 100.0, RemoteXY.PID_D * 15.0 / 100.0,
             RemoteXY.MAX_SPEED, RemoteXY.Start_switch, RemoteXY.manual_mode_switch,
             RemoteXY.FIVE_SEC_WAIT, remoteRun, battery_volts());
}

bool remoteCommand(const char *cmd, bool driving, Print &out) {
  char name[8];
  float value;
  if (strcmp(cmd, "params") == 0) {
    printParams(out);
    return true;
  }
  if (strcmp(cmd, "stop") == 0) {
    RemoteXY.Start_switch = 0;
    remoteRun = false;
    logEvent("remote: stop");
    out.println("stopped");
    return true;
  }
  if (sscanf(cmd, "set %7s %f", name, &value) == 2) {
    if (RemoteXY.Start_switch) { out.println("Not while main switch is ON"); return true; }
    // Same scaling as the app sliders (P/D: 0-15 in steps of 0.15, I: 0-0.1 in steps of 0.001)
    if (strcmp(name, "kp") == 0)         RemoteXY.PID_P = (int8_t)constrain(lroundf(value * 100.0f / 15.0f), 0, 100);
    else if (strcmp(name, "ki") == 0)    RemoteXY.PID_I = (int8_t)constrain(lroundf(value * 100.0f / 0.1f), 0, 100);
    else if (strcmp(name, "kd") == 0)    RemoteXY.PID_D = (int8_t)constrain(lroundf(value * 100.0f / 15.0f), 0, 100);
    else if (strcmp(name, "speed") == 0) RemoteXY.MAX_SPEED = constrain(value, 0.0f, 100.0f);
    else { out.println("set what? kp ki kd speed"); return true; }
    logEvent("remote: set %s %.3f", name, value);
    printParams(out);
    return true;
  }
  int seconds;
  if (sscanf(cmd, "start %d", &seconds) == 1) {
    if (RemoteXY.Start_switch) { out.println("Already ON"); return true; }
    if (RemoteXY.manual_mode_switch) { out.println("Manual mode is ON in the app - refusing"); return true; }
    if (battery_critical()) {
      out.printf("battery critical (%.2f V) - charge it - refusing\n", battery_volts());
      return true;
    }
    if (wifiLog_msSinceViewer() > REMOTE_LINK_TIMEOUT_MS) {
      out.println("start needs a live Wi-Fi link (tools/auto_run.py keeps it alive) - refusing");
      return true;
    }
    if (seconds < 1 || seconds > (int)REMOTE_MAX_RUN_S) { out.printf("start 1..%lu seconds\n", (unsigned long)REMOTE_MAX_RUN_S); return true; }
    RemoteXY.Start_switch = 1;
    startedByButton = true;  // forces the full 5 s countdown (bypass switch ignored)
    remoteRun = true;
    remoteRunEndMs = millis() + START_DELAY_MS + (uint32_t)seconds * 1000;
    logEvent("remote: start, 5 s countdown + %d s", seconds);
    out.printf("starting: 5 s countdown, then %d s\n", seconds);
    return true;
  }
  return false;
}

void remoteRunWatchdog() {
  if (!remoteRun) return;
  if (RemoteXY.Start_switch == 0) {  // stopped by button / app / stop command
    remoteRun = false;
    return;
  }
  if ((int32_t)(millis() - remoteRunEndMs) >= 0) {
    RemoteXY.Start_switch = 0;
    remoteRun = false;
    logEvent("remote run: time up, stopped");
  } else if (battery_critical()) {
    RemoteXY.Start_switch = 0;
    remoteRun = false;
    logEvent("remote run: battery critical (%.2f V), stopped", battery_volts());
  } else if (wifiLog_msSinceViewer() > REMOTE_LINK_TIMEOUT_MS) {
    RemoteXY.Start_switch = 0;
    remoteRun = false;
    logEvent("remote run: laptop link LOST, stopped");
  }
}

// ============================================================================
// SENSOR FAILSAFE (2026-11-04: after an impact the I2C bus died, all sensors read
// "nothing there" and the robot drove blind into the wall until power-off)
// Blind (>=2 sensors failing for 3 loops) -> motor neutral right away, then reset
// the I2C bus + re-init sensors (at most once per second). "Blind" is cleared
// only after 5 healthy loops in a row. Returns true while blind.
// ============================================================================
bool sensorFailsafe() {
  static bool blind = false;
  static int healthyLoops = 0;
  static uint32_t lastRecoveryMs = 0;
  static uint32_t recoveryIntervalMs = 1000;  // 5 s after a recovery that did not help
  static uint32_t blindSinceMs = 0;

  if (sensors_blind()) {
    if (!blind) {
      blind = true;
      blindSinceMs = millis();
      logEvent("SENSORS BLIND (%d failing) -> motor neutral", sensors_failing());
    }
    healthyLoops = 0;
  } else if (blind) {
    healthyLoops = (sensors_failing() < SENSOR_BLIND_MIN_FAILING) ? healthyLoops + 1 : 0;
    if (healthyLoops >= 5) {
      blind = false;
      logEvent("sensors OK again after %lu ms", (unsigned long)(millis() - blindSinceMs));
    }
  }

  if (blind) {
    motor.writeMicroseconds(1500);  // before the recovery blocks the loop
    servo.writeMicroseconds(1500);
    final_speed = 1500;
    if (millis() - lastRecoveryMs > recoveryIntervalMs) {
      lastRecoveryMs = millis();
      logStage(STAGE_SENSOR_M);  // if recovery itself hangs, the watchdog log shows a sensor stage
      uint32_t t = millis();
      int ok = sensors_recover();
      logEvent("I2C bus recovery: %d sensors OK (%lu ms)", ok, (unsigned long)(millis() - t));
      // Several sensors still dead (wiring?): retry less often, each try blocks ~0.3 s per dead sensor
      recoveryIntervalMs = (ok >= 6) ? 1000 : 5000;
    }
  }
  return blind;
}
