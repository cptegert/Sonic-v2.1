#ifndef REMOTEXY_CONFIG_H
#define REMOTEXY_CONFIG_H

#include <Preferences.h>

// Declare preferences as external (defined in Sonic_X.ino)
extern Preferences preferences;











//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

// you can enable debug logging to Serial at 115200
//#define REMOTEXY__DEBUGLOG    

// RemoteXY select connection mode and include library 
#define REMOTEXY_MODE__ESP32CORE_BLE

#include <BLEDevice.h>

// RemoteXY connection settings 
#include "robot_config.h"
// Bluetooth name and GUI are chosen at runtime from the robot identity stored on
// the board (robot_config.h): see robot_remotexy_init() in Sonic_X.ino.
#define REMOTEXY_BLUETOOTH_NAME "Robot"   // unused (RemoteXY_Init macro not used)
#define REMOTEXY_ACCESS_PASSWORD "EQ123"


#include <RemoteXY.h>

// RemoteXY GUI configuration  
#pragma pack(push, 1)  

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
////////////////////CONFIGURATION BELOW//////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////









// Both GUI byte arrays are in every firmware; the identity picks one at startup.
// The struct below is shared: both GUIs use exactly the same variables.
#include "RemoteXY_GUI_Sonic.h"
#include "RemoteXY_GUI_Shadow.h"
  
// this structure defines all the variables and events of your control interface 
struct {

    // input variables
  uint8_t main_page; // =1 if page is visible, else =0, from 0 to 1
  uint8_t Color_page; // =1 if page is visible, else =0, from 0 to 1
  uint8_t Parameter_tune_page; // =1 if page is visible, else =0, from 0 to 1
  uint8_t Start_switch; // =1 if switch ON and =0 if OFF, from 0 to 1
  int8_t PID_P; // from 0 to 100
  int8_t PID_I; // from 0 to 100
  int8_t PID_D; // from 0 to 100
  int8_t joystick_steering_x; // from -100 to 100
  int8_t joystick_speed_y; // from -100 to 100
  uint8_t manual_mode_switch; // =1 if switch ON and =0 if OFF, from 0 to 1
  float MAX_SPEED;
  float CURVE_SPEED_REDUCTION;
  float MIN_SPEED;
  uint8_t FIVE_SEC_WAIT; // =1 if state is ON, else =0, from 0 to 1
  uint8_t green_first; // =1 if switch ON and =0 if OFF, from 0 to 1
  uint8_t color_detection_on; // =1 if switch ON and =0 if OFF, from 0 to 1
  float red_min;
  float green_min;
  float MIN_SIGNAL_RATE;
  int16_t STUCK_DELAY_MS; // -32768 .. +32767
  int16_t REVERSE_DURATION_MS; // -32768 .. +32767
  float CLOSE_DISTANCE_CM;
  uint8_t Save_EEPROM_button; // =1 if button pressed, else =0, from 0 to 1
  int16_t gyro_threshold; // -32768 .. +32767
  int16_t ANGLE_CHANGE_THRESHOLD; // -32768 .. +32767
  uint8_t CALIBRATE_MPU_COMPASS; // =1 if button pressed, else =0, from 0 to 1
  float UPHILL_MULTIPLIER;
  float DOWNHILL_MULTIPLIER;

    // output variables
  float speed_linearbar; // from 0 to 100
  float p_value_show;
  float i_value_show;
  float d_value_show;
  float linearbar_01; // from 0 to 100
  float linearbar_02; // from 0 to 100
  float linearbar_03; // from 0 to 100
  float linearbar_04; // from 0 to 100
  float linearbar_05; // from 0 to 100
  float linearbar_06; // from 0 to 100
  float linearbar_07; // from 0 to 100
  float steer_linearbar; // from 0 to 100
  float GYRO_ANGLE_SHOW; // from -20 to 20
  float color_rate_RED;
  float color_rate_GREEN;
  float motor_speed;
  float steering;
  float distance_middle_show;
  float signal_rate_graph;
  float DELTA_X;
  float DELTA_Y;
  uint8_t Saved_string; // from 0 to 1
  float CYCLE_TIME_SHOW;

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0

} RemoteXY;   















/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
////////////////////CONFIGURATION ABOVE//////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////













#pragma pack(pop)
 
/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////


// External reference to the saved indication timer (defined in main .ino file)
extern unsigned long savedIndicationStartTime;

// External references to global variables (defined in globals.cpp)
extern float kp;
extern float ki;
extern float kd;
extern float speed_limit;
extern float speed_reduction;
extern float signal_rate_minimal;
extern float red_min;
extern float green_min;
extern uint8_t color_detection_on;
extern float min_speed;
extern int16_t STUCK_DELAY_MS;
extern int16_t REVERSE_DURATION_MS;
extern float CLOSE_DISTANCE_CM;
extern float gyro_threshold;
extern float angle_change_threshold;
extern float DOWNHILL_MULTIPLIER;
extern float UPHILL_MULTIPLIER;

// Function to save all settings to Preferences (must be declared before event handlers)
inline void saveAllSettings() {
  // Sync RemoteXY values to globals before saving
  STUCK_DELAY_MS = RemoteXY.STUCK_DELAY_MS;
  REVERSE_DURATION_MS = RemoteXY.REVERSE_DURATION_MS;
  CLOSE_DISTANCE_CM = RemoteXY.CLOSE_DISTANCE_CM;
  gyro_threshold = RemoteXY.gyro_threshold;
  angle_change_threshold = RemoteXY.ANGLE_CHANGE_THRESHOLD;
  DOWNHILL_MULTIPLIER = RemoteXY.DOWNHILL_MULTIPLIER;
  UPHILL_MULTIPLIER = RemoteXY.UPHILL_MULTIPLIER;

  preferences.begin("sonic-x", false);

  // Save converted PID float values (not slider values)
  preferences.putFloat("kp", kp);
  preferences.putFloat("ki", ki);
  preferences.putFloat("kd", kd);

  // Save all other global variables
  preferences.putFloat("max_speed", speed_limit);
  preferences.putFloat("curve_reduce", speed_reduction);
  preferences.putFloat("signal_rate", signal_rate_minimal);
  preferences.putFloat("red_min", red_min);
  preferences.putFloat("green_min", green_min);
  preferences.putUChar("color_detect", color_detection_on);
  preferences.putFloat("min_speed", min_speed);
  preferences.putShort("stuck_delay", STUCK_DELAY_MS);
  preferences.putShort("rev_duration", REVERSE_DURATION_MS);
  preferences.putFloat("close_dist", CLOSE_DISTANCE_CM);
  preferences.putFloat("gyro_thresh", gyro_threshold);
  preferences.putFloat("angle_thresh", angle_change_threshold);
  preferences.putFloat("downhill_mult", DOWNHILL_MULTIPLIER);
  preferences.putFloat("uphill_mult", UPHILL_MULTIPLIER);


  preferences.end();

  // Show "SAVED" indication for 0.7 seconds
  RemoteXY.Saved_string = 1;
  savedIndicationStartTime = millis();
}

inline void RemoteXY_PID_P_event() {
  Serial.println("P value slider changed"+ String(RemoteXY.PID_P));
}

inline void RemoteXY_PID_I_event() {
  Serial.println("I value slider changed"+ String(RemoteXY.PID_I));
}

inline void RemoteXY_PID_D_event() {
  Serial.println("D value slider changed" + String(RemoteXY.PID_D));
}



#endif // REMOTEXY_CONFIG_H