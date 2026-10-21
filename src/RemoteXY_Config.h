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
#define REMOTEXY_BLUETOOTH_NAME "Sonic-X"
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









uint8_t const PROGMEM RemoteXY_CONF_PROGMEM[] =   // 1646 bytes V19 , PRO license required
  { 255,59,0,89,0,103,6,19,0,0,0,83,111,110,105,99,95,118,50,0,
  24,1,106,200,3,3,2,2,35,0,73,97,154,6,40,132,128,0,2,26,
  0,0,0,0,0,0,200,66,0,0,72,66,36,0,0,150,66,0,0,200,
  66,121,0,0,72,66,0,0,150,66,2,0,0,200,65,0,0,72,66,1,
  0,0,0,0,0,0,200,65,129,3,35,7,10,64,107,80,0,2,4,7,
  34,17,0,2,26,31,31,79,78,0,79,70,70,0,129,3,51,3,10,64,
  107,73,0,129,3,66,8,11,64,107,68,0,129,5,0,33,6,64,1,77,
  97,105,110,32,115,119,105,116,99,104,0,4,14,34,63,13,128,2,26,67,
  79,35,23,12,78,2,26,2,4,14,49,63,13,128,2,26,4,14,64,63,
  13,128,2,26,67,79,50,23,12,78,2,26,3,67,79,65,23,12,78,2,
  26,2,5,56,156,40,40,32,2,26,31,2,51,121,48,23,1,2,26,31,
  31,77,97,110,117,97,108,95,109,111,100,101,0,65,117,116,111,109,97,116,
  105,99,95,109,111,100,101,0,7,1,87,48,12,110,64,2,26,2,1,129,
  1,80,35,7,64,17,77,97,120,32,115,112,101,101,100,0,129,1,119,51,
  7,64,17,83,112,101,101,100,32,114,101,100,117,99,116,105,111,110,0,7,
  1,131,48,12,110,64,2,26,2,3,7,1,106,48,12,110,64,2,26,2,
  1,129,1,99,33,7,64,17,77,105,110,32,115,112,101,101,100,0,131,71,
  7,32,17,2,17,2,31,80,97,103,101,0,6,129,1,126,51,5,0,17,
  40,48,46,51,32,102,97,115,116,101,114,44,32,48,44,54,32,115,108,111,
  119,101,114,32,41,0,73,4,159,4,35,132,128,0,1,26,0,0,0,0,
  0,0,200,66,0,0,0,0,1,0,0,0,0,0,0,200,65,2,0,0,
  200,65,0,0,72,66,3,0,0,72,66,0,0,150,66,135,0,0,150,66,
  0,0,200,66,73,10,155,4,36,132,128,0,1,26,0,0,0,0,0,0,
  200,66,0,0,0,0,1,0,0,0,0,0,0,200,65,2,0,0,200,65,
  0,0,72,66,3,0,0,72,66,0,0,150,66,135,0,0,150,66,0,0,
  200,66,73,16,153,4,35,132,128,0,1,26,0,0,0,0,0,0,200,66,
  0,0,0,0,1,0,0,0,0,0,0,200,65,2,0,0,200,65,0,0,
  72,66,3,0,0,72,66,0,0,150,66,135,0,0,150,66,0,0,200,66,
  73,22,151,4,35,132,128,0,1,26,0,0,0,0,0,0,200,66,0,0,
  0,0,1,0,0,0,0,0,0,200,65,2,0,0,200,65,0,0,72,66,
  3,0,0,72,66,0,0,150,66,135,0,0,150,66,0,0,200,66,73,28,
  153,4,35,132,128,0,1,26,0,0,0,0,0,0,200,66,0,0,0,0,
  1,0,0,0,0,0,0,200,65,2,0,0,200,65,0,0,72,66,3,0,
  0,72,66,0,0,150,66,135,0,0,150,66,0,0,200,66,73,34,155,4,
  36,132,128,0,1,26,0,0,0,0,0,0,200,66,0,0,0,0,1,0,
  0,0,0,0,0,200,65,2,0,0,200,65,0,0,72,66,3,0,0,72,
  66,0,0,150,66,135,0,0,150,66,0,0,200,66,73,40,159,4,35,132,
  128,0,1,26,0,0,0,0,0,0,200,66,0,0,0,0,1,0,0,0,
  0,0,0,200,65,2,0,0,200,65,0,0,72,66,3,0,0,72,66,0,
  0,150,66,136,0,0,150,66,0,0,200,66,129,10,144,27,6,0,31,68,
  105,115,116,97,110,99,101,115,0,129,58,77,38,8,64,17,71,121,114,111,
  32,97,110,103,108,101,0,73,55,147,42,6,133,128,0,2,26,0,0,0,
  0,0,0,200,66,0,0,72,66,36,0,0,150,66,0,0,200,66,121,0,
  0,72,66,0,0,150,66,2,0,0,200,65,0,0,72,66,1,0,0,0,
  0,0,0,200,65,71,60,86,34,34,58,0,2,1,75,0,0,160,193,0,
  0,160,65,0,0,160,65,0,0,160,64,0,0,0,0,1,0,10,46,7,
  18,18,48,1,134,31,79,70,70,0,31,79,78,0,129,42,2,27,6,0,
  39,53,32,115,101,99,32,119,97,105,116,0,19,0,2,3,36,41,11,0,
  2,26,12,1,71,82,69,69,78,0,82,69,68,0,129,8,29,31,6,0,
  12,71,114,101,101,110,32,102,105,114,115,116,63,0,129,3,13,43,6,0,
  16,67,111,108,111,114,32,100,101,116,101,99,116,105,111,110,58,0,2,3,
  19,40,9,0,2,26,31,31,79,78,0,79,70,70,0,68,0,71,50,41,
  18,8,1,135,129,1,65,31,6,0,16,67,111,108,111,114,32,114,97,116,
  101,115,58,0,129,3,51,19,4,64,12,71,114,101,101,110,95,109,105,110,
  0,129,30,51,16,4,64,1,82,101,100,95,109,105,110,0,7,27,56,26,
  8,110,64,2,26,2,4,7,1,56,26,8,110,64,2,26,2,4,68,34,
  151,30,46,49,8,1,109,111,116,111,114,0,7,66,54,27,10,110,64,2,
  26,2,2,129,57,46,44,5,0,15,77,105,110,105,109,97,108,32,115,105,
  103,110,97,108,32,114,97,116,101,58,0,68,4,151,29,46,49,26,16,115,
  116,101,101,114,0,131,71,7,32,17,2,17,2,31,80,97,103,101,0,24,
  68,55,112,43,36,49,8,36,100,105,115,116,97,110,99,101,95,109,105,100,
  100,108,101,0,68,253,112,53,35,49,8,15,115,105,103,110,97,108,95,114,
  97,116,101,32,40,109,105,100,100,108,101,41,0,129,53,65,50,6,0,16,
  68,101,108,116,97,32,71,32,109,101,97,115,117,114,105,110,103,0,68,55,
  72,44,40,18,8,1,135,22,0,130,57,135,46,44,27,28,7,2,46,48,
  12,118,64,2,26,2,129,2,39,36,7,64,17,83,116,117,99,107,32,100,
  101,108,97,121,0,7,2,66,48,12,118,64,2,26,2,129,2,59,53,7,
  64,17,82,101,118,101,114,115,101,32,100,117,114,97,116,105,111,110,0,7,
  2,87,48,12,110,64,2,26,2,2,129,2,80,63,7,64,17,67,108,111,
  115,101,32,100,105,115,116,97,110,99,101,32,40,99,109,41,0,131,71,7,
  32,17,2,17,2,31,80,97,103,101,0,33,1,45,5,20,20,0,2,31,
  0,74,4,11,40,10,1,2,135,64,83,65,86,69,68,0,129,2,180,54,
  7,64,17,67,121,99,108,101,32,115,112,101,101,100,32,40,109,115,41,0,
  67,4,189,39,11,77,2,26,2,7,2,144,40,10,118,64,2,26,2,129,
  1,136,55,6,0,16,71,121,114,111,32,97,110,103,108,101,32,116,104,114,
  101,115,104,111,108,100,0,129,1,159,51,6,0,16,72,105,116,32,97,110,
  103,108,101,32,116,104,114,101,115,104,111,108,100,0,7,2,165,40,10,118,
  64,2,26,2,129,1,106,65,6,0,16,67,97,108,105,98,114,97,116,101,
  32,109,112,117,47,99,111,109,112,97,115,115,0,1,2,115,19,19,1,2,
  31,0,7,60,144,40,10,110,64,2,26,2,2,7,60,165,40,10,110,64,
  2,26,2,2,129,59,136,30,6,0,16,85,112,104,105,108,108,32,109,117,
  108,116,46,0,129,59,159,38,6,0,16,68,111,119,110,104,105,108,108,32,
  109,117,108,116,46,0 };
  
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