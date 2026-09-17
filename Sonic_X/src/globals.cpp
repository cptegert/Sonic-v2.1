#include "globals.h"
#include <Arduino.h>

// Define the global variables //this made these remoteXY variables usable globally(in every file)
// samasuguse REMOTEXY nimega globaalsed input variabled (copy-paste remoteXY- config.h failist +  )

    // input variables
  uint8_t manual_mode_switch; // =1 if switch ON and =0 if OFF, from 0 to 1
  uint8_t Start_switch; // =1 if switch ON and =0 if OFF, from 0 to 1
  float kp;
  float ki;
  float kd;
  float speed_limit;
  float min_speed;
  float signal_rate_minimal;
  int8_t joystick_steering_x; // from -100 to 100
  int8_t joystick_speed_y; // from -100 to 100
  float speed_reduction;
  uint8_t green_first; // =1 if switch ON and =0 if OFF, from 0 to 1
  uint8_t color_detection_on; // =1 if switch ON and =0 if OFF, from 0 to 1
  float red_min;
  float green_min;

    // output variables
  float motor_speed;
  float signal_rate_graph;
  float steering;
  float color_rate_RED;
  float color_rate_GREEN;
  float distance0graph; // from 0 to 100
  float distance1graph; // from 0 to 100
  float distance2graph; // from 0 to 100
  float distance3graph; // from 0 to 100
  float distance4graph; // from 0 to 100
  float distance5graph; // from 0 to 100
  float distance6graph; // from 0 to 100
