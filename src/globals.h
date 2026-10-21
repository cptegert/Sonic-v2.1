#ifndef GLOBALS_H
#define GLOBALS_H
#include <cstdint> // Ensure uint8_t and int8_t are defined
#include <ESP32Servo.h>

  extern uint8_t manual_mode_switch; // =1 if switch ON and =0 if OFF, from 0 to 1
  extern uint8_t Start_switch; // =1 if switch ON and =0 if OFF, from 0 to 1
  extern float kp;
  extern float ki;
  extern float kd;
  extern float speed_limit;
  extern float min_speed;
  extern float signal_rate_minimal;
  extern int8_t joystick_steering_x; // from -100 to 100
  extern int8_t joystick_speed_y; // from -100 to 100
  extern float speed_reduction;
  extern uint8_t green_first; // =1 if switch ON and =0 if OFF, from 0 to 1
  extern uint8_t color_detection_on; // =1 if switch ON and =0 if OFF, from 0 to 1
  extern float red_min;
  extern float green_min;

    // output variables
  extern float motor_speed;
  extern float signal_rate_graph;
  extern float steering;
  extern float color_rate_RED;
  extern float color_rate_GREEN;
  extern float distance0graph; // from 0 to 100
  extern float distance1graph; // from 0 to 100
  extern float distance2graph; // from 0 to 100
  extern float distance3graph; // from 0 to 100
  extern float distance4graph; // from 0 to 100
  extern float distance5graph; // from 0 to 100
  extern float distance6graph; // from 0 to 100

  // RemoteXY fields used in autonomousDriving.cpp
  extern int16_t STUCK_DELAY_MS;
  extern int16_t REVERSE_DURATION_MS;
  extern float CLOSE_DISTANCE_CM;
  extern float gyro_threshold;
  extern float angle_change_threshold;
  extern float DOWNHILL_MULTIPLIER;
  extern float UPHILL_MULTIPLIER;
  
#endif