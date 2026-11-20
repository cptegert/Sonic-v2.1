#ifndef AUTONOMOUSDRIVING_H
#define AUTONOMOUSDRIVING_H

#include <stdint.h>

// Uncomment to enable debug serial output (WARNING: blocks main loop, reduces performance)
// #define DEBUG_SERIAL_OUTPUT

void autonomous_mode();
extern float steer_output;
extern float final_speed;
extern float speed_limit;
extern float speed_reduction;
void read_Color();
void Read_Gyro(); //if not written here, then error: 'Read_Gyro' was not declared in this scope" 
extern const int SERVO_MIN_US;
extern const int SERVO_CENTER_US;
extern const int SERVO_MAX_US;

// Soft reversal (see autonomousDriving.cpp): neutral pause, ramped reverse at
// REVERSE_POWER of the max speed, neutral pause, then forward again.
#define REVERSE_PAUSE_MS      150    // neutral before reversing
#define REVERSE_RAMP_MS       200    // ramp from neutral to full reverse power
#define REVERSE_POWER         0.5f   // reverse power, fraction of max speed (was 0.8)
#define REVERSE_END_PAUSE_MS  100    // neutral before driving forward again

// Snapshot of the controller's internals from the last autonomous_mode() call,
// for the Wi-Fi telemetry recorder (PID tuning). Read-only copy, no effect on driving.
struct PidTelemetry {
  float left, middle, right;   // blended distances used by the controller (cm)
  float bias;                  // obstacle bias added to the error
  float error;                 // right - left + bias
  float pTerm, iTerm, dTerm;   // kp*error, ki*integral, kd*derivative
  float pid;                   // sum of the terms
  float steerUs;               // servo output (also while reversing)
  float speedUs;               // motor output
  float steerMult;             // speed reduction from steering
  float brake;                 // predictive brake factor
  uint8_t reversing;           // 1 = in reversing mode
  uint8_t trapped;             // 1 = TRAPPED detected this loop
};
extern PidTelemetry pidTel;
#endif
