#ifndef AUTONOMOUSDRIVING_H
#define AUTONOMOUSDRIVING_H

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
#endif
