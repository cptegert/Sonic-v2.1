#ifndef MANUALDRIVING_H 
#define MANUALDRIVING_H

void manual_mode();
extern float final_speed;
extern float steer_output;
void writeServoUS(int us);  // declare it here
extern const int SERVO_MIN_US;
extern const int SERVO_MAX_US;

#endif