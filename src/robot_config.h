// Per-robot identity. ROBOT_NAME comes from platformio.ini (env:sonic / env:shadow).
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#ifndef ROBOT_NAME
#define ROBOT_NAME "Sonic-X"   // build without an environment flag = Sonic
#endif

#define ROBOT_WIFI_SSID ROBOT_NAME "-log"   // e.g. "Sonic-X-log", "Shadow-log"

// Motor scale: different ESCs reach full speed at different signals. The driving
// code thinks in Sonic's range (1500 neutral .. 2000 full forward); every motor
// command is scaled around neutral: out = 1500 + (us - 1500) * MOTOR_SCALE.
// Sonic's ESC = 1.0. Measure another ESC with the "motor <us> <s>" command.
#ifndef MOTOR_SCALE
#define MOTOR_SCALE 1.0f
#endif

#endif
