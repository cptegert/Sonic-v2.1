// Per-robot identity. ROBOT_NAME comes from platformio.ini (env:sonic / env:shadow).
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#ifndef ROBOT_NAME
#define ROBOT_NAME "Sonic-X"   // build without an environment flag = Sonic
#endif

#define ROBOT_WIFI_SSID ROBOT_NAME "-log"   // e.g. "Sonic-X-log", "Shadow-log"

#endif
