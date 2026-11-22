// Learned speed map: slow down where the track regularly goes wrong
// -----------------------------------------------------------------------------
// No IMU, so the robot cannot see the hill itself. Instead it learns WHERE in the
// lap trouble happens: the lap is cut into SPEEDMAP_PARTS parts (position from the
// lap detection), and per part it notes trouble (wall < SPEEDMAP_NEAR_CM,
// reversing, TRAPPED, not moving). After every regular lap:
//   part had trouble -> factor -= SPEEDMAP_STEP_DOWN (down to SPEEDMAP_MIN_FACTOR)
//   part was clean   -> factor += SPEEDMAP_STEP_UP   (back up to 1.0)
// While driving, the speed is multiplied by the factor of this part AND the next
// one, so the robot brakes BEFORE a trouble spot.
// Safety: only ever slower (factor <= 1), never faster than the set speed; does
// nothing while the laps are not regular (crash, push, missed lap); on/off/reset by
// command. The map is saved on the robot when a drive ends and survives power-off
// (battery swap). The typical lap time is saved with it: when a drive reaches 3
// regular laps and their lap time differs more than SPEEDMAP_NEW_TRACK from the
// saved one, it is a different track -> the map resets itself (unknown competition
// track). Manual reset: "map reset".
// Commands: map | map on | map off | map reset
// -----------------------------------------------------------------------------
#ifndef SPEED_MAP_H
#define SPEED_MAP_H

#include <Arduino.h>

#define SPEEDMAP_PARTS       20
#define SPEEDMAP_NEAR_CM     15
#define SPEEDMAP_STEP_DOWN   0.10f
#define SPEEDMAP_STEP_UP     0.03f
#define SPEEDMAP_MIN_FACTOR  0.60f
#define SPEEDMAP_NEW_TRACK   0.25f   // lap time differs > 25 % from the saved one -> new track

void speedmap_begin();                 // setup(): load the saved map
// every loop while driving: what happened at this moment
void speedmap_update(bool driving, int minSideCm, bool reversing, bool trapped, bool notMoving);
float speedmap_factor();               // speed multiplier now (1.0 = no change)
int speedmap_part();                   // current part 0..PARTS-1, -1 = position unknown
bool speedmap_command(const char *cmd, Print &out);   // map / map on|off|reset

#endif
