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
// Named tracks (2026-11-22): up to TRACK_SLOTS tracks are remembered, each with its
// own map, lap time and lap fingerprint (so the next drive finds the same lap mark
// and the parts line up). "track <name>" switches (new name = learn from scratch,
// oldest track is forgotten when full). Default track: "default".
// Commands: map | map on (slow down only) | map dynamic (slower + faster) |
//           map freeze (use it, learn nothing) | map off (not used, kept) |
//           map reset | track | track <name> | track rename <new>
// Grip / dirty wheels: see GRIP_* below.
// -----------------------------------------------------------------------------
#ifndef SPEED_MAP_H
#define SPEED_MAP_H

#include <Arduino.h>

#define SPEEDMAP_PARTS       20
#define SPEEDMAP_NEAR_CM     15
#define SPEEDMAP_STEP_DOWN   0.10f
#define SPEEDMAP_STEP_UP     0.03f
#define SPEEDMAP_MIN_FACTOR  0.60f
// Dynamic mode ("map dynamic", 2026-11-22): a part that was clean and never closer
// than SPEEDMAP_SAFE_CM to a wall in a regular lap gets SPEEDMAP_BOOST_STEP faster,
// up to SPEEDMAP_MAX_FACTOR. Trouble after speeding up: back down, and the part
// remembers its limit (factor at the trouble minus SPEEDMAP_CEIL_MARGIN).
#define SPEEDMAP_MAX_FACTOR  1.25f
#define SPEEDMAP_SAFE_CM     25
#define SPEEDMAP_BOOST_STEP  0.02f
#define SPEEDMAP_CEIL_MARGIN 0.04f
#define SPEEDMAP_NEW_TRACK   0.25f   // lap time differs > 25 % from the saved one -> new track
// Grip: a part where the robot gets GRIP_DROP_CM closer to the wall than usual
// (no hard trouble yet) is slowed by GRIP_STEP_DOWN - dirty wheels slide wider.
// "mark cleaned" (dashboard button) brings slowed parts most of the way back.
#define GRIP_DROP_CM         8
#define GRIP_STEP_DOWN       0.03f
#define GRIP_BASE_ALPHA      0.05f   // "usual" distance follows slowly
#define GRIP_CLEAN_RECOVER   0.6f
#define TRACK_SLOTS          6
#define TRACK_NAME_LEN       16

void speedmap_begin();                 // setup(): load the saved map
// every loop while driving: what happened at this moment
void speedmap_update(bool driving, int minSideCm, bool reversing, bool trapped, bool notMoving);
float speedmap_factor();               // speed multiplier now (1.0 = no change)
int speedmap_part();                   // current part 0..PARTS-1, -1 = position unknown
bool speedmap_command(const char *cmd, bool driving, Print &out);   // map ... / track ...
const char *speedmap_track_name();
int speedmap_mode();                   // 0 off, 1 slow down only, 2 dynamic, 3 frozen
const char *speedmap_mode_name();

#endif
