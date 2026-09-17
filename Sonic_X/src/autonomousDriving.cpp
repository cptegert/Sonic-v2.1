#include "autonomousDriving.h"  //main header file for this
#include "globals.h"            //for seeing left speed, right speed, move motors
#include "distance_sensors.h"   //for seeing distance values "extern int distances[3];"
#include <Arduino.h>            //for serial printing and more? Arduino functionality
// Note: We don't include RemoteXY_Config.h here to avoid multiple definition errors.
// Instead, we use global copies of the needed fields (declared in globals.h)

extern Servo motor;  // motor ESC (speed control)
extern Servo servo;  // Front-wheel steering servo

// === PID steering variables ===
float previous_error = 0.0;
float integral = 0.0;
const float integral_limit = 50.0;  // Increased from 10.0 - allows integral to handle persistent errors (curved tracks, drift)

// === Servo settings ===
const int MAX_STEER = SERVO_MAX_US-SERVO_CENTER_US;      // = 625μs (range: center 1500μs, left 2125μs, right 875μs)

// === Reversing / Stuck Detection ===
bool reversing = false;
unsigned long reverseStart = 0;
unsigned long closeStart = 0;            // when we first saw obstacle <10 cm

// === Hit Recovery / Angle Tracking ===
bool hit_recovery_mode = false;
float initial_heading = 0.0;
float heading_at_hit = 0.0;
bool initial_heading_set = false;

void autonomous_mode() {

  // Read sensor distances
  float left_distance = distances[0];
  float middle_distance = distances[1];
  float right_distance = distances[2];

  // ===================================================================================
  //        STUCK DETECTION → ENTER REVERSING MODE IF ANY SENSOR <20CM FOR > 0.9 SEC
  // ===================================================================================
  bool tooClose =
    (left_distance < CLOSE_DISTANCE_CM) || (middle_distance < CLOSE_DISTANCE_CM) || (right_distance < CLOSE_DISTANCE_CM);

  if (!reversing) {
    if (tooClose) {
      if (closeStart == 0) closeStart = millis();  // start counting
      if (millis() - closeStart > STUCK_DELAY_MS) {
        // === Enter reversing mode ===
        reversing = true;
        reverseStart = millis();
        closeStart = 0;  // reset timer
      }
    } else {
      closeStart = 0;  // reset if distances are safe
    }
  }

  // ===================================================================================
  //                              HIT RECOVERY MODE
  // ===================================================================================
  // Reverse back to original heading after hit detection
  if (hit_recovery_mode) {
    return; // Skip forward logic
  }

  // ===================================================================================
  //                                 REVERSING MODE
  // ===================================================================================
  //3. Insert this BEFORE your PID logic (When reversing, skip your forward logic entirely)
  if (reversing) {
    // Motor reverse (adjust if ESC uses different reverse signal)
    // Reverse speed = 80% of limited maximum speed
    final_speed = 1500 - (speed_limit * 5) * 0.8;
    motor.writeMicroseconds(final_speed);

    // --- Controlled steering based on sensor closeness ---
    // Steer away from the closer obstacle while reversing
    int reverseSteer;

    if (left_distance <= right_distance) {
        // Left side is closer → steer LEFT while reversing (away from left wall)
        reverseSteer = SERVO_CENTER_US + MAX_STEER;
    } else {
        // Right side is closer → steer RIGHT while reversing (away from right wall)
        reverseSteer = SERVO_CENTER_US - MAX_STEER;
    }

    servo.writeMicroseconds(reverseSteer);

    // End reversing after duration
    if (millis() - reverseStart > REVERSE_DURATION_MS) {
        reversing = false;
        // Reset PID state when exiting reverse mode to prevent steering jerk
        integral = 0.0;
        previous_error = 0.0;
    }

    return; // VERY IMPORTANT → skip forward/PID logic
  }



  // ===================================================================================
  //                                     COLOR SENSING
  // ===================================================================================

        if (color_detection_on) {
          read_Color();
        }

  // ===================================================================================
  //                   OBSTACLES IN FRONT, AT CENTER OF THE ROAD
  // ===================================================================================

  // ==== IMPROVED MIDDLE SENSOR BIAS LOGIC ====
  float bias = 0.0;

  // TRAPPED SITUATION DETECTION: All three sensors blocked
  if (middle_distance < 35 && left_distance < 35 && right_distance < 35) {
      // Force immediate reversing - we're trapped!
      closeStart = millis() - STUCK_DELAY_MS;
      Serial.println("⚠️ TRAPPED! All sensors blocked - forcing reverse");
  }
  // NORMAL OBSTACLE AVOIDANCE: Middle obstacle detected
  else if (middle_distance < 80) {
      // Calculate urgency based on how close the obstacle is
      // Closer = higher urgency (0.0 at 80cm, up to ~2.67 at 0cm)
      float urgency = (80.0 - middle_distance) / 30.0;
      urgency = constrain(urgency, 0.0, 2.5);  // Cap maximum urgency

      // Calculate which side has more space
      float side_difference = right_distance - left_distance;

      // Proportional bias: combines urgency with side preference
      // If right has 30cm more space and urgency=1.0: bias = 1.0 * 20 * 1 = 20
      // If left has 30cm more space and urgency=2.0: bias = 2.0 * 20 * -1 = -40
      float direction = (side_difference > 0) ? 1.0 : -1.0;
      bias = urgency * 20.0 * direction;

      // Add extra bias if one side is significantly more open
      if (abs(side_difference) > 40) {
          bias *= 1.3;  // 30% boost for clearly better path
      }

      // Clamp bias to reasonable limits
      bias = constrain(bias, -50.0, 50.0);
  }
  
  // ===================================================================================
  //                        NORMAL FORWARD DRIVING (PID STEERING)
  // ===================================================================================

  // --- Compute PID error based on left-right difference ---
  float error = right_distance - left_distance;

  // ADD BIAS INTO ERROR (simple, clean, predictable)
  error += bias;

  // PID integral
  integral += error;
  integral = constrain(integral, -integral_limit, integral_limit);

  // PID derivative
  float derivative = error - previous_error;
  previous_error = error;

  // Raw PID output
  float pid = kp * error + ki * integral + kd * derivative;

  // ==================================TURNING AMPLIFIER=============================================
  // === PID steering amplification when reversing (not used now, but kept for logic consistency) ===
  // also when closer to the wall/ obstacle, then amplify turning
  float effective_pid = pid;

  // Convert PID to servo microseconds
  steer_output = SERVO_CENTER_US - effective_pid;

  // Clamp servo output
  steer_output = constrain(steer_output, SERVO_CENTER_US - MAX_STEER, SERVO_CENTER_US + MAX_STEER); 

  // Apply steering + speed
  servo.writeMicroseconds(steer_output);


  // ===================================================================================
  //                               MOTOR SPEED CONTROL
  // ===================================================================================

  // ===== SPEED CONTROL BY STEERING =====
  // steering deviation from center
  float steer_diff = abs(steer_output - SERVO_CENTER_US);

  // map steering angle to speed reduction (0–500 range)
  float steer_factor = (float)steer_diff / MAX_STEER;   // 0.0 = straight, 1.0 = max turn

  // clamp
  steer_factor = constrain(steer_factor, 0.0, 1.0);

  // speed multiplier (1.0 = full speed, 0.65 = slow speed)
  float speed_multiplier = 1.0 - (speed_reduction * steer_factor);

  // ==================================GYRO=============================================
    // SLOPE DETECTION - Adjust speed based on slope using RemoteXY multipliers
  float slope_multiplier = 1.0;  // Default: no slope adjustment



  // ===================================================================================
  //                     SPEED-ADAPTIVE PREDICTIVE BRAKING
  // ===================================================================================
  // Slow down BEFORE corners, not during them - prevents wall crashes at high speed

  float min_side_distance = min(left_distance, right_distance);
  float predictive_brake = 1.0;

  // Current speed as percentage (0.0 to 1.0)
  float speed_factor_brake = (speed_limit / 100.0); // e.g., 0.7 at 70% speed

  // Brake when approaching side walls (curve detection)
  if (min_side_distance < 60) {
      // Calculate proximity urgency: 60cm = safe, 20cm = critical
      float proximity = (60.0 - min_side_distance) / 40.0; // Range: 0.0 to 1.0
      proximity = constrain(proximity, 0.0, 1.0);

      // Brake harder at higher speeds (speed_factor_brake amplifies braking)
      // At 70% speed approaching 20cm wall: brake to ~65% of current speed
      predictive_brake = 1.0 - (0.5 * proximity * speed_factor_brake);
  }

  // Additional braking for front obstacles (middle sensor)
  if (middle_distance < 150) {
      // Linear scaling: 100cm = no brake, 0cm = max brake
      float middle_brake = middle_distance / 100.0; // Range: 0.0 to 1.0
      middle_brake = constrain(middle_brake, 0.4, 1.0); // Minimum 40% speed

      // Use the more restrictive brake (lower value)
      predictive_brake = min(predictive_brake, middle_brake);
  }

  // final speed for normal driving (not for reversing, reversing skips this part of the code)
  // "*5"- speed limit is from 0 to 100% and *5 is because 500 is range (from 1500 to 2000)
  // "speed multiplier"- (1.0 = full, 0.25-0.4 = slow) for precise turning without drifting
  // "predictive_brake"- (1.0 = no brake, 0.4-0.7 = heavy brake) slows before corners
  // "slope_multiplier"- (1.0 = flat, 0.5-1.0 = downhill) reduces speed on slopes
  final_speed = 1500 + (speed_limit * 5 * speed_multiplier * predictive_brake * slope_multiplier);

  // Constrain to minimum speed to prevent stopping in corners
  float min_speed_pwm = 1500 + (min_speed * 5); // Convert min_speed (0-100) to PWM (1500-2000)
  final_speed = constrain(final_speed, min_speed_pwm, 2000);

  motor.writeMicroseconds(final_speed);


  // ===================================================================================
  //                                   DEBUG PRINT
  // ===================================================================================
  #ifdef DEBUG_SERIAL_OUTPUT  // Make debug output optional
  // Single batched printf - MUCH faster than 26 separate Serial.print() calls
  Serial.printf("L:%d M:%d R:%d | Bias:%.1f Dir:%s | Err:%.1f P:%.2f I:%.2f D:%.2f | PID:%.1f | Steer:%d SteerMult:%.2f Brake:%.2f | Speed:%d\n",
    (int)left_distance, (int)middle_distance, (int)right_distance,
    bias,
    (bias > 0) ? "RIGHT" : (bias < 0) ? "LEFT" : "CENTER",
    error,
    kp * error,
    ki * integral,
    kd * derivative,
    pid,
    (int)steer_output,
    speed_multiplier,
    predictive_brake,
    (int)final_speed
  );
  #endif
  }