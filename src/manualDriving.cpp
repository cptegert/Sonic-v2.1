#include "manualDriving.h" //main headder file for this
#include "globals.h" //for seeing left speed, rightspeed, move motors
//#include <ESP32Servo.h> //servo juhtimiseks?
#include <Arduino.h> //for serial printing
extern Servo motor;  // 👈 this tells the compiler motor is declared elsewhere
extern Servo servo; 


void manual_mode(){

//rattamootorite kiiruse reguleerimine telefoni joystickiga
  final_speed = map (joystick_speed_y, -100, 100, 1500-(speed_limit*5), 1500+(speed_limit*5)); //1000 max speed reverse, 1500 idle and 2000 max speed forward
  motor.writeMicroseconds(final_speed);

//esirataste keeramise juhtimine telefoni joystickiga
  steer_output = map (joystick_steering_x, 100, -100, SERVO_MIN_US, SERVO_MAX_US); //joistick controlling steering...  -100, 2000 is left;   0,1500 is centre;  100,2000 is right
    servo.writeMicroseconds(steer_output);

  Serial.println("Manual mode confirmed");
  //Serial.println(joystick_speed_y);
  Serial.println(steer_output);
}
