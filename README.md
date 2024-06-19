internal variables
  float motor_vels[6];
  
  float angles[6];
  
  float last_angles[6];
  
  bool servo_bool = true;
  
  bool servo_currently_moving = false; 


subscribed:
- /n10/cmd_vel Twist: get driving data

  callback: calculate speeds and angles based on servo_bool, write in motor_vels and angles

  x = 1 => 1m/s;

  y = 1 => 1m/s;

  ang = 1 => pi rad/s counterclockwise
  
- /n10/servo_enable Bool: servomode on and off

  callback: set servo_bool, set angles to 0
  
- timer

  callback: (every 10ms) if no new data or servo turning, publish 0 speed; publish speeds; if angles changed, publish angles and set servo_currently_moving true; if 2 seconds have passed, set it to false

publisher:
- /n10/motor_vel Float32MultiArray length 6: rpms of wheels for edu_drive
- /n10/servo_cmd_wheels Float32MultiArray length 6: angles for servos

IMPORTANT
use velocity_plot.py to visualize the velocity vectors of the wheels published to /n10/servo_cmd_angle and - /n10/motor_vel
