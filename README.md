# Description
The translator node for calculating the speeds and angles of the 6 wheels and the robot arm

ros2 name: `n10_drive_translator_node`

launch with :
```
ros2 launch translator translator.launch.py
```

# Interface

## subscribed
- `/n10/cmd_vel` `geometry_msgs::msg::Twist` `100 Hz expected` : linear and angular velocity of the robot
  - `inear.x` : from `-1 ≐ -1 m/s` to `1 ≐ 1 m/s`
  - `linear.y` : from `-1 ≐ -1 m/s` to `1 ≐ 1 m/s`
  - `angular.z` : from `-1 ≐ -π/2 m/s` to `1 ≐ π/2 m/s` counterclockwise 

- `/n10/arm_state` `std_msgs::msg::Float32MultiArray (4)` : absolute arm position, ground_angle and grabber state 

## publishing
- `/n10/motor_vel` `std_msgs::msg::Float32MultiArray (6)` `100 Hz` : wheel RPMs

  - publishes 0 after `100ms` not `/n10/cmd_vel` recieved

- `/n10/servo_cmd_wheels` `std_msgs::msg::Float32MultiArray (6)` `100 Hz` : wheel angles

  - from `-π/2 ≐ pointing right` to `π/2 ≐ pointing left`

-  `/n10/servo_cmd_arm` `std_msgs::msg::Float32MultiArray (4)` `100 Hz` : arm_joint angles and grabber servo angle

# Parameters
Can be set in `parameter/translator.yaml`

# Simulation
- `plot/velocity_plot.py` for visualization of velocity vectors for the wheels published to `/n10/servo_cmd_wheels` and `/n10/motor_vel`

- `plot/arm_plot.py` for visualization of arm angles published to `/n10/servo_cmd_arm` 
