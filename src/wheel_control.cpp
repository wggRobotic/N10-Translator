#include <TRANSLATOR/translator.hpp>

void translator::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {

  if (verbosity) RCLCPP_INFO(this->get_logger(), "Recieved on /n10/cmd_vel {{%f, %f, %f}, {%f, %f, %f}}\n", msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z);

  last_call_time_ = this->now();

  float lin_x = msg->linear.x * velocity_linear_scalar;
  float lin_y = msg->linear.y * velocity_linear_scalar;
  float ang_vel = msg->angular.z * velocity_angular_scalar;

  vec2f wheel_vels[6];

  // ------ CALCULATION OF SPEEDS AND ANGLES ------- //

  //SPEEDS
  wheel_vels[0] = { - robot_halfwidth * ang_vel + lin_x, robot_wheel_distance * ang_vel + lin_y};
  wheel_vels[1] = { robot_halfwidth * ang_vel + lin_x, robot_wheel_distance * ang_vel + lin_y};
  wheel_vels[2] = { - robot_halfwidth * ang_vel + lin_x, lin_y};
  wheel_vels[3] = { robot_halfwidth * ang_vel + lin_x, lin_y};
  wheel_vels[4] = { - robot_halfwidth * ang_vel + lin_x, - robot_wheel_distance * ang_vel + lin_y};
  wheel_vels[5] = { robot_halfwidth * ang_vel + lin_x, - robot_wheel_distance * ang_vel + lin_y};
  
  //speed to wheel rotations per second and negations based on pointing direction  
  for (int i = 0; i < 6; i++) motor_rpms[i] = 60 * mgt(wheel_vels[i]) / (2 * robot_wheel_radius * M_PI) * (wheel_vels[i].x > 0 ? 1 : -1);

  //ANGLES
  for (int i = 0; i < 6; i++) {

    if (wheel_vels[i].y == 0) angles[i] = 0;

    else {
      if (wheel_vels[i].x == 0) angles[i] = M_PI * (wheel_vels[i].y > 0 ? 1 : -1);
      else angles[i] = atan(wheel_vels[i].y / wheel_vels[i].x);
    }

  }

}

//100 Hz
void translator::wheel_timer_callback() {
  //publish speeds
  auto vel_msg = std_msgs::msg::Float32MultiArray();
  vel_msg.data.resize(6);

  if ( (this->now().nanoseconds() - last_call_time_.nanoseconds()) / 1e6 > 100) {
    for (int i = 0; i < 6; i++) vel_msg.data[i] = 0;
  }

  else { for (int i = 0; i < 6; i++) vel_msg.data[i] = motor_rpms[i]; }

  motor_vel_pub_->publish(vel_msg);

  //publish angle
  auto angle_msg = std_msgs::msg::Float32MultiArray();
  angle_msg.data.resize(6);

  for (int i = 0; i < 6; i++) angle_msg.data[i] = angles[i];

  servo_cmd_wheel_pub_->publish(angle_msg);

}