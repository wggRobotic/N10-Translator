#include <TRANSLATOR/translator.hpp>

void translator::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  last_call_time_ = this->now();

  float ang_vel = msg->angular.z * M_PI;
  float lin_x = msg->linear.x;
  float lin_y = msg->linear.y;

  vec2f wheel_vels[6];

  // ------ CALCULATION OF SPEEDS AND ANGLES -------//

  //SPEEDS
  wheel_vels[0] = { -halfwith * ang_vel + lin_x, wheeldistance * ang_vel + lin_y};
  wheel_vels[1] = { halfwith * ang_vel + lin_x, wheeldistance * ang_vel + lin_y};
  wheel_vels[2] = { -ang_vel * halfwith + lin_x, lin_y};
  wheel_vels[3] = { ang_vel * halfwith + lin_x, lin_y};
  wheel_vels[4] = { -halfwith * ang_vel + lin_x, -wheeldistance * ang_vel + lin_y};
  wheel_vels[5] = { halfwith * ang_vel + lin_x, -wheeldistance * ang_vel + lin_y};

  //ANGLES
  for (int i = 0; i < 6; i++) {
    if (wheel_vels[i].x == 0) {
      if (wheel_vels[i].y > 0) angles[i] = M_PI / 2;
        else if (wheel_vels[i].y < 0) angles[i] = -M_PI / 2;
        else angles[i] = 0;
    }
          
    else {
      angles[i] = atan(wheel_vels[i].y / wheel_vels[i].x);
    }
  }

  //speed to wheel rotations per second and negations based on pointing direction  
  for (int i = 0; i < 6; i++) {
    motor_vels[i] = 60 * mgt(wheel_vels[i]) / (2 * wheelradius * M_PI);
    if (wheel_vels[i].x < 0) motor_vels[i] *= -1; 
  }

}

void translator::wheel_timer_callback() {
    //publish speeds
  auto vel_msg = std_msgs::msg::Float32MultiArray();
  vel_msg.data.resize(6);

  if ( (this->now().nanoseconds() - last_call_time_.nanoseconds()) / 1e6 > 100) {
    for (int i = 0; i < 6; i++) vel_msg.data[i] = 0;
  }

  else { for (int i = 0; i < 6; i++) vel_msg.data[i] = motor_vels[i]; }

  motor_vel_pub_->publish(vel_msg);

  //publish angle
  auto angle_msg = std_msgs::msg::Float32MultiArray();
  angle_msg.data.resize(6);

  for (int i = 0; i < 6; i++) {
    angle_msg.data[i] = angles[i];
  }

  servo_cmd_wheel_pub_->publish(angle_msg);

}