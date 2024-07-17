#include <TRANSLATOR/translator.hpp>


void translator::arm_state_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {

  if (verbosity) RCLCPP_INFO(this->get_logger(), "Recieved on /n10/arm_state {%f, %f, %f}\n", msg->data[0], msg->data[1], msg->data[2]); 
  
  gripper_state = msg->data[3];

  float x = msg->data[0];
  float y = msg->data[1];

  float costheta = (x * x + y * y + arm_segment_1_length * arm_segment_1_length - arm_segment_2_length * arm_segment_2_length) /(2 * arm_segment_1_length * sqrt(x * x + y * y ));

  float coseta = (arm_segment_1_length * arm_segment_1_length + arm_segment_2_length * arm_segment_2_length - x * x - y * y)/(2 * arm_segment_2_length * arm_segment_1_length);

  if (1 < costheta || 1 < coseta || -1 > costheta || -1 > coseta ) {
    if (verbosity) RCLCPP_ERROR(this->get_logger(), "Out of physical bounds\n");
    return;
  }

  float alpha = (x == 0 ? M_PI / 2 * (y > 0 ? 1 : -1) : atan(y / x) + M_PI * (x < 0 ? (y > 0 ? 1 : -1) : 0)) + acos(costheta);

  float beta = acos(coseta) - M_PI;

  if (alpha > 7 * M_PI / 12 || alpha < - M_PI / 4 || beta > 3 * M_PI / 4 || beta < - 3 * M_PI / 4 ) {
    if (verbosity) {
      RCLCPP_ERROR(this->get_logger(), "Out of allowed or servo bounds\n");
      if (alpha > 7 * M_PI / 12 || alpha < - M_PI / 4) RCLCPP_INFO(this->get_logger(), "Alpha out of bounds. should be in [%f, %f] but is %f\n", - M_PI / 4, 7 * M_PI / 12, alpha);
      if (beta > 3 * M_PI / 4 || beta < - 3 * M_PI / 4) RCLCPP_INFO(this->get_logger(), "Beta out of bounds. should be in [%f, %f] but is %f\n", - 3 * M_PI / 4, 3 * M_PI / 4, beta);
      }
    return;
  }

  float gamma = - beta - alpha + msg->data[2];

  if(gamma >  M_PI / 3  || gamma < - M_PI / 2) {
    if (verbosity) {RCLCPP_ERROR(this->get_logger(), "Compensation joint out of bounds %f, %f, %f\n", alpha, beta, gamma);}
    return;
  }

   if (verbosity) RCLCPP_INFO(this->get_logger(), "Moving alpha to %f, beta to %f\n", alpha, beta);
  
  arm_ground_angle = msg->data[2];

  target_arm_pos.x = msg->data[0];
  target_arm_pos.y = msg->data[1];
  
}



//100 Hz
void translator::arm_timer_callback() {

  vec2f diff = {target_arm_pos.x - current_arm_pos.x, target_arm_pos.y - current_arm_pos.y};
  float diffmgt = mgt(diff);

  vec2f next_state = {current_arm_pos.x + (0.0005f / diffmgt) * diff.x, current_arm_pos.y + (0.0005f / diffmgt) * diff.y};
  if(diffmgt < 0.0005f) next_state = target_arm_pos;

  float x = next_state.x;
  float y = next_state.y;

  auto arm_angle_msg = std_msgs::msg::Float32MultiArray();
  arm_angle_msg.data.resize(4); 

  //alpha

  float alpha = last_alpha;

  float costheta = (x * x + y * y + arm_segment_1_length * arm_segment_1_length - arm_segment_2_length * arm_segment_2_length) /(2 * arm_segment_1_length * sqrt(x * x + y * y ));

  if(costheta <= 1 && costheta >= -1) {
    float new_alpha = (x == 0 ? M_PI / 2 * (y > 0 ? 1 : -1) : atan(y / x) + M_PI * (x < 0 ? (y > 0 ? 1 : -1) : 0)) + acos(costheta);
    if(new_alpha <= 7 * M_PI / 12 && new_alpha >= - M_PI / 4) {alpha = new_alpha; last_alpha = new_alpha;}
  }
  
  //beta

  float beta = last_beta;

  float coseta = (arm_segment_1_length * arm_segment_1_length + arm_segment_2_length * arm_segment_2_length - x * x - y * y)/(2 * arm_segment_2_length * arm_segment_1_length); 

  if(coseta <= 1 && coseta >= -1) {
    float new_beta = acos(coseta) - M_PI;
    if(new_beta <= 3 * M_PI / 4 && new_beta >= - 3 * M_PI / 4 ) {beta = new_beta; last_beta = new_beta;}
    
  }

  //gamma

  float gamma = last_gamma;

  float new_gamma = - alpha - beta + arm_ground_angle;

  if(new_gamma <= M_PI / 3 && new_gamma >= - M_PI / 2) { gamma = new_gamma; last_gamma = new_gamma;}

  arm_angle_msg.data[0] = alpha - M_PI / 2;
  arm_angle_msg.data[1] = beta;
  arm_angle_msg.data[2] = gamma;
  arm_angle_msg.data[3] = gripper_state;
  
  servo_cmd_arm_pub_->publish(arm_angle_msg); 

  current_arm_pos = next_state;
  return;
}
