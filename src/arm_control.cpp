#include <TRANSLATOR/translator.hpp>


void translator::arm_state_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {

  if (verbosity) RCLCPP_INFO(this->get_logger(), "recieved on /n10/arm_state {%f, %f, %f}\n", msg->data[0], msg->data[1], msg->data[2]); 
  gripper_state = msg->data[2];

  float x = msg->data[0] - arm_segment_3_length;
  float y = msg->data[1];

  float costheta = (x * x + y * y + arm_segment_1_length * arm_segment_1_length - arm_segment_2_length * arm_segment_2_length) /(2 * arm_segment_1_length * sqrt(x * x + y * y ));

  float coseta = (arm_segment_1_length * arm_segment_1_length + arm_segment_2_length * arm_segment_2_length - x * x - y * y)/(2 * arm_segment_2_length * arm_segment_1_length);

  if (1 < costheta || 1 < coseta || -1 > costheta || -1 > coseta ) {
    if (verbosity) RCLCPP_ERROR(this->get_logger(), "out of physical bounds\n")
    return;
  }

  float alpha = (x == 0 ? M_PI / 2 * (y > 0 ? 1 : -1) : atan(y / x) + M_PI * (x < 0 ? (y > 0 ? 1 : -1) : 0)) + acos(costheta);

  float beta = acos(coseta) - M_PI

  if (alpha > 2 * M_PI / 3 || alpha < - M_PI / 4 || 3 * M_PI / 4 < beta || - 3 * M_PI / 4 > beta) {
    if (verbosity) RCLCPP_ERROR(this->get_logger(), "out of allowed or servo bounds\n")
    return;
  }

  target_arm_pos.x = msg->data[0];
  target_arm_pos.y = msg->data[1];
  
}



//10 Hz
void translator::arm_timer_callback() {

  vec2f diff = {target_arm_pos.x - current_arm_pos.x, target_arm_pos.y - current_arm_pos.y};
  float diffmgt = mgt(diff);

  vec2f next_state = (diffmgt < 0.005f ? target_arm_pos : {current_arm_pos.x + (0.005f / diffmgt) * diff.x, current_arm_pos.y + (0.005f / diffmgt) * diff.y});

  float x = next_state.x - arm_segment_3_length;
  float y = next_state.y;

  auto arm_angle_msg = std_msgs::msg::Float32MultiArray();
  arm_angle_msg.data.resize(4); 

  float costheta = (x * x + y * y + arm_segment_1_length * arm_segment_1_length - arm_segment_2_length * arm_segment_2_length) /(2 * arm_segment_1_length * sqrt(x * x + y * y ));

  if()

  float coseta = (arm_segment_1_length * arm_segment_1_length + arm_segment_2_length * arm_segment_2_length - x * x - y * y)/(2 * arm_segment_2_length * arm_segment_1_length); 



  if (1 >= costheta && 1 >= coseta && -1 <= costheta && -1 <= coseta) {
    printf("next out of bounds: costheta: %f   coseta: %f", costheta, coseta);
    
  }

  
  
  if (1 >= costheta && -1 <= costheta) {}
  arm_angle_msg.data[0] = (next_state.x - arm_segment_3_length == 0 ? M_PI / 2 : ( next_state.x - arm_segment_3_length < 0 ? atan(next_state.y / (next_state.x - arm_segment_3_length)) + M_PI : atan(next_state.y / (next_state.x - arm_segment_3_length)) )) + acos(costheta) - M_PI / 2;
  arm_angle_msg.data[1] = acos(coseta) - M_PI;
  arm_angle_msg.data[2] = -M_PI / 2 - arm_angle_msg.data[0] - arm_angle_msg.data[1];
  arm_angle_msg.data[3] = gripper_state;
  servo_cmd_arm_pub_->publish(arm_angle_msg); 

  current_arm_pos = next_state;
  return;
}