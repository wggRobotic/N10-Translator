#include <TRANSLATOR/translator.hpp>


void translator::arm_state_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  target_arm_pos.x = msg->data[0];
  target_arm_pos.y = msg->data[1];
  gripper_state = msg->data[2];
  RCLCPP_INFO(this->get_logger(), "recieved..."); 
}

//every 100ms
void translator::arm_timer_callback() {

  float costheta_whole = ((target_arm_pos.x - seglength3) * (target_arm_pos.x - seglength3) + target_arm_pos.y * target_arm_pos.y + seglength1 * seglength1 - seglength2 * seglength2)/(2 * seglength1 * sqrt((target_arm_pos.x - seglength3) * (target_arm_pos.x - seglength3) + target_arm_pos.y * target_arm_pos.y ));
  float coseta_whole = (seglength1 * seglength1 + seglength2 * seglength2 - (target_arm_pos.x - seglength3) * (target_arm_pos.x - seglength3) - target_arm_pos.y * target_arm_pos.y)/(2 * seglength2 * seglength1);

  if(1 < costheta_whole || 1 < coseta_whole || -1 > costheta_whole || -1 > coseta_whole) {
    printf("targer out of bounds :costheta: %f   coseta: %f", costheta_whole, coseta_whole);
    return;
  }


  vec2f diff = {target_arm_pos.x - current_arm_pos.x, target_arm_pos.y - current_arm_pos.y};
  float diffmgt = mgt(diff);

  vec2f next_state;

  if(diffmgt < 0.005f) next_state = target_arm_pos;
  else next_state = {current_arm_pos.x + (0.005f / diffmgt) * diff.x, current_arm_pos.y + (0.005f / diffmgt) * diff.y};

  float costheta = ((next_state.x - seglength3) * (next_state.x - seglength3) + next_state.y * next_state.y + seglength1 * seglength1 - seglength2 * seglength2)/(2 * seglength1 * sqrt((next_state.x - seglength3) * (next_state.x - seglength3) + next_state.y * next_state.y ));
  float coseta = (seglength1 * seglength1 + seglength2 * seglength2 - (next_state.x - seglength3) * (next_state.x - seglength3) - next_state.y * next_state.y)/(2 * seglength2 * seglength1);
      

  if(1 < costheta || 1 < coseta || -1 > costheta || -1 > coseta) {
    printf("next out of bounds: costheta: %f   coseta: %f", costheta, coseta);
    return;
  }

  auto arm_angle_msg = std_msgs::msg::Float32MultiArray();
  arm_angle_msg.data.resize(4); 
  current_arm_pos = next_state;
  arm_angle_msg.data[0] = (next_state.x - seglength3 == 0 ? M_PI / 2 : ( next_state.x - seglength3 < 0 ? atan(next_state.y / (next_state.x - seglength3)) + M_PI : atan(next_state.y / (next_state.x - seglength3)) )) + acos(costheta) - M_PI / 2;
  arm_angle_msg.data[1] = acos(coseta) - M_PI;
  arm_angle_msg.data[2] = -M_PI / 2 - arm_angle_msg.data[0] - arm_angle_msg.data[1];
  arm_angle_msg.data[3] = gripper_state;
  servo_cmd_arm_pub_->publish(arm_angle_msg); 

}