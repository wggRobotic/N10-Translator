#include <TRANSLATOR/translator.hpp>

using namespace std::chrono_literals;

// 0 = left front; 1 = right front; 2 = left middle; 3 = right middle; 4 = left back; 5 = right back;

translator::translator() : Node("n10_drive_translator_node") {
    //subscribers and publishers + clock
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/n10/cmd_vel", 10, std::bind(&translator::cmd_vel_callback, this, std::placeholders::_1));
  arm_state_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/n10/arm_state", 10, std::bind(&translator::arm_state_callback, this, std::placeholders::_1)); 

  motor_vel_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/n10/motor_vel", 10);
  servo_cmd_wheel_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/n10/servo_cmd_wheels", 10);
  servo_cmd_arm_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/n10/servo_cmd_arm", 10);
  wheel_timer_ = this->create_wall_timer(10ms, std::bind(&translator::wheel_timer_callback, this));
  arm_timer_ = this->create_wall_timer(100ms, std::bind(&translator::arm_timer_callback, this));
  last_call_time_ = this->now();

  RCLCPP_INFO(this->get_logger(), "Translator listening ...");
}