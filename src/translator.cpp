#include <TRANSLATOR/translator.hpp>

using namespace std::chrono_literals;

// 0 = left front; 1 = right front; 2 = left middle; 3 = right middle; 4 = left back; 5 = right back;

translator::translator() : Node("n10_drive_translator_node") {
  // Declare parameters
  this->declare_parameter<bool>("verbosity", false);

  this->declare_parameter<float>("robot.halfwidth", 0.0f);
  this->declare_parameter<float>("robot.wheel_distance", 0.0f);
  this->declare_parameter<float>("robot.wheel_radius", 0.0f);

  this->declare_parameter<float>("velocity_conversion.linear_scalar", 0.0f);
  this->declare_parameter<float>("velocity_conversion.angular_scalar", 0.0f);

  this->declare_parameter<float>("arm.segment_1_length", 0.0f);
  this->declare_parameter<float>("arm.segment_2_length", 0.0f);

  this->declare_parameter<float>("arm.initial_x", 0.0f);
  this->declare_parameter<float>("arm.initial_y", 0.0f);
  this->declare_parameter<float>("arm.initial_ground_angle", 0.0f);

  // Get parameters
  verbosity = this->get_parameter("verbosity").as_bool();

  robot_halfwidth = this->get_parameter("robot.halfwidth").as_double();
  robot_wheel_distance = this->get_parameter("robot.wheel_distance").as_double();
  robot_wheel_radius = this->get_parameter("robot.wheel_radius").as_double();

  velocity_linear_scalar = this->get_parameter("velocity_conversion.linear_scalar").as_double();
  velocity_angular_scalar = this->get_parameter("velocity_conversion.angular_scalar").as_double();

  arm_segment_1_length = this->get_parameter("arm.segment_1_length").as_double();
  arm_segment_2_length = this->get_parameter("arm.segment_2_length").as_double();
  
  arm_initial_x = this->get_parameter("arm.initial_x").as_double();
  arm_initial_y = this->get_parameter("arm.initial_y").as_double();
  arm_ground_angle = this->get_parameter("arm.initial_ground_angle").as_double();

  current_arm_pos = {arm_initial_x, arm_initial_y};
  target_arm_pos = {arm_initial_x, arm_initial_y};

  last_alpha = M_PI / 2;
  last_beta = 0;
  last_gamma = arm_ground_angle - last_alpha - last_beta;

  //subscribers and publishers + clock
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/n10/cmd_vel", 10, std::bind(&translator::cmd_vel_callback, this, std::placeholders::_1));
  arm_state_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/n10/arm_state", 10, std::bind(&translator::arm_state_callback, this, std::placeholders::_1)); 

  motor_vel_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/n10/motor_vel", 10);
  servo_cmd_wheel_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/n10/servo_cmd_wheels", 10);
  servo_cmd_arm_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/n10/servo_cmd_arm", 10);
  wheel_timer_ = this->create_wall_timer(10ms, std::bind(&translator::wheel_timer_callback, this));
  arm_timer_ = this->create_wall_timer(10ms, std::bind(&translator::arm_timer_callback, this));
  last_call_time_ = this->now();

  RCLCPP_INFO(this->get_logger(), "Translator listening ...\n");
}