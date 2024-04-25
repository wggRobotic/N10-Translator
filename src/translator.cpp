#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"


class translator : public rclcpp::Node
{
public:
  translator() : Node("n10_drive_translator_node") {
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/n10/cmd_vel", 10, std::bind(&translator::cmd_vel_callback, this, std::placeholders::_1));
    drive_enable_sub_ = this->create_subscription<std_msgs::msg::Bool>("/n10/drive_enable", 10, std::bind(&translator::drive_enable_callback, this, std::placeholders::_1));
    servo_enable_sub_ = this->create_subscription<std_msgs::msg::Bool>("/n10/servo_enable", 10, std::bind(&translator::servo_enable_callback, this, std::placeholders::_1));
    servo_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/n10/servo_cmd_vel", 10);
    //profmay_pub_ = this->create_publisher<??>("??", 10);
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
      if(drive_bool) {
        if(servo_bool) servo_cmd_vel_pub_->publish(*msg);
      }
    }
    void drive_enable_callback(const std_msgs::msg::Bool::SharedPtr msg) {drive_bool = msg->data;}
    void servo_enable_callback(const std_msgs::msg::Bool::SharedPtr msg) {servo_bool = msg->data;}
    

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr drive_enable_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr servo_enable_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr servo_cmd_vel_pub_;
  //rclcpp::Publisher<??>::SharedPtr profmay_pub_;

  bool drive_bool = true;
  bool servo_bool = true;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<translator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
