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
  translator() : Node("translator") {
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("n10/cmd_vel", 10, std::bind(&translator::cmd_vel_callback, this, std::placeholders::_1));
    ServoOn_sub_ = this->create_subscription<std_msgs::msg::Bool>("ServoOn", 10, std::bind(&translator::ServoOn_callback, this, std::placeholders::_1));
    servo_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("n10/servo_cmd_vel", 10);
    //profmay_pub_ = this->create_publisher<??>("??", 10);
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
      if(servo_bool) servo_pub_->publish(*msg);
    }

    void ServoOn_callback(const std_msgs::msg::Bool::SharedPtr msg) {servo_bool = msg->data;}
    

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ServoOn_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr servo_pub_;
  //rclcpp::Publisher<??>::SharedPtr profmay_pub_;

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
