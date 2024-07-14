#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <stdio.h>
#include <cmath>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "rclcpp/clock.hpp"
#include "builtin_interfaces/msg/time.hpp"

struct vec2f {
  float x;
  float y;
};

float mgt(vec2f v);

class translator : public rclcpp::Node {
  public:
    translator();

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    void arm_state_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    void wheel_timer_callback();

    void arm_timer_callback();

  private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr arm_state_sub_;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr motor_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr servo_cmd_wheel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr servo_cmd_arm_pub_;
    rclcpp::TimerBase::SharedPtr wheel_timer_;
    rclcpp::TimerBase::SharedPtr arm_timer_;
    
    float motor_vels[6] = {0, 0, 0, 0, 0, 0};
    float angles[6] = {0, 0, 0, 0, 0, 0};

    vec2f current_arm_pos;
    vec2f target_arm_pos;
    float gripper_state;

    rclcpp::Time last_call_time_;

    //parameter
    bool verbosity;

    float robot_halfwidth;
    float robot_wheel_distance;
    float robot_wheel_radius;

    float velocity_linear_scalar;
    float velocity_angular_scalar;

    float arm_segment_1_length;
    float arm_segment_2_length;
    float arm_segment_3_length;
};