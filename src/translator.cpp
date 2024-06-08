#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <stdio.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
//mode 0: skid
//mode 1: servo

struct vec2f {
  float x;
  float y;
};

float mgt(vec2f v) {return sqrt(v.x * v.x + v.y * v.y);}

float halfwith = 0.1;
float wheeldistance = 0.1;
float wheelradius = 0.06;


class translator : public rclcpp::Node {
  public:
    translator() : Node("n10_drive_translator_node") {
      cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/n10/cmd_vel", 10, std::bind(&translator::cmd_vel_callback, this, std::placeholders::_1));
      drive_enable_sub_ = this->create_subscription<std_msgs::msg::Bool>("/n10/drive_enable", 10, std::bind(&translator::drive_enable_callback, this, std::placeholders::_1));
      servo_enable_sub_ = this->create_subscription<std_msgs::msg::Bool>("/n10/servo_enable", 10, std::bind(&translator::servo_enable_callback, this, std::placeholders::_1));
      servo_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/n10/servo_cmd_vel", 10);
      motor_vel_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/n10/motor_vel", 10);
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
      if(drive_bool) {
        // 0 = left front; 1 = right front; 2 = left middle; 3 = right middle; 4 = left back; 5 = right back;
        float ang_vel = msg->angular.z;
        float lin_x = msg->linear.x;
        float lin_y = msg->linear.y;
        vec2f wheel_vels[6];

        //velocity vectors

        wheel_vels[0] = { -halfwith * ang_vel + lin_x, wheeldistance * ang_vel + lin_y};
        wheel_vels[1] = { halfwith * ang_vel + lin_x, wheeldistance * ang_vel + lin_y};
        wheel_vels[2] = { -ang_vel * halfwith + lin_x, lin_y};
        wheel_vels[3] = { ang_vel * halfwith + lin_x, lin_y};
        wheel_vels[4] = { -halfwith * ang_vel + lin_x, -wheeldistance * ang_vel + lin_y};
        wheel_vels[5] = { halfwith * ang_vel + lin_x, -wheeldistance * ang_vel + lin_y};

        auto motor_msg = std_msgs::msg::Float32MultiArray();
        motor_msg.data.resize(6, 0.0);
        
        //wheel rotations per second and negations based on pointing direction
        
        for(int i = 0; i < 6; i++) {
          motor_msg.data[i] = 60 * mgt(wheel_vels[i]) / (2 * wheelradius * M_PI);
          if(wheel_vels[i].x < 0) motor_msg.data[i] *= -1; 

        }
        
        motor_vel_pub_->publish(motor_msg);

        //for servocontrol
        if(servo_bool) servo_cmd_vel_pub_->publish(*msg);
      }

    }

    void drive_enable_callback(const std_msgs::msg::Bool::SharedPtr msg) {drive_bool = msg->data;}
    
    void servo_enable_callback(const std_msgs::msg::Bool::SharedPtr msg) {
      if(msg->data == false && servo_bool == true) {
        auto msg = std::make_shared<geometry_msgs::msg::Twist>();
        msg->linear.x = 1;
        msg->linear.y = 0;
        msg->angular.z = 0;
        servo_cmd_vel_pub_->publish(*msg);
      }
      servo_bool = msg->data;

    }


  private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr drive_enable_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr servo_enable_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr servo_cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr motor_vel_pub_;

    bool drive_bool = true;
    bool servo_bool = true;
};


int main(int argc, char * argv[]) {
  
  rclcpp::init(argc, argv);
  auto node = std::make_shared<translator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
