#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <stdio.h>
#include <cmath>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "rclcpp/clock.hpp"
#include "builtin_interfaces/msg/time.hpp"

struct vec2f {
  float x;
  float y;
};

float mgt(vec2f v) {return sqrt(v.x * v.x + v.y * v.y);}

using namespace std::chrono_literals;

const float halfwith = 0.11;
const float wheeldistance = 0.16;
const float wheelradius = 0.056;

class translator : public rclcpp::Node {
  public:
    translator() : Node("n10_drive_translator_node") {
      cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/n10/cmd_vel", 10, std::bind(&translator::cmd_vel_callback, this, std::placeholders::_1));
      servo_enable_sub_ = this->create_subscription<std_msgs::msg::Bool>("/n10/servo_enable", 10, std::bind(&translator::servo_enable_callback, this, std::placeholders::_1));
      servo_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/n10/servo_cmd_vel", 10);
      motor_vel_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/n10/motor_vel", 10);
      angel_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/n10/servo_cmd_angle", 10);
      timer_ = this->create_wall_timer(10ms, std::bind(&translator::timer_callback, this));
      last_call_time_ = this->now();
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
      last_call_time_ = this->now();
      if(drive_bool) {
        // 0 = left front; 1 = right front; 2 = left middle; 3 = right middle; 4 = left back; 5 = right back;
        float ang_vel = msg->angular.z;
        float lin_x = msg->linear.x;
        float lin_y = msg->linear.y * M_PI/2;
        vec2f wheel_vels[6];

        //LOGIC
        if(servo_bool) {

          wheel_vels[0] = { -halfwith * ang_vel + lin_x, wheeldistance * ang_vel + lin_y};
          wheel_vels[1] = { halfwith * ang_vel + lin_x, wheeldistance * ang_vel + lin_y};
          wheel_vels[2] = { -ang_vel * halfwith + lin_x, lin_y};
          wheel_vels[3] = { ang_vel * halfwith + lin_x, lin_y};
          wheel_vels[4] = { -halfwith * ang_vel + lin_x, -wheeldistance * ang_vel + lin_y};
          wheel_vels[5] = { halfwith * ang_vel + lin_x, -wheeldistance * ang_vel + lin_y};

          //calculate angles

          for(int i = 0; i < 6; i++) {
            if(wheel_vels[i].x == 0) {
              if(wheel_vels[i].y > 0) angles[i] = M_PI / 2;
              else if (wheel_vels[i].y < 0) angles[i] = -M_PI / 2;
              else angles[i] = 0;
            }
            else {
              angles[i] = atan(wheel_vels[i].y / wheel_vels[i].x);
            }
          }
        }

        else {
          wheel_vels[0] = { -halfwith * ang_vel + lin_x, 0};
          wheel_vels[1] = { halfwith * ang_vel + lin_x, 0};
          wheel_vels[2] = { -ang_vel * halfwith + lin_x, 0};
          wheel_vels[3] = { ang_vel * halfwith + lin_x, 0};
          wheel_vels[4] = { -halfwith * ang_vel + lin_x, 0};
          wheel_vels[5] = { halfwith * ang_vel + lin_x, 0};
        }

        //wheel rotations per second and negations based on pointing direction  
        for(int i = 0; i < 6; i++) {
          motor_vels[i] = 60 * mgt(wheel_vels[i]) / (2 * wheelradius * M_PI);
          if(wheel_vels[i].x < 0) motor_vels[i] *= -1; 
        }

      }
    }
    
    void timer_callback() {
        auto vel_msg = std::make_shared<Float32MultiArray>();
        vel_msg.resize(6);
        if((this->now() - last_call_time_) / 1e6 < 100 || !servo_currently_moving) {for (int i = 0; i < 6; i++) vel_msg[i] = 0;}

        else {
        vel_msg[0] = motor_vels[0];
        vel_msg[1] = motor_vels[1];
        vel_msg[2] = motor_vels[2];
        vel_msg[3] = motor_vels[3];
        vel_msg[4] = motor_vels[4];
        vel_msg[5] = motor_vels[5];
        }

        motor_vel_pub_->publish(*vel_msg);

        if((angles[0] != last_angles[0] || angles[1] != last_angles[1] || angles[2] != last_angles[2] || angles[3] != last_angles[3] || angles[4] != last_angles[4] || angles[5] != last_angles[5] ) && !servo_currently_moving) {
          auto angle_msg = std::make_shared<Float32MultiArray>();
          vel_msg.resize(6);

          for(int i = 0; i < 6; i++) {
            angles_new[i] = angles[i];
            vel_msg[i] = angles[i];
          }

          angel_pub_->publish(*angle_msg);

          servo_currently_moving = true;
        }
    }
    
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
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr servo_enable_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr servo_cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr motor_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr angel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    //needs to be made thread safe
    //
    bool servo_bool = true;

    float angles[6] = {0, 0, 0, 0, 0, 0};
    float motor_vels[6] = {0, 0, 0, 0, 0, 0};

    bool servo_currently_moving = false;
    
    float last_angles[6] = {0, 0, 0, 0, 0, 0};

    rclcpp::Time last_call_time_;

};

int main(int argc, char * argv[]) {
  
  rclcpp::init(argc, argv);
  auto node = std::make_shared<translator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
