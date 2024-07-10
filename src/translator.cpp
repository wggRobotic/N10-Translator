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

#define halfwith 0.105f
#define wheeldistance 0.155f
#define wheelradius 0.05f

#define seglength1 0.105f
#define seglength2 0.13f

struct vec2f {
  float x;
  float y;
};

float mgt(vec2f v) {return sqrt(v.x * v.x + v.y * v.y);}

using namespace std::chrono_literals;

// 0 = left front; 1 = right front; 2 = left middle; 3 = right middle; 4 = left back; 5 = right back;

//translator node
class translator : public rclcpp::Node {
  public:
    translator() : Node("n10_drive_translator_node") {
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

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
      last_call_time_ = this->now();

      float ang_vel = msg->angular.z * M_PI;
      float lin_x = msg->linear.x;
      float lin_y = msg->linear.y;

      vec2f wheel_vels[6];

      // ------ CALCULATION OF SPEEDS AND ANGLES -------//

      //SPEEDS
      wheel_vels[0] = { -halfwith * ang_vel + lin_x, wheeldistance * ang_vel + lin_y};
      wheel_vels[1] = { halfwith * ang_vel + lin_x, wheeldistance * ang_vel + lin_y};
      wheel_vels[2] = { -ang_vel * halfwith + lin_x, lin_y};
      wheel_vels[3] = { ang_vel * halfwith + lin_x, lin_y};
      wheel_vels[4] = { -halfwith * ang_vel + lin_x, -wheeldistance * ang_vel + lin_y};
      wheel_vels[5] = { halfwith * ang_vel + lin_x, -wheeldistance * ang_vel + lin_y};

      //ANGLES
      for (int i = 0; i < 6; i++) {
        if (wheel_vels[i].x == 0) {
          if (wheel_vels[i].y > 0) angles[i] = M_PI / 2;
          else if (wheel_vels[i].y < 0) angles[i] = -M_PI / 2;
          else angles[i] = 0;
        }
          
        else {
          angles[i] = atan(wheel_vels[i].y / wheel_vels[i].x);
        }
      }

      //speed to wheel rotations per second and negations based on pointing direction  
      for (int i = 0; i < 6; i++) {
        motor_vels[i] = 60 * mgt(wheel_vels[i]) / (2 * wheelradius * M_PI);
        if (wheel_vels[i].x < 0) motor_vels[i] *= -1; 
      }

    }

    void arm_state_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
      target_arm_pos.x = msg->data[0];
      target_arm_pos.y = msg->data[1];
      gripper_state = msg->data[2];
      RCLCPP_INFO(this->get_logger(), "recieved..."); 
    }

    //get called every 10ms
    void wheel_timer_callback() {
        //publish speeds
      auto vel_msg = std_msgs::msg::Float32MultiArray();
      vel_msg.data.resize(6);

      if ( (this->now().nanoseconds() - last_call_time_.nanoseconds()) / 1e6 > 100) {
        for (int i = 0; i < 6; i++) vel_msg.data[i] = 0;
      }

      else { for (int i = 0; i < 6; i++) vel_msg.data[i] = motor_vels[i]; }

      motor_vel_pub_->publish(vel_msg);

      //publish angle
      auto angle_msg = std_msgs::msg::Float32MultiArray();
      angle_msg.data.resize(6);

      for (int i = 0; i < 6; i++) {
        angle_msg.data[i] = angles[i];
      }

      servo_cmd_wheel_pub_->publish(angle_msg);

    }
    //every 100ms
    void arm_timer_callback() {
      if(target_arm_pos.x < -0.001) return;

      auto arm_angle_msg = std_msgs::msg::Float32MultiArray();
      arm_angle_msg.data.resize(3); 

      vec2f diff = {target_arm_pos.x - current_arm_pos.x, target_arm_pos.y - current_arm_pos.y};
      float diffmgt = mgt(diff);

      vec2f next_state;

      if(diffmgt < 0.005f) next_state = target_arm_pos;
      else next_state = {current_arm_pos.x + (0.005f / diffmgt) * diff.x, current_arm_pos.y + (0.005f / diffmgt) * diff.y};

      float costheta = (next_state.x * next_state.x + next_state.y * next_state.y + seglength1 * seglength1 - seglength2 * seglength2)/(2 * seglength1 * sqrt(next_state.x * next_state.x + next_state.y * next_state.y ));
      float coseta = (seglength1 * seglength1 + seglength2 * seglength2 - next_state.x * next_state.x - next_state.y * next_state.y)/(2 * seglength2 * seglength1);

      if(-1 <= costheta && -1 <= coseta && 1 >= costheta && 1 >= coseta) {
        current_arm_pos = next_state;
        arm_angle_msg.data[0] = atan(next_state.y / next_state.x) + acos(costheta) - M_PI / 2;
        arm_angle_msg.data[1] = acos(coseta) - M_PI;
        arm_angle_msg.data[2] = gripper_state;
        servo_cmd_arm_pub_->publish(arm_angle_msg); 
      }
      else  printf("costheta: %f   coseta: %f", costheta, coseta);
    }

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

    vec2f current_arm_pos = {seglength2 + seglength1, 0};
    vec2f target_arm_pos = {seglength2 + seglength1, 0};
    float gripper_state;

    rclcpp::Time last_call_time_;
};


int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<translator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
