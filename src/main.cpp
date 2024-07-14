#include <TRANSLATOR/translator.hpp>

float mgt(vec2f v) {return sqrt(v.x * v.x + v.y * v.y);}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<translator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}