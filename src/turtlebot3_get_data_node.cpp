#include "turtlebot3_cpp/turtlebot3_get_data.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<turtlebot3_MPC>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
