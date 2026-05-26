#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "drone_perception/qr_vision_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QrVisionNode>());
  rclcpp::shutdown();
  return 0;
}
