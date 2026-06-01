#include <rclcpp/rclcpp.hpp>
#include "drone_qt/ground_link_bridge.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GroundLinkBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}