#include <rclcpp/rclcpp.hpp>
#include "drone_qt_2/airborne_link_bridge.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AirborneLinkBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}