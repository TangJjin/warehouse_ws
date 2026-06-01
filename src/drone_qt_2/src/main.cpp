#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include "drone_qt_2/airborne_node.hpp"
#include "drone_qt_2/airborne_link_bridge.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto airborne_node = std::make_shared<AirborneNode>();
    auto airborne_link_bridge = std::make_shared<AirborneLinkBridge>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(airborne_node);
    executor.add_node(airborne_link_bridge);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}