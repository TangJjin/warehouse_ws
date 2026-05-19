#include "rclcpp/rclcpp.hpp"
#include "drone_qt_2/airborne_node.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AirborneNode>());
    rclcpp::shutdown();
    return 0;
}