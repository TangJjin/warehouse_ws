#include <QApplication>
#include <thread>

#include "drone_qt/mainwindow.hpp"
#include <rclcpp/rclcpp.hpp>
#include "drone_qt/ground_link_bridge.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    //创建ROS管理器对象，并启动ROS spinning线程
    auto ros_manager = std::make_shared<RosManager>();
    ros_manager->start();

    //创建GroundLinkBridge对象，并启动一个线程来运行ROS spinning
    auto ground_bridge = std::make_shared<GroundLinkBridge>();
    std::thread bridge_spin_thread([ground_bridge]() {
        rclcpp::spin(ground_bridge);
    });

    //创建主窗口对象，并显示界面
    MainWindow window(ros_manager);
    window.show();

    //进入Qt事件循环，等待用户操作
    const int ret = app.exec();

    //退出Qt事件循环后，关闭ROS spinning，并等待线程结束
    rclcpp::shutdown();
    if (bridge_spin_thread.joinable()) {
        bridge_spin_thread.join();
    }
    return ret;
}