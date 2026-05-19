#include <QApplication>

#include "drone_qt/mainwindow.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    int ret = 0;
    {
        QApplication app(argc, argv);
        MainWindow main_window;
        main_window.show();
        ret = app.exec();
    }
    rclcpp::shutdown();
    return ret;
}