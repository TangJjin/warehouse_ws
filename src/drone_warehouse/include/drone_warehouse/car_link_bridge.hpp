#pragma once

#include <cstdint>
#include <string>

#include <QByteArray>
#include <QSerialPort>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

class CarLinkBridge : public rclcpp::Node
{
public:
    CarLinkBridge();
    ~CarLinkBridge() override = default;

private:
    void setupRosInterfaces();
    void setupSerial();
    void onSerialReadyRead();

    bool tryParseCarFrame(QByteArray &payload);
    bool validateFrame(const QByteArray &frame) const;
    void publishCarLocalPosition(const QByteArray &payload);

    std::string serial_port_;
    int baud_rate_{115200};

    QSerialPort serial_;
    QByteArray rx_buffer_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
        car_local_position_pub_;
};