#include "drone_qt_2/airborne_node.hpp"

#include <chrono>
#include <functional>
#include <sstream>

//使用std::chrono_literals来方便地使用时间单位，如1s表示1秒
using namespace std::chrono_literals;

AirborneNode::AirborneNode()
    : rclcpp::Node("airborne_node")
{
    //设置ROS接口，包括发布者、服务和定时器
    setupInterfaces();
    RCLCPP_INFO(this->get_logger(), "airborne node started");
}

void AirborneNode::setupInterfaces()
{
    auto status_pub_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    //创建一个发布者，发布无人机状态话题，消息类型为自定义消息
    status_pub_ = this->create_publisher<drone_msgs::msg::DroneStatus>(
        "/drone/status", status_pub_qos);

    auto barcode_pub_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    //创建一个发布者，发布条形码捕获话题，消息类型为自定义消息
    barcode_pub_ = this->create_publisher<drone_msgs::msg::BarcodeCapture>(
        "/drone/barcode_capture", barcode_pub_qos);

    auto position_pub_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    //创建一个发布者，发布无人机本地位置话题，消息类型为geometry_msgs::msg::PoseStamped
    local_position_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/drone/local_position", position_pub_qos);

    //创建一个订阅者，订阅视觉系统发布的条形码捕获话题，消息类型为自定义消息
    auto barcode_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    vision_barcode_sub_ = this->create_subscription<drone_msgs::msg::BarcodeCapture>(
        "/drone/image", barcode_qos,
        //使用std::bind将成员函数绑定为订阅回调函数
        std::bind(&AirborneNode::handleBarcodeCapture, this, std::placeholders::_1));

    auto status_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    //创建一个订阅者，订阅无人机状态话题，消息类型为mavros_msgs::msg::State
    status_sub_ = this->create_subscription<mavros_msgs::msg::State>(
        "/mavros/state", status_qos,
        [this](const mavros_msgs::msg::State::SharedPtr msg) {
            if (msg->mode == "MANUAL") {
                flight_mode = drone_msgs::msg::DroneStatus::MODE_MANUAL;
            } else if (msg->mode == "OFFBOARD") {
                flight_mode = drone_msgs::msg::DroneStatus::MODE_OFFBOARD;
            } else if (msg->mode == "STABILIZE") {
                flight_mode = drone_msgs::msg::DroneStatus::MODE_STABILIZE;
            } else if (msg->mode == "AUTO") {
                flight_mode = drone_msgs::msg::DroneStatus::MODE_AUTO;
            } else if (msg->mode == "LOITER") {
                flight_mode = drone_msgs::msg::DroneStatus::MODE_LOITER;
            } else if (msg->mode == "RTL") {
                flight_mode = drone_msgs::msg::DroneStatus::MODE_RTL;
            } else {
                flight_mode = drone_msgs::msg::DroneStatus::MODE_UNKNOWN;
            }
            connected = msg->connected ? true : false;
            armed = msg->armed;
        });

    auto capture_qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();

    //创建一个订阅者，订阅无人机电量，消息类型为sensor_msgs::BatteryState
    battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
        "/mavros/battery", capture_qos,
        [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
             if (msg->voltage <= 0.0f) {
                return;
            }

            if (msg->percentage < 0.0f) {
                return;
            }
            battery_voltage = msg->voltage;
            battery_percent = msg->percentage;
        });

    auto position_qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
    //创建一个订阅者，订阅无人机本地位置，消息类型为geometry_msgs::PoseStamped
    local_position_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/mavros/local_position/pose", position_qos,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            local_position_pub_->publish(*msg);
        });

    //创建一个服务，提供任务启动接口，服务类型为自定义服务
    start_task_srv_ = this->create_service<drone_msgs::srv::StartTask>(
        "/drone/start_task",
        //使用std::bind将成员函数绑定为服务回调函数
        std::bind(
            &AirborneNode::handleStartTask,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    start_offboard_srv_ = this->create_service<drone_msgs::srv::StartOffboard>(
        "/drone/start_offboard",
        std::bind(
            &AirborneNode::handleStartOffboard,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    timer_ = this->create_wall_timer(
        0.2s,
        std::bind(&AirborneNode::onTimer, this));
}

void AirborneNode::onTimer()
{
    // if (task_running_) {
    //     task_progress_ += 20;

    //     if (task_progress_ >= 100) {
    //         task_progress_ = 100;
    //         task_running_ = false;
    //         current_task_name_.clear();
    //         action_name_.clear();
    //         RCLCPP_INFO(this->get_logger(), "task finished");
    //     }
    // }

    publishStatus();
}

void AirborneNode::publishStatus()
{
    //构建一个状态消息，并发布到话题上
    drone_msgs::msg::DroneStatus status_msg;
    status_msg.connected = connected;
    status_msg.armed = armed;
    status_msg.flight_mode = flight_mode;
    status_msg.battery_voltage = battery_voltage;
    status_msg.battery_percent = battery_percent;
    status_pub_->publish(status_msg);
}

void AirborneNode::handleBarcodeCapture(
    const drone_msgs::msg::BarcodeCapture::SharedPtr msg)
{
    //接收到视觉系统发布的条形码捕获消息后，直接转发到自己的条形码捕获话题上
    barcode_pub_->publish(*msg);
}

void AirborneNode::handleStartOffboard(
    const std::shared_ptr<drone_msgs::srv::StartOffboard::Request> request,
    std::shared_ptr<drone_msgs::srv::StartOffboard::Response> response)
{
    (void)request;

    if (offboard_started_) {
        response->success = false;
        response->message = "/drone/start_offboard服务未启动";
        return;
    }

    const bool ok = startOffboardCommand();
    if (!ok) {
        response->success = false;
        response->message = "offboard启动失败";
        return;
    }

    current_task_name_ = request->request_source;

    offboard_started_ = true;
    response->success = true;
    response->message = "offboard启动成功";
}

void AirborneNode::handleStartTask(
    //使用std::shared_ptr来接收请求和响应对象
    const std::shared_ptr<drone_msgs::srv::StartTask::Request> request,
    std::shared_ptr<drone_msgs::srv::StartTask::Response> response)
{
    (void)request;

    //如果已经有任务在运行，则拒绝新的任务请求，并在日志中记录警告信息
    if (task_running_) {
        response->success = false;
        response->message = "有任务正在运行：" + current_task_name_;
        //RCLCPP_WARN(this->get_logger(), "reject start_task: task already running");
        return;
    }

    if (task_started_) {
        response->success = false;
        response->message = "/drone/start_task服务未启动";
        return;
    }

    const bool ok = startTaskCommand();
    if (!ok) {
        response->success = false;
        response->message = "task启动失败";
        return;
    }

    current_task_name_ = request->task_name;

    task_started_ = true;
    response->success = true;
    response->message = "成功起飞";
}

bool AirborneNode::startOffboardCommand()
{
    const std::string command =
        "bash -lc 'source /opt/ros/humble/setup.bash && "
        "source ~/drone_ws/install/setup.bash && "
        "ros2 launch drone_bringup run_offboard.launch.py enable_offboard_control:=true'";

    std::thread([this, command]() {
        RCLCPP_INFO(this->get_logger(), "starting offboard command...");
        const int ret = std::system(command.c_str());
        RCLCPP_INFO(this->get_logger(), "offboard command exited with code=%d", ret);
        offboard_started_ = false;
    }).detach();

    return true;
}

bool AirborneNode::startTaskCommand()
{
    const std::string command =
        "bash -lc 'source /opt/ros/humble/setup.bash && "
        "source ~/drone_ws/install/setup.bash && "
        "ros2 topic pub --once /start_mission std_msgs/msg/Empty \"{}\"'";

    std::thread([this, command]() {
        RCLCPP_INFO(this->get_logger(), "starting takeoff program command...");
        const int ret = std::system(command.c_str());
        RCLCPP_INFO(this->get_logger(), "takeoff program command exited with code=%d", ret);
        task_started_ = false;
    }).detach();

    return true;
}



// std::string AirborneNode::buildStatusText() const
// {
//     //包含当前状态、任务运行状态和进度信息
//     std::ostringstream oss;
//     oss << "task_running=" << (task_running_ ? "true" : "false")
//         << ";progress=" << task_progress_;
//     return oss.str();
// }