#include "drone_qt_2/airborne_node.hpp"

#include <chrono>
#include <functional>
#include <sstream>
#include <fstream>
#include <filesystem>

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

    auto control_path_ready_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    //创建一个订阅者，订阅控制程序的确认消息，消息类型为自定义消息
    ready_status_sub_ = this->create_subscription<drone_msgs::msg::ReadyStatus>(
        "/control/path_ready", 
        control_path_ready_qos, 
        [this](const drone_msgs::msg::ReadyStatus::SharedPtr msg)//回调
        {
            const bool ready = msg->air_route_ready;

            if (ready) {
                return;
            } else {
                //如果收到ready信号为false，说明控制程序确认失败，可能是因为无人机未进入offboard模式或者其他原因导致无法执行任务，此时需要重置状态以允许重新上传路线和启动任务
                offboard_started_ = false;
                task_started_ = false;
            }
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

    //创建一个服务，提供路线重传接口，服务类型为自定义服务
    stop_push_srv_ = this->create_service<drone_msgs::srv::StartTask>(
        "/drone/stop_push",
        //使用std::bind将成员函数绑定为服务回调函数
        std::bind(
            &AirborneNode::handleStopPush,
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

    upload_mission_yaml_srv_ = this->create_service<drone_msgs::srv::UploadMissionYaml>(
        "/drone/upload_mission_yaml",
        std::bind(
            &AirborneNode::handleUploadMissionYaml,
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
        response->message = "offboard 已经启动，拒绝重复启动";
        return;
    }

    if (!mission_uploaded_) {
        response->success = false;
        response->message = "mission YAML 尚未上传";
        offboard_started_ = false;
        return;
    }

    if (current_mission_path_.empty()) {
        response->success = false;
        response->message = "mission 路径为空";
        offboard_started_ = false;
        return;
    }

    if (!std::filesystem::exists(current_mission_path_)) {
        response->success = false;
        response->message = "mission 文件不存在";
        offboard_started_ = false;
        return;
    }

    const bool ok = startOffboardCommand();
    if (!ok) {
        response->success = false;
        response->message = "offboard 启动失败";
        offboard_started_ = false;
        return;
    }

    current_task_name_ = request->request_source;
    offboard_started_ = true;
    response->success = true;
    response->message = "offboard 启动成功，等待ready信号确认";
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
        response->message = "start 已经启动，拒绝重复启动";
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

void AirborneNode::handleStopPush(
    //使用std::shared_ptr来接收请求和响应对象
    const std::shared_ptr<drone_msgs::srv::StartTask::Request> request,
    std::shared_ptr<drone_msgs::srv::StartTask::Response> response)
{
    (void)request;

    if (task_stoped_) {
        response->success = false;
        response->message = "当前没有启动offboard";
        return;
    }

    std::string error_message;
    const bool ok = stopTaskCommand(error_message);
    if (!ok) {
        response->success = false;
        response->message = "offboard 停止失败";
        task_stoped_ = false;
        return;
    }

    current_task_name_ = request->task_name;

    task_stoped_ = true;
    offboard_started_ = false;
    response->success = true;
    response->message = "停止offboard成功";
}

bool AirborneNode::saveMissionYamlToFile(
    const std::string &yaml_text,
    std::string &saved_path,
    std::string &error_message)
{
    //将接收到的yaml字符串保存到文件中，并返回保存路径和错误信息
    const std::string dir_path = "/home/orangepi/drone_ws/install/drone_control/share/drone_control/config";
    const std::string file_path = dir_path + "/ground_mission.yaml";

    //使用std::filesystem创建目录，如果目录不存在的话
    try {
        std::filesystem::create_directories(dir_path);
    } catch (const std::exception &e) {
        error_message = std::string("创建 mission 目录失败: ") + e.what();
        return false;
    }

    //使用std::ofstream打开文件进行写入，如果文件无法打开则返回错误信息
    std::ofstream out(file_path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        error_message = "无法打开 mission 文件进行写入";
        return false;
    }

    //将yaml文本写入文件，并检查写入是否成功，如果写入失败则返回错误信息
    out << yaml_text;
    out.close();

    if (!out) {
        error_message = "写入 mission 文件失败";
        return false;
    }

    //保存路径并返回成功标志
    saved_path = file_path;
    return true;
}

void AirborneNode::handleUploadMissionYaml(
    const std::shared_ptr<drone_msgs::srv::UploadMissionYaml::Request> request,
    std::shared_ptr<drone_msgs::srv::UploadMissionYaml::Response> response)
{
    if (!request) {
        response->success = false;
        response->message = "请求为空";
        response->saved_path = "";
        return;
    }

    if (request->mission_yaml.empty()) {
        response->success = false;
        response->message = "mission_yaml 为空";
        response->saved_path = "";
        return;
    }

    //将接收到的yaml字符串保存到文件中，并获取保存路径和错误信息
    std::string saved_path;
    std::string error_message;
    const bool ok = saveMissionYamlToFile(request->mission_yaml, saved_path, error_message);

    if (!ok) {
        response->success = false;
        response->message = error_message;
        response->saved_path = "";
        mission_uploaded_ = false;
        current_mission_path_.clear();
        return;
    }

    mission_uploaded_ = true;
    current_mission_path_ = saved_path;

    response->success = true;
    response->message = "mission YAML 保存成功";
    response->saved_path = saved_path;
}

bool AirborneNode::startOffboardCommand()
{
    if (current_mission_path_.empty()) {
        return false;
    }

    if (offboard_process_ && offboard_process_->state() != QProcess::NotRunning) {
        RCLCPP_WARN(this->get_logger(), "offboard process is already running");
        return false;
    }

    if (!offboard_process_) {
        offboard_process_ = new QProcess();

        QObject::connect(offboard_process_, &QProcess::readyReadStandardOutput, [this]() {
            const QByteArray text = offboard_process_->readAllStandardOutput();
            if (!text.isEmpty()) {
                RCLCPP_INFO(this->get_logger(), "[offboard stdout]\n%s", text.constData());
            }
        });

        QObject::connect(offboard_process_, &QProcess::readyReadStandardError, [this]() {
            const QByteArray text = offboard_process_->readAllStandardError();
            if (!text.isEmpty()) {
                RCLCPP_ERROR(this->get_logger(), "[offboard stderr]\n%s", text.constData());
            }
        });

        QObject::connect(
            offboard_process_,
            QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            [this](int exit_code, QProcess::ExitStatus exit_status) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "offboard process finished, exit_code=%d, exit_status=%d",
                    exit_code,
                    static_cast<int>(exit_status));
                offboard_started_ = false;
            });
    }

    const QString command = QString(
        "source /opt/ros/humble/setup.bash && "
        "source ~/drone_ws/install/setup.bash && "
        "ros2 launch drone_bringup run_offboard.launch.py "
        "enable_offboard_control:=true "
        "mission_yaml_path:=%1")
        .arg(QString::fromStdString(current_mission_path_));

    RCLCPP_INFO(this->get_logger(), "starting offboard process with QProcess...");
    offboard_process_->start("bash", QStringList() << "-lc" << command);

    task_stoped_ = false;

    if (!offboard_process_->waitForStarted(3000)) {
        RCLCPP_ERROR(this->get_logger(), "failed to start offboard process");
        return false;
    }

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

bool AirborneNode::stopTaskCommand(std::string &error_message)
{
    if (!offboard_process_) {
        error_message = "offboard 进程对象不存在";
        return false;
    }

    if (offboard_process_->state() == QProcess::NotRunning) {
        offboard_started_ = false;
        error_message = "offboard 当前未运行";
        return false;
    }

    offboard_process_->terminate();
    if (!offboard_process_->waitForFinished(5000)) {
        offboard_process_->kill();
        if (!offboard_process_->waitForFinished(3000)) {
            error_message = "offboard 进程无法停止";
            return false;
        }
    }

    offboard_started_ = false;
    error_message.clear();

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