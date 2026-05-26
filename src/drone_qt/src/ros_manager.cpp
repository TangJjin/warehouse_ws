#include "drone_qt/ros_manager.hpp"

RosManager::RosManager(QObject *parent)
    : QObject(parent),
    node_(std::make_shared<rclcpp::Node>("ground_qt_node"))
{
    setupRosInterfaces();
}

RosManager::~RosManager()
{
    //取消执行器的 spinning，并等待线程结束
    executor_.cancel();

    //如果线程可连接，则连接线程
    if (spin_thread_.joinable())
    {
        spin_thread_.join();
    }
}

void RosManager::setupRosInterfaces()
{
    //保证投递
    auto group_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    //创建一个发布者，发布预规划路线消息，消息类型为自定义消息
    path_pub_ = node_->create_publisher<drone_msgs::msg::WorldGroup>(
        "/drone/world_group",
        group_qos
    );

    auto status_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    //创建一个订阅者，订阅无人机状态话题，消息类型为自定义消息
    status_sub_ = node_->create_subscription<drone_msgs::msg::DroneStatus>(
        "/drone/status", 
        status_qos, 
        [this](const drone_msgs::msg::DroneStatus::SharedPtr msg)//回调
        {
            connected = msg->connected;
            battery_percent = msg->battery_percent;
            flight_mode = static_cast<int>(msg->flight_mode);
            armed = msg->armed;
            const QString task_name = QString::fromStdString(msg->state);

            //使用Qt的信号槽机制在线程安全的方式下发状态更新信号，包含连接状态、电量百分比、飞行模式和解锁状态等信息
            QMetaObject::invokeMethod(
                this,
                [this, task_name]() {
                    emit statusUpdated(
                        connected,
                        battery_percent,
                        flight_mode,
                        armed,
                        task_name);
                },
                Qt::QueuedConnection);
        });

    auto control_status_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    //创建一个订阅者，订阅控制程序状态话题，消息类型为自定义消息
    task_status_sub_ = node_->create_subscription<drone_msgs::msg::TaskStatus>(
        "/task/status", 
        control_status_qos, 
        [this](const drone_msgs::msg::TaskStatus::SharedPtr msg)//回调
        {
            const bool task_running = msg->task_running;
            const int action_step = msg->action_step;
            const int action_num = msg->action_num;
            const QString action_name = QString::fromStdString(msg->action_name);

            //使用Qt的信号槽机制在线程安全的方式下发状态更新信号，包含连接状态、电量百分比、飞行模式和解锁状态等信息
            QMetaObject::invokeMethod(
                this,
                [this, task_running, action_step, action_num, action_name]() {
                    emit action_statusUpdated(
                        task_running,
                        action_step,
                        action_num,
                        action_name);
                },
                Qt::QueuedConnection);
        });

    auto control_path_ready_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    //创建一个订阅者，订阅控制程序的确认消息，消息类型为自定义消息
    ready_status_sub_ = node_->create_subscription<drone_msgs::msg::ReadyStatus>(
        "/control/path_ready", 
        control_path_ready_qos, 
        [this](const drone_msgs::msg::ReadyStatus::SharedPtr msg)//回调
        {
            const bool ready = msg->air_route_ready;

            QMetaObject::invokeMethod(
                this,
                [this, ready]() {
                    emit pathReadyChanged(ready);
                },
                Qt::QueuedConnection);
        });

    auto barcode_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    //创建一个订阅者，订阅条形码捕获话题，消息类型为自定义消息
    barcode_sub_ = node_->create_subscription<drone_msgs::msg::BarcodeCapture>(
        "drone/barcode_capture",
        barcode_qos,
        [this](const drone_msgs::msg::BarcodeCapture::SharedPtr msg)
        {
            //将接收到的条形码捕获消息中的图像数据转换为QByteArray格式，以便在Qt界面中使用
            const QByteArray image_bytes(
                reinterpret_cast<const char*>(msg->image_data.data()),
                static_cast<int>(msg->image_data.size()));

            //使用Qt的信号槽机制在线程安全的方式下发条形码捕获信号，包含捕获到的条形码数据、图像数据、图像格式和时间文本等信息
            const QString barcode = QString::fromStdString(msg->barcode);
            const QString image_format = QString::fromStdString(msg->image_format);
            const qint64 time_ms =
                static_cast<qint64>(msg->stamp.sec) * 1000LL +
                static_cast<qint64>(msg->stamp.nanosec) / 1000000LL;
            const QString time_text = 
                QDateTime::fromMSecsSinceEpoch(time_ms,Qt::LocalTime)
                .toString("yyyy-MM-dd hh:mm:ss");

            //使用QMetaObject::invokeMethod来在线程安全的方式下发信号，确保信号在Qt的事件循环中被正确处理
            QMetaObject::invokeMethod(
                this,
                [this, barcode, image_bytes, image_format, time_text]() {
                    emit barcodeCaptured(barcode, image_bytes, image_format, time_text);
                },
                Qt::QueuedConnection);
        });

    auto local_position_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    //创建一个订阅者，订阅无人机本地位置话题，消息类型为geometry_msgs::msg::PoseStamped
    local_position_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "drone/local_position",
        local_position_qos,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            const double x = msg->pose.position.x;
            const double y = msg->pose.position.y;
            const double z = msg->pose.position.z;

            //使用Qt的信号槽机制在线程安全的方式下发位置更新信号，包含无人机的二维位置坐标和高度等信息
            QMetaObject::invokeMethod(
                this,
                [this, x, y, z]() {
                    emit positionUpdated(x, y, z);
                },
                Qt::QueuedConnection);
        });

auto delta_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    delta_sub_ = node_->create_subscription<geometry_msgs::msg::Vector3>(
        "/pose_yaw_compare/delta",
        delta_qos,
        [this](const geometry_msgs::msg::Vector3::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(delta_mutex_);
            latest_dx_ = msg->x;
            latest_dy_ = msg->y;
            latest_dyaw_ = msg->z;
        });

    //创建一个服务客户端，用于调用任务启动服务
    start_task_client_ = node_->create_client<drone_msgs::srv::StartTask>(
        "/drone/start_task");

    //创建一个服务客户端，用于调用路线重传服务
    stop_push_client_ = node_->create_client<drone_msgs::srv::StartTask>(
        "/drone/stop_push");

    //创建一个服务客户端，用于调用offboard启动服务
    start_offboard_client_ = node_->create_client<drone_msgs::srv::StartOffboard>(
    "/drone/start_offboard");

    //创建一个服务客户端，用于调用任务yaml上传服务
    upload_mission_yaml_client_ = node_->create_client<drone_msgs::srv::UploadMissionYaml>(
    "/drone/upload_mission_yaml");

    using namespace std::chrono_literals;
    timer_ = node_->create_wall_timer(
        0.5s,
        [this]()
        {
            double dx, dy, dyaw;
            {
                std::lock_guard<std::mutex> lock(delta_mutex_);
                dx = latest_dx_;
                dy = latest_dy_;
                dyaw = latest_dyaw_;
            }

            QMetaObject::invokeMethod(
                this,
                [this, dx, dy, dyaw]() {
                    emit deltaUpdated(dx, dy, dyaw);
                },
                Qt::QueuedConnection);
        });
}

void RosManager::start()
{
    //如果已经启动，则直接返回
    if (started_)
    {
        return;
    }

    //将ROS节点添加到执行器中，并启动一个线程来运行ROS spinning
    executor_.add_node(node_);
    spin_thread_ = std::thread([this]()
    {
        executor_.spin();
    });

    started_ = true;
}

void RosManager::startTask()
{
    if(!start_task_client_->service_is_ready())
    {
        //emit commandResult(false, "service /drone/start_task is not ready");
        return;
    }
    
    //创建一个服务请求对象，并设置任务名称字段
    auto request = std::make_shared<drone_msgs::srv::StartTask::Request>();
    request->task_name = "action_task_run";//开始执行任务

    //异步调用服务，并在收到响应时发出命令结果信号
    start_task_client_->async_send_request(
        request,
        [this](rclcpp::Client<drone_msgs::srv::StartTask>::SharedFuture future)
        {
            //获取服务响应，并发出命令结果信号，包含成功标志和响应消息
            const auto response = future.get();
            emit commandResult(
                response->success,
                QString::fromStdString(response->message));
        });
}

void RosManager::stopTask()
{
    if(!stop_push_client_->service_is_ready())
    {
        //emit commandResult(false, "service /drone/start_task is not ready");
        return;
    }

    //创建一个服务请求对象，并设置任务名称字段
    auto request = std::make_shared<drone_msgs::srv::StartTask::Request>();
    request->task_name = "stop_task";//停止任务

    //异步调用服务，并在收到响应时发出命令结果信号
    stop_push_client_->async_send_request(
        request,
        [this](rclcpp::Client<drone_msgs::srv::StartTask>::SharedFuture future)
        {
            //获取服务响应，并发出命令结果信号，包含成功标志和响应消息
            const auto response = future.get();
            emit stopcommandResult(
                response->success,
                QString::fromStdString(response->message));
        });
}

void RosManager::requestStartOffboard()
{
    if (!start_offboard_client_ || !start_offboard_client_->service_is_ready()) {
        //emit offboardCommandResult(false, "服务 /drone/start_offboard 未加载");
        return;
    }

    auto request = std::make_shared<drone_msgs::srv::StartOffboard::Request>();
    request->request_source = "ground_station_upload_flow";

    start_offboard_client_->async_send_request(
        request,
        [this](rclcpp::Client<drone_msgs::srv::StartOffboard>::SharedFuture future)
        {
            const auto response = future.get();
            emit offboardCommandResult(
                response->success,
                QString::fromStdString(response->message));
        });
}

void RosManager::uploadMissionYaml(const QString &mission_yaml)
{
    if (!upload_mission_yaml_client_ || !upload_mission_yaml_client_->service_is_ready()) {
        emit missionUploadFinished(false, "服务 /drone/upload_mission_yaml 未就绪", "");
        return;
    }

    auto request = std::make_shared<drone_msgs::srv::UploadMissionYaml::Request>();
    //将传入的QString类型的mission_yaml转换为std::string类型，并赋值给服务请求对象的mission_yaml字段
    request->mission_yaml = mission_yaml.toStdString();

    upload_mission_yaml_client_->async_send_request(
        request,
        [this](rclcpp::Client<drone_msgs::srv::UploadMissionYaml>::SharedFuture future)
        {
            const auto response = future.get();
            QMetaObject::invokeMethod(
                this,
                [this, response]() {
                    emit missionUploadFinished(
                        response->success,
                        QString::fromStdString(response->message),
                        QString::fromStdString(response->saved_path));
                },
                Qt::QueuedConnection);
        });
}

// void RosManager::publishPath(const QVector<QPoint> &path_points)
// {
//     if (!path_pub_) {
//         return;
//     }

//     //构建一个GridPath消息，并将路径点列表中的坐标点转换为消息中的行列格式，然后发布消息
//     drone_msgs::msg::GridGroup msg;

//     //遍历路径点列表，将每个坐标点的x和y值分别添加到消息的行和列字段中
//     for (const auto &point : path_points) {
//         drone_msgs::msg::GridPoint grid_point;
//         grid_point.rows = point.x();
//         grid_point.cols = point.y();
//         msg.points.push_back(grid_point);
//     }

//     path_pub_->publish(msg);
// }

void RosManager::publishPath(const QVector<WorldCoord> &World_points)
{
    if (!path_pub_) {
        return;
    }

    //构建一个GridPath消息，并将路径点列表中的坐标点转换为消息中的行列格式，然后发布消息
    drone_msgs::msg::WorldGroup msg;

    //遍历路径点列表，将每个坐标点的x和y值分别添加到消息的行和列字段中
    for (const auto &point : World_points) {
        drone_msgs::msg::WorldPoint World_point;
        World_point.x = point.x;
        World_point.y = point.y;
        msg.points.push_back(World_point);
    }

    path_pub_->publish(msg);
    push_flag = true;
    emit pushFlagChanged(push_flag);
}
