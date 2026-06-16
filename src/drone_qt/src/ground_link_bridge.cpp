#include "drone_qt/ground_link_bridge.hpp"

#include <QDataStream>

namespace lp = drone_msgs::link_protocol;

namespace
{
    uint16_t crc16_ccitt(const uint8_t *data, int length)
    {
        uint16_t crc = 0xFFFF;
        for (int i = 0; i < length; ++i) {
            crc ^= static_cast<uint16_t>(data[i]) << 8;
            for (int j = 0; j < 8; ++j) {
                if (crc & 0x8000) {
                    crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
                } else {
                    crc = static_cast<uint16_t>(crc << 1);
                }
            }
        }
        return crc;
    }
}

GroundLinkBridge::GroundLinkBridge()
    : QObject(),
      rclcpp::Node("ground_link_bridge")
{
    setupRosInterfaces();
    setupSerial();
}

void GroundLinkBridge::setupRosInterfaces()
{
    /*************** 创建与机载端相对应的通信接口 ***************/
    status_pub_ = this->create_publisher<drone_msgs::msg::DroneStatus>(
        "/serial/drone/status", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());

    task_status_pub_ = this->create_publisher<drone_msgs::msg::TaskStatus>(
        "/serial/drone/task/status", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    path_ready_pub_ = this->create_publisher<drone_msgs::msg::ReadyStatus>(
        "/serial/drone/control/path_ready", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    return_world_group_pub_ = this->create_publisher<drone_msgs::msg::WorldGroup>(
        "/serial/drone/return/world_group", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    vision_barcode_pub_ = this->create_publisher<drone_msgs::msg::BarcodeCapture>(
        "/serial/drone/vision/barcode", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    delta_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
        "/serial/drone/pose_yaw_compare/delta", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());

    upload_mission_summary_srv_ = this->create_service<drone_msgs::srv::UploadMissionSummary>(
        "/serial/drone/upload_mission_summary",
        [this](
        const std::shared_ptr<drone_msgs::srv::UploadMissionSummary::Request> request,
        std::shared_ptr<drone_msgs::srv::UploadMissionSummary::Response> response)
        {
            if (!request || request->points.empty()) {
                response->success = false;
                response->message = "路线点为空";
                response->saved_path = "";
                response->action_count = 0;
                return;
            }

            const QByteArray payload = encodeUploadMissionSummaryRequest(*request, next_msg_id_++);
            const uint16_t seq = sendPacket(lp::kTypeUploadMissionSummaryReq, lp::kFlagNeedAck, payload, true);

            pending_upload_calls_[seq] = PendingUploadMissionSummaryCall{};

            const auto deadline = this->now() + rclcpp::Duration::from_seconds(8.0);
            while (rclcpp::ok()) {
                if (serial_.waitForReadyRead(20)) {
                    onSerialReadyRead();
                }
                onRetryTimer();

                auto it = pending_upload_calls_.find(seq);
                if (it != pending_upload_calls_.end() && it->second.done) {
                    response->success = it->second.success;
                    response->message = it->second.message;
                    response->saved_path = it->second.saved_path;
                    response->action_count = it->second.action_count;
                    pending_upload_calls_.erase(it);
                    return;
                }

                if (this->now() > deadline) {
                    response->success = false;
                    response->message = "等待 UploadMissionSummaryResp 超时";
                    response->saved_path = "";
                    response->action_count = 0;
                    pending_upload_calls_.erase(seq);
                    return;
                }
            }

            response->success = false;
            response->message = "节点停止，UploadMissionSummary 未完成";
            response->saved_path = "";
            response->action_count = 0;
            pending_upload_calls_.erase(seq);
        });

    start_offboard_srv_ = this->create_service<drone_msgs::srv::StartOffboard>(
        "/serial/drone/start_offboard",
        [this](
        const std::shared_ptr<drone_msgs::srv::StartOffboard::Request> request,
        std::shared_ptr<drone_msgs::srv::StartOffboard::Response> response)
        {
            if (!request) {
                response->success = false;
                response->message = "空请求";
                return;
            }

            const QByteArray payload = encodeStartOffboardRequest(*request);
            const uint16_t seq = sendPacket(lp::kTypeStartOffboardReq, lp::kFlagNeedAck, payload, true);

            pending_start_offboard_calls_[seq] = PendingStartOffboardCall{};

            const auto deadline = this->now() + rclcpp::Duration::from_seconds(5.0);
            while (rclcpp::ok()) {
                if (serial_.waitForReadyRead(20)) {
                    onSerialReadyRead();
                }
                onRetryTimer();

                auto it = pending_start_offboard_calls_.find(seq);
                if (it != pending_start_offboard_calls_.end() && it->second.done) {
                    response->success = it->second.success;
                    response->message = it->second.message;
                    pending_start_offboard_calls_.erase(it);
                    return;
                }

                if (this->now() > deadline) {
                    response->success = false;
                    response->message = "等待 StartOffboardResp 超时";
                    pending_start_offboard_calls_.erase(seq);
                    return;
                }
            }

            response->success = false;
            response->message = "节点停止，StartOffboard 未完成";
            pending_start_offboard_calls_.erase(seq);
        });

    start_task_srv_ = this->create_service<drone_msgs::srv::StartTask>(
        "/serial/drone/start_task",
        [this](
        const std::shared_ptr<drone_msgs::srv::StartTask::Request> request,
        std::shared_ptr<drone_msgs::srv::StartTask::Response> response)
        {
            if (!request) {
                response->success = false;
                response->message = "空请求";
                return;
            }

            const QByteArray payload = encodeStartTaskRequest(*request);
            const uint16_t seq = sendPacket(lp::kTypeStartTaskReq, lp::kFlagNeedAck, payload, true);

            pending_start_task_calls_[seq] = PendingStartTaskCall{};

            const auto deadline = this->now() + rclcpp::Duration::from_seconds(5.0);
            while (rclcpp::ok()) {
                if (serial_.waitForReadyRead(20)) {
                    onSerialReadyRead();
                }
                onRetryTimer();

                auto it = pending_start_task_calls_.find(seq);
                if (it != pending_start_task_calls_.end() && it->second.done) {
                    response->success = it->second.success;
                    response->message = it->second.message;
                    pending_start_task_calls_.erase(it);
                    return;
                }

                if (this->now() > deadline) {
                    response->success = false;
                    response->message = "等待 StartTaskResp 超时";
                    pending_start_task_calls_.erase(seq);
                    return;
                }
            }

            response->success = false;
            response->message = "节点停止，StartTask 未完成";
            pending_start_task_calls_.erase(seq);
        });

    stop_push_srv_ = this->create_service<drone_msgs::srv::StartTask>(
        "/serial/drone/stop_push",
        [this](
        const std::shared_ptr<drone_msgs::srv::StartTask::Request> request,
        std::shared_ptr<drone_msgs::srv::StartTask::Response> response)
        {
            if (!request) {
                response->success = false;
                response->message = "空请求";
                return;
            }

            const QByteArray payload = encodeStopPushRequest(*request);
            const uint16_t seq = sendPacket(lp::kTypeStopPushReq, lp::kFlagNeedAck, payload, true);

            pending_stop_push_calls_[seq] = PendingStopPushCall{};

            const auto deadline = this->now() + rclcpp::Duration::from_seconds(5.0);
            while (rclcpp::ok()) {
                if (serial_.waitForReadyRead(20)) {
                    onSerialReadyRead();
                }
                onRetryTimer();

                auto it = pending_stop_push_calls_.find(seq);
                if (it != pending_stop_push_calls_.end() && it->second.done) {
                    response->success = it->second.success;
                    response->message = it->second.message;
                    pending_stop_push_calls_.erase(it);
                    return;
                }

                if (this->now() > deadline) {
                    response->success = false;
                    response->message = "等待 StopPushResp 超时";
                    pending_stop_push_calls_.erase(seq);
                    return;
                }
            }

            response->success = false;
            response->message = "节点停止，StopPush 未完成";
            pending_stop_push_calls_.erase(seq);
        });
    /************************************************************/
}


/************************ 用户层 *************************/
void GroundLinkBridge::setupSerial()
{
    serial_.setPortName("/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0");
    serial_.setBaudRate(QSerialPort::Baud115200);
    serial_.setDataBits(QSerialPort::Data8);
    serial_.setParity(QSerialPort::NoParity);
    serial_.setStopBits(QSerialPort::OneStop);
    serial_.setFlowControl(QSerialPort::NoFlowControl);

    if (!serial_.open(QIODevice::ReadWrite)) {
        RCLCPP_ERROR(this->get_logger(), "failed to open serial port");
    }
}

void GroundLinkBridge::onSerialReadyRead()
{
    rx_buffer_.append(serial_.readAll());

    //接收到数据后，进入第一层handlePacket处理函数，后续分路
    Packet packet;
    while (tryParseOnePacket(packet)) {
        handlePacket(packet);
    }
}

uint16_t GroundLinkBridge::sendPacket(uint8_t type, uint8_t flags, const QByteArray &payload, bool need_ack)
{
    const uint16_t seq = nextSeq();
    //将消息类型、标志位、序列号和载荷数据编码成一个完整的帧数据，准备发送
    const QByteArray frame = encodeFrame(type, flags, seq, payload);
    //发送协议帧到串口
    serial_.write(frame);

    //如果需要ACK，则将请求信息保存到待重试列表中，以便后续检查和重试
    if (need_ack) {
        PendingRequest pending;
        pending.request_type = type;
        pending.seq = seq;
        pending.retry_count = 0;
        pending.encoded_frame = frame;
        pending.deadline = this->now() + rclcpp::Duration::from_seconds(0.3);
        pending_requests_[seq] = pending;
    }
    return seq;
}

void GroundLinkBridge::handlePacket(const Packet &packet)
{
    //如果收到的是ACK帧，则调用handleAck函数处理，并返回
    if (packet.type == lp::kTypeAck) {
        handleAck(packet.seq);
        return;
    }

    //如果收到的帧需要ACK，则先发送ACK帧，通知对方已经收到消息，避免对方继续重试
    if (packet.flags & lp::kFlagNeedAck) {
        sendAck(packet.seq);
    }

    //根据帧的类型调用不同的处理逻辑，并发送相应的ACK或响应数据
    switch (packet.type) {
    case lp::kTypeDroneStatus:
        handleDroneStatusReport(packet.payload);
        break;
    case lp::kTypePathReady:
        handlePathReadyReport(packet.payload);
        break;
    case lp::kTypeTaskStatus:
        handleTaskStatusReport(packet.payload);
        break;
    case lp::kTypeReturnWorldGroup:
        handleReturnWorldGroupReport(packet.payload);
        break;
    case lp::kTypeVisionBarcode:
        handleVisionBarcodeReport(packet.payload);
        break;
    case lp::kTypeDelta:
        handleDeltaReport(packet.payload);
        break;
    case lp::kTypeUploadMissionSummaryResp:
        handleUploadMissionSummaryResponse(packet.seq, packet.payload);
        break;
    case lp::kTypeStartOffboardResp:
        handleStartOffboardResponse(packet.seq, packet.payload);
        break;
    case lp::kTypeStartTaskResp:
        handleStartTaskResponse(packet.seq, packet.payload);
        break;
    case lp::kTypeStopPushResp:
        handleStopPushResponse(packet.seq, packet.payload);
        break;
    default:
        RCLCPP_WARN(this->get_logger(), "unknown packet type: 0x%02X", packet.type);
        break;
    }
}

void GroundLinkBridge::sendAck(uint16_t seq)
{
    //发送ACK帧到串口，通知对方之前发送的消息已经收到，无需再重试
    const QByteArray empty_payload;
    const QByteArray frame = encodeFrame(lp::kTypeAck, lp::kFlagAck, seq, empty_payload);
    serial_.write(frame);
}

void GroundLinkBridge::handleAck(uint16_t seq)
{
    //收到ACK帧后，从待重试列表中删除对应的请求信息，表示请求已经成功完成，不需要再重试
    pending_requests_.erase(seq);
}

uint16_t GroundLinkBridge::nextSeq()
{
    //生成一个新的序列号，确保每个请求都有一个唯一的标识符，便于匹配响应和处理重试逻辑
    return next_seq_++;
}






/******************* 数据处理层(发送层) ********************/

QByteArray GroundLinkBridge::encodeUploadMissionSummaryRequest(
    const drone_msgs::srv::UploadMissionSummary::Request &request,
    uint16_t msg_id) const
{
    Q_UNUSED(msg_id);

    QByteArray payload;//创建一块空字节缓冲区
    QDataStream stream(&payload, QIODevice::WriteOnly);//创建一个数据流，让你可以方便地把整数、浮点数按二进制写到 payload
    stream.setByteOrder(QDataStream::LittleEndian);//规定多字节数据按小端写入

    const auto &points = request.points;
    stream << static_cast<quint16>(points.size());//先写点的数量
    for (const auto &point : points) {
        stream << static_cast<float>(point.x);
        stream << static_cast<float>(point.y);
    }

    const auto &summary = request.summary;
    stream << static_cast<double>(summary.takeoff_altitude);
    stream << static_cast<double>(summary.move_altitude);
    stream << static_cast<double>(summary.start_altitude);
    stream << static_cast<double>(summary.yaw);
    stream << static_cast<double>(summary.tolerance);
    stream << static_cast<double>(summary.takeoff_hover_duration);
    stream << static_cast<double>(summary.landing_hover_duration);
    stream << static_cast<double>(summary.move_hover_duration);

    stream << static_cast<quint8>(summary.add_hover_between_takeoff ? 1 : 0);
    stream << static_cast<quint8>(summary.add_hover_between_landing ? 1 : 0);
    stream << static_cast<quint8>(summary.add_hover_between_moves ? 1 : 0);
    stream << static_cast<quint8>(summary.use_camera_aim ? 1 : 0);
    stream << static_cast<quint8>(summary.auto_start_mission ? 1 : 0);
    stream << static_cast<quint8>(summary.compress_straight_segments ? 1 : 0);

    const QByteArray frame = QByteArray::fromStdString(summary.frame);
    stream << static_cast<quint16>(frame.size());
    payload.append(frame);

    return payload;
}

QByteArray GroundLinkBridge::encodeStartOffboardRequest(
    const drone_msgs::srv::StartOffboard::Request &request) const
{
    //编码函数，用于将ROS服务请求对象编码成协议帧的载荷格式，准备发送给机载端
    QByteArray payload;//新建一个空的字节数组
    QDataStream stream(&payload, QIODevice::WriteOnly);//写进payload
    stream.setByteOrder(QDataStream::LittleEndian);

    //取出请求里的字符串字段
    const QByteArray source = QByteArray::fromStdString(request.request_source);
    stream << static_cast<quint16>(source.size());//写入字符长度
    payload.append(source);//写入本体内容
    //输出：[字符串长度][字符串原始字节]
    return payload;
}

QByteArray GroundLinkBridge::encodeStartTaskRequest(
    const drone_msgs::srv::StartTask::Request &request) const
{
    //编码函数，用于将ROS服务请求对象编码成协议帧的载荷格式，准备发送给机载端
    QByteArray payload;//新建一个空的字节数组
    QDataStream stream(&payload, QIODevice::WriteOnly);//写进payload
    stream.setByteOrder(QDataStream::LittleEndian);

    //取出请求里的字符串字段
    const QByteArray task_name = QByteArray::fromStdString(request.task_name);
    stream << static_cast<quint16>(task_name.size());//写入字符长度
    payload.append(task_name);//写入本体内容
    //输出：[字符串长度][字符串原始字节]
    return payload;
}

QByteArray GroundLinkBridge::encodeStopPushRequest(
    const drone_msgs::srv::StartTask::Request &request) const
{
    //编码函数，用于将ROS服务请求对象编码成协议帧的载荷格式，准备发送给机载端
    QByteArray payload;//新建一个空的字节数组
    QDataStream stream(&payload, QIODevice::WriteOnly);//写进payload
    stream.setByteOrder(QDataStream::LittleEndian);

    //取出请求里的字符串字段
    const QByteArray task_name = QByteArray::fromStdString(request.task_name);
    stream << static_cast<quint16>(task_name.size());//写入字符长度
    payload.append(task_name);//写入本体内容
    //输出：[字符串长度][字符串原始字节]
    return payload;
}








/******************* 数据处理层(接收层) ********************/

void GroundLinkBridge::handleDroneStatusReport(const QByteArray &payload)
{
    QDataStream stream(payload);
    stream.setByteOrder(QDataStream::LittleEndian);

    quint8 connected = 0;
    float battery_percent = 0.0f;
    quint8 flight_mode = 0;
    quint8 armed = 0;
    quint16 state_len = 0;

    stream >> connected;
    stream >> battery_percent;
    stream >> flight_mode;
    stream >> armed;
    stream >> state_len;

    if (payload.size() < 1 + 4 + 1 + 1 + 2 + state_len) {
        RCLCPP_ERROR(this->get_logger(), "invalid DroneStatus payload");
        return;
    }

    const QString state = QString::fromUtf8(payload.mid(10, state_len));

    drone_msgs::msg::DroneStatus msg;
    msg.connected = (connected != 0);
    msg.battery_percent = battery_percent;
    msg.flight_mode = static_cast<uint8_t>(flight_mode);
    msg.armed = (armed != 0);
    msg.state = state.toStdString();

    status_pub_->publish(msg);
}

void GroundLinkBridge::handleTaskStatusReport(const QByteArray &payload)
{
    QDataStream stream(payload);
    stream.setByteOrder(QDataStream::LittleEndian);

    quint8 task_running = 0;
    quint16 action_name_len = 0;
    qint32 action_step = 0;
    quint8 action_num = 0;

    stream >> task_running;
    stream >> action_name_len;

    if (payload.size() < 1 + 2 + action_name_len + 4 + 1) {
        RCLCPP_ERROR(this->get_logger(), "invalid TaskStatus payload");
        return;
    }
    const QString action_name = QString::fromUtf8(payload.mid(3, action_name_len));
    const QByteArray tail = payload.mid(3 + action_name_len);
    QDataStream tail_stream(tail);
    tail_stream.setByteOrder(QDataStream::LittleEndian);

    tail_stream >> action_step;
    tail_stream >> action_num;

    drone_msgs::msg::TaskStatus msg;
    msg.task_running = (task_running != 0);
    msg.action_name = action_name.toStdString();
    msg.action_step = action_step;
    msg.action_num = action_num;

    task_status_pub_->publish(msg);
}

void GroundLinkBridge::handlePathReadyReport(const QByteArray &payload)
{
    if (payload.size() < 1) {
        RCLCPP_ERROR(this->get_logger(), "invalid PathReady payload");
        return;
    }

    drone_msgs::msg::ReadyStatus msg;
    msg.air_route_ready = (static_cast<uint8_t>(payload[0]) != 0);
    path_ready_pub_->publish(msg);
}

void GroundLinkBridge::handleReturnWorldGroupReport(const QByteArray &payload)
{
    QDataStream stream(payload);
    stream.setByteOrder(QDataStream::LittleEndian);

    quint16 point_count = 0;
    stream >> point_count;

    if (payload.size() < 2 + static_cast<int>(point_count) * 4 * 2) {
        RCLCPP_ERROR(this->get_logger(), "invalid ReturnWorldGroup payload");
        return;
    }

    drone_msgs::msg::WorldGroup msg;
    msg.points.reserve(point_count);

    for (quint16 i = 0; i < point_count; ++i) {
        float x = 0.0f;
        float y = 0.0f;
        stream >> x;
        stream >> y;

        drone_msgs::msg::WorldPoint point;
        point.x = x;
        point.y = y;
        msg.points.push_back(point);
    }

    return_world_group_pub_->publish(msg);
}

void GroundLinkBridge::handleVisionBarcodeReport(const QByteArray &payload)
{
    QDataStream stream(payload);
    stream.setByteOrder(QDataStream::LittleEndian);

    qint32 sec = 0;
    quint32 nanosec = 0;
    quint16 barcode_len = 0;
    quint16 image_format_len = 0;

    stream >> sec;
    stream >> nanosec;
    stream >> barcode_len;

    if (payload.size() < 4 + 4 + 2 + barcode_len + 2) {
        RCLCPP_ERROR(this->get_logger(), "invalid VisionBarcode payload");
        return;
    }
    const QString barcode = QString::fromUtf8(payload.mid(10, barcode_len));
    const QByteArray tail = payload.mid(10 + barcode_len);
    QDataStream tail_stream(tail);
    tail_stream.setByteOrder(QDataStream::LittleEndian);

    tail_stream >> image_format_len;

    if (payload.size() < 4 + 4 + 2 + barcode_len + 2 + image_format_len) {
        RCLCPP_ERROR(this->get_logger(), "invalid VisionBarcode payload");
        return;
    }
    const QString image_format = QString::fromUtf8(payload.mid((12 + barcode_len), image_format_len));

    drone_msgs::msg::BarcodeCapture msg;
    msg.stamp.sec = sec;
    msg.stamp.nanosec = nanosec;
    msg.barcode = barcode.toStdString();
    msg.image_format = image_format.toStdString();

    vision_barcode_pub_->publish(msg);
}

void GroundLinkBridge::handleDeltaReport(const QByteArray &payload)
{
    QDataStream stream(payload);
    stream.setByteOrder(QDataStream::LittleEndian);

    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    stream >> x;
    stream >> y;
    stream >> z;

    if (payload.size() < 4 + 4 + 4) {
        RCLCPP_ERROR(this->get_logger(), "invalid Delta payload");
        return;
    }

    geometry_msgs::msg::Vector3 msg;
    msg.x = x;
    msg.y = y;
    msg.z = z;

    delta_pub_->publish(msg);
}

void GroundLinkBridge::handleUploadMissionSummaryResponse(uint16_t seq, const QByteArray &payload)
{
    QDataStream stream(payload);
    stream.setByteOrder(QDataStream::LittleEndian);

    quint8 success = 0;
    quint16 msg_len = 0;

    stream >> success;
    stream >> msg_len;

    if (payload.size() < 1 + 2 + msg_len + 2 + 4) {
        RCLCPP_ERROR(this->get_logger(), "invalid UploadMissionSummaryResp payload");
        return;
    }

    const QString message = QString::fromUtf8(payload.mid(3, msg_len));

    const QByteArray tail = payload.mid(3 + msg_len);
    QDataStream tail_stream(tail);
    tail_stream.setByteOrder(QDataStream::LittleEndian);

    quint16 saved_path_len = 0;
    tail_stream >> saved_path_len;

    if (payload.size() < 1 + 2 + msg_len + 2 + saved_path_len + 4) {
        RCLCPP_ERROR(this->get_logger(), "invalid UploadMissionSummaryResp payload");
        return;
    }

    const QString saved_path =
        QString::fromUtf8(payload.mid(5 + msg_len, saved_path_len));

    const QByteArray count_tail = payload.mid(5 + msg_len + saved_path_len);
    QDataStream count_stream(count_tail);
    count_stream.setByteOrder(QDataStream::LittleEndian);

    quint32 action_count = 0;
    count_stream >> action_count;

    auto it = pending_upload_calls_.find(seq);
    if (it != pending_upload_calls_.end()) {
        it->second.success = (success != 0);
        it->second.message = message.toStdString();
        it->second.saved_path = saved_path.toStdString();
        it->second.action_count = action_count;
        it->second.done = true;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "UploadMissionSummaryResp: success=%d, message=%s, saved_path=%s, action_count=%u",
        static_cast<int>(success),
        message.toStdString().c_str(),
        saved_path.toStdString().c_str(),
        static_cast<unsigned int>(action_count));
}

void GroundLinkBridge::handleStartOffboardResponse(uint16_t seq, const QByteArray &payload)
{
    QDataStream stream(payload);
    stream.setByteOrder(QDataStream::LittleEndian);

    quint8 success = 0;
    quint16 msg_len = 0;
    stream >> success;
    stream >> msg_len;

    if (payload.size() < 3 + msg_len) {
        RCLCPP_ERROR(this->get_logger(), "invalid StartOffboardResp payload");
        return;
    }

    const QByteArray msg_bytes = payload.mid(3, msg_len);
    const QString message = QString::fromUtf8(msg_bytes);

    auto it = pending_start_offboard_calls_.find(seq);
    if (it != pending_start_offboard_calls_.end()) {
        it->second.success = (success != 0);
        it->second.message = message.toStdString();
        it->second.done = true;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "StartOffboardResp: success=%d, message=%s",
        static_cast<int>(success),
        message.toStdString().c_str());
}

void GroundLinkBridge::handleStartTaskResponse(uint16_t seq, const QByteArray &payload)
{
    QDataStream stream(payload);
    stream.setByteOrder(QDataStream::LittleEndian);

    quint8 success = 0;
    quint16 msg_len = 0;
    stream >> success;
    stream >> msg_len;

    if (payload.size() < 3 + msg_len) {
        RCLCPP_ERROR(this->get_logger(), "invalid StartTaskResp payload");
        return;
    }

    const QByteArray msg_bytes = payload.mid(3, msg_len);
    const QString message = QString::fromUtf8(msg_bytes);

    auto it = pending_start_task_calls_.find(seq);
    if (it != pending_start_task_calls_.end()) {
        it->second.success = (success != 0);
        it->second.message = message.toStdString();
        it->second.done = true;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "StartTaskResp: success=%d, message=%s",
        static_cast<int>(success),
        message.toStdString().c_str());
}

void GroundLinkBridge::handleStopPushResponse(uint16_t seq, const QByteArray &payload)
{
    QDataStream stream(payload);
    stream.setByteOrder(QDataStream::LittleEndian);

    quint8 success = 0;
    quint16 msg_len = 0;
    stream >> success;
    stream >> msg_len;

    if (payload.size() < 3 + msg_len) {
        RCLCPP_ERROR(this->get_logger(), "invalid StopPushResp payload");
        return;
    }

    const QByteArray msg_bytes = payload.mid(3, msg_len);
    const QString message = QString::fromUtf8(msg_bytes);

    auto it = pending_stop_push_calls_.find(seq);
    if (it != pending_stop_push_calls_.end()) {
        it->second.success = (success != 0);
        it->second.message = message.toStdString();
        it->second.done = true;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "StopPushResp: success=%d, message=%s",
        static_cast<int>(success),
        message.toStdString().c_str());
}








/************************ 底层 *************************/

bool GroundLinkBridge::tryParseOnePacket(Packet &packet)
{
    while(true){
        //尝试从接收缓冲区中解析出完整的协议帧，直到无法再解析出新的帧为止
        while (rx_buffer_.size() >= 2) {
            const uint8_t b0 = static_cast<uint8_t>(rx_buffer_[0]);
            const uint8_t b1 = static_cast<uint8_t>(rx_buffer_[1]);
            if (b0 == lp::kSof1 && b1 == lp::kSof2) {
                break;
            }
            rx_buffer_.remove(0, 1);
        }

        if (rx_buffer_.size() < 11) {
            return false;
        }

        //从帧数据中提取载荷长度字段，计算出预期的帧长度，并与实际接收的帧长度进行比较，如果不匹配则认为帧不合法，继续尝试解析下一个帧
        const auto *data = reinterpret_cast<const uint8_t *>(rx_buffer_.constData());
        const uint16_t payload_len =
            static_cast<uint16_t>(data[7]) |
            (static_cast<uint16_t>(data[8]) << 8);

        //协议帧格式：2字节帧头 + 1字节版本 + 1字节类型 + 1字节标志 + 2字节序列号 + 2字节载荷长度 + N字节载荷 + 2字节CRC16校验
        const int frame_len = 9 + static_cast<int>(payload_len) + 2;
        if (rx_buffer_.size() < frame_len) {
            return false;
        }

        const QByteArray frame = rx_buffer_.left(frame_len);
        rx_buffer_.remove(0, frame_len);

        if (!validateFrame(frame)) {
            continue;
        }

        //从帧数据中提取消息类型、标志位、序列号和载荷数据，填充到Packet结构体中，供后续处理函数使用
        const auto *frame_data = reinterpret_cast<const uint8_t *>(frame.constData());
        packet.type = frame_data[3];
        packet.flags = frame_data[4];
        packet.seq = static_cast<uint16_t>(frame_data[5]) |
                    (static_cast<uint16_t>(frame_data[6]) << 8);
        packet.payload = frame.mid(9, payload_len);
        return true;
    }
}

QByteArray GroundLinkBridge::encodeFrame(
    uint8_t type,
    uint8_t flags,
    uint16_t seq,
    const QByteArray &payload) const
{
    //编码函数，用于将消息类型、标志位、序列号和载荷数据编码成一个完整的帧数据，准备发送
    QByteArray frame;
    frame.reserve(9 + payload.size() + 2);

    //帧格式：2字节帧头 + 1字节版本 + 1字节类型 + 1字节标志 + 2字节序列号 + 2字节载荷长度 + N字节载荷 + 2字节CRC16校验
    frame.append(static_cast<char>(lp::kSof1));
    frame.append(static_cast<char>(lp::kSof2));
    frame.append(static_cast<char>(lp::kVersion));
    frame.append(static_cast<char>(type));
    frame.append(static_cast<char>(flags));

    frame.append(static_cast<char>(seq & 0xFF));
    frame.append(static_cast<char>((seq >> 8) & 0xFF));

    const uint16_t payload_len = static_cast<uint16_t>(payload.size());
    frame.append(static_cast<char>(payload_len & 0xFF));
    frame.append(static_cast<char>((payload_len >> 8) & 0xFF));

    frame.append(payload);

    const uint8_t *crc_begin = reinterpret_cast<const uint8_t *>(frame.constData() + 2);
    const int crc_len = frame.size() - 2;
    const uint16_t crc = crc16_ccitt(crc_begin, crc_len);

    frame.append(static_cast<char>(crc & 0xFF));
    frame.append(static_cast<char>((crc >> 8) & 0xFF));

    return frame;
}

bool GroundLinkBridge::validateFrame(const QByteArray &frame) const
{
    //验证函数，用于检查接收到的帧数据是否符合协议格式和校验要求，确保数据的正确性和完整性
    if (frame.size() < 11) {
        return false;
    }

    const auto *data = reinterpret_cast<const uint8_t *>(frame.constData());

    //检查帧头、版本号、载荷长度和CRC校验等字段，确保帧的合法性和完整性
    if (data[0] != lp::kSof1 || data[1] != lp::kSof2) {
        return false;
    }

    if (data[2] != lp::kVersion) {
        return false;
    }

    //从帧数据中提取载荷长度字段，计算出预期的帧长度，并与实际接收的帧长度进行比较，如果不匹配则认为帧不合法
    const uint16_t payload_len =
        static_cast<uint16_t>(data[7]) |
        (static_cast<uint16_t>(data[8]) << 8);

    const int expected_size = 9 + static_cast<int>(payload_len) + 2;
    if (frame.size() != expected_size) {
        return false;
    }

    //从帧数据中提取CRC16校验字段，计算出接收到的帧数据的CRC16校验值，并与提取的CRC16值进行比较，如果不匹配则认为帧数据有误，需要丢弃
    const uint16_t recv_crc =
        static_cast<uint16_t>(data[frame.size() - 2]) |
        (static_cast<uint16_t>(data[frame.size() - 1]) << 8);

    //计算CRC16校验值时，需要从帧数据的第3个字节开始（即版本号字段），一直计算到载荷数据的最后一个字节，才能得到正确的CRC16值
    const uint16_t calc_crc = crc16_ccitt(data + 2, frame.size() - 4);

    //如果帧数据的CRC16校验通过，则认为帧数据合法，可以继续进行后续处理；否则认为帧数据有误，需要丢弃
    return recv_crc == calc_crc;
}

void GroundLinkBridge::onRetryTimer()
{
    const auto now = this->now();

    //检查待重试的请求是否超时，并进行重试
    for (auto it = pending_requests_.begin(); it != pending_requests_.end(); ) {
        auto &pending = it->second;
        if (now < pending.deadline) {
            ++it;
            continue;
        }

        //如果重试次数超过3次，则认为请求失败，删除请求信息，并记录错误日志
        if (pending.retry_count >= 3) {
            RCLCPP_ERROR(this->get_logger(), "request seq=%u timeout", pending.seq);
            it = pending_requests_.erase(it);
            continue;
        }

        //重试发送协议帧到串口，并更新重试次数和截止时间，继续检查下一个请求
        serial_.write(pending.encoded_frame);
        pending.retry_count += 1;
        pending.deadline = now + rclcpp::Duration::from_seconds(0.3);
        ++it;
    }
}