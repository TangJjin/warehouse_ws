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

    //启动一个定时器，周期为100毫秒，用于检查待重试的请求是否超时，并进行重试
    retry_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&GroundLinkBridge::onRetryTimer, this));
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
            sendPacket(kTypeUploadMissionSummaryReq, kFlagNeedAck, payload, true);

            // 第一版这里只给出桥接骨架。
            // 真正落地时，这里需要把 response 暂存起来，等机载响应回来后再填充。
            response->success = true;
            response->message = "请求已转发到数传链路，需补全异步响应绑定";
            response->saved_path = "";
            response->action_count = 0;
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
            sendPacket(lp::kTypeStartOffboardReq, lp::kFlagNeedAck, payload, true);

            response->success = true;
            response->message = "StartOffboard 请求已发出，后续再补完整异步响应绑定";
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
            sendPacket(lp::kTypeStartTaskReq, lp::kFlagNeedAck, payload, true);

            response->success = true;
            response->message = "StartTask 请求已发出，后续再补完整异步响应绑定";
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
            sendPacket(lp::kTypeStopPushReq, lp::kFlagNeedAck, payload, true);

            response->success = true;
            response->message = "StopPush 请求已发出，后续再补完整异步响应绑定";
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

    QObject::connect(&serial_, &QSerialPort::readyRead, [this]() {
        onSerialReadyRead();
    });

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

void GroundLinkBridge::sendPacket(uint8_t type, uint8_t flags, const QByteArray &payload, bool need_ack)
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
        handleUploadMissionSummaryResponse(packet.payload);
        break;
    case lp::kTypeStartOffboardResp:
        handleStartOffboardResponse(packet.payload);
        break;
    case lp::kTypeStartTaskResp:
        handleStartTaskResponse(packet.payload);
        break;
    case lp::kTypeStopPushResp:
        handleStopPushResponse(packet.payload);
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
    
}

QByteArray GroundLinkBridge::encodeStartOffboardRequest(
    const drone_msgs::srv::StartOffboard::Request &request) const
{
    //编码函数，用于将ROS服务请求对象编码成协议帧的载荷格式，准备发送给机载端
    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::LittleEndian);

    const QByteArray source = QByteArray::fromStdString(request.request_source);
    stream << static_cast<quint16>(source.size());
    payload.append(source);
    return payload;
}

QByteArray GroundLinkBridge::encodeStartTaskRequest(
    const drone_msgs::srv::StartTask::Request &request) const
{
    
}

QByteArray GroundLinkBridge::encodeStopPushRequest(
    const drone_msgs::srv::StartTask::Request &request) const
{
    
}








/******************* 数据处理层(接收层) ********************/

void GroundLinkBridge::handleDroneStatusReport(const QByteArray &payload)
{
    QDataStream stream(payload);
    stream.setByteOrder(QDataStream::LittleEndian);

    quint8 connected = 0;
    float battery_percent = 0.0f;
    qint32 flight_mode = 0;
    quint8 armed = 0;
    quint16 state_len = 0;

    stream >> connected;
    stream >> battery_percent;
    stream >> flight_mode;
    stream >> armed;
    stream >> state_len;

    if (payload.size() < 1 + 4 + 4 + 1 + 2 + state_len) {
        RCLCPP_ERROR(this->get_logger(), "invalid DroneStatus payload");
        return;
    }

    const QString state = QString::fromUtf8(payload.mid(12, state_len));

    drone_msgs::msg::DroneStatus msg;
    msg.connected = (connected != 0);
    msg.battery_percent = battery_percent;
    msg.flight_mode = static_cast<uint8_t>(flight_mode);
    msg.armed = (armed != 0);
    msg.state = state.toStdString();

    status_pub_->publish(msg);
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

void GroundLinkBridge::handleTaskStatusReport(const QByteArray &payload)
{
    
}

void GroundLinkBridge::handleReturnWorldGroupReport(const QByteArray &payload)
{

}

void GroundLinkBridge::handleVisionBarcodeReport(const QByteArray &payload)
{

}

void GroundLinkBridge::handleDeltaReport(const QByteArray &payload)
{

}

void GroundLinkBridge::handleUploadMissionSummaryResponse(const QByteArray &payload)
{

}

void GroundLinkBridge::handleStartOffboardResponse(const QByteArray &payload)
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

    RCLCPP_INFO(
        this->get_logger(),
        "StartOffboardResp: success=%d, message=%s",
        static_cast<int>(success),
        message.toStdString().c_str());
}

void GroundLinkBridge::handleStartTaskResponse(const QByteArray &payload)
{

}

void GroundLinkBridge::handleStopPushResponse(const QByteArray &payload)
{

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