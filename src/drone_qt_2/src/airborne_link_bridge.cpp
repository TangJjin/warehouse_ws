#include "drone_qt_2/airborne_link_bridge.hpp"

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

AirborneLinkBridge::AirborneLinkBridge()
    : rclcpp::Node("airborne_link_bridge")
{
    setupRosInterfaces();
    setupSerial();
}

void AirborneLinkBridge::setupRosInterfaces()
{
    upload_mission_summary_client_ = this->create_client<drone_msgs::srv::UploadMissionSummary>(
        "/drone/upload_mission_summary");

    start_offboard_client_ = this->create_client<drone_msgs::srv::StartOffboard>(
        "/drone/start_offboard");

    start_task_client_ = this->create_client<drone_msgs::srv::StartTask>(
        "/drone/start_task");

    stop_push_client_ = this->create_client<drone_msgs::srv::StartTask>(
        "/drone/stop_push");

    status_sub_ = this->create_subscription<drone_msgs::msg::DroneStatus>(
        "/drone/status",
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
        [this](const drone_msgs::msg::DroneStatus::SharedPtr msg) {
            //publishDroneStatus(msg);
        });

    task_status_sub_ = this->create_subscription<drone_msgs::msg::TaskStatus>(
        "/drone/task/status",
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        [this](const drone_msgs::msg::TaskStatus::SharedPtr msg) {
            //publishTaskStatus(msg);
        });

    path_ready_sub_ = this->create_subscription<drone_msgs::msg::ReadyStatus>(
        "/drone/control/path_ready",
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        [this](const drone_msgs::msg::ReadyStatus::SharedPtr msg) {
            //publishPathReady(msg);
        });
}

/************************ 用户层 *************************/

void AirborneLinkBridge::setupSerial()
{
    serial_.setPortName("/dev/ttyUSB0");
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

void AirborneLinkBridge::onSerialReadyRead()
{
    rx_buffer_.append(serial_.readAll());
    //尝试从接收缓冲区中解析出完整的协议帧，直到无法再解析出新的帧为止
    Packet packet;
    while (tryParseOnePacket(packet)) {
        handlePacket(packet);
    }
}

void AirborneLinkBridge::handlePacket(const Packet &packet)
{
    //如果收到的是ACK帧，则调用handleAck函数处理，并返回
    if (packet.type == lp::kTypeAck) {
        return;
    }

    //如果收到的帧需要ACK，则先发送ACK帧，通知对方已经收到消息，避免对方继续重试
    if (packet.flags & lp::kFlagNeedAck) {
        sendAck(packet.seq);
    }

    //根据帧的类型调用不同的处理逻辑，并发送相应的ACK或响应数据
    switch (packet.type) {
    case lp::kTypeStartOffboardReq:
        handleStartOffboardRequest(packet.seq, packet.payload);

        RCLCPP_INFO(this->get_logger(), "recv request type=0x%02X seq=%u", packet.type, packet.seq);
        break;
    default:
        RCLCPP_WARN(this->get_logger(), "unknown packet type: 0x%02X", packet.type);
        break;
    }
}

void AirborneLinkBridge::sendAck(uint16_t seq)
{
    const QByteArray empty_payload;
    const QByteArray frame = encodeFrame(lp::kTypeAck, lp::kFlagAck, seq, empty_payload);
    serial_.write(frame);
}

void AirborneLinkBridge::sendResponse(uint8_t type, uint16_t seq, const QByteArray &payload)
{
    const QByteArray frame = encodeFrame(type, 0, seq, payload);
    serial_.write(frame);
    RCLCPP_INFO(this->get_logger(), "sending response type=0x%02X seq=%u", type, seq);
}





/******************* 数据处理层(发送层) ********************/







/******************* 数据处理层(接收层) ********************/

void AirborneLinkBridge::handleStartOffboardRequest(uint16_t seq, const QByteArray &payload)
{
    if (!start_offboard_client_ || !start_offboard_client_->service_is_ready()) {
        QByteArray resp_payload;
        QDataStream resp_stream(&resp_payload, QIODevice::WriteOnly);
        resp_stream.setByteOrder(QDataStream::LittleEndian);

        const QByteArray msg = QByteArray("service /drone/start_offboard not ready");
        resp_stream << static_cast<quint8>(0);
        resp_stream << static_cast<quint16>(msg.size());
        resp_payload.append(msg);

        sendResponse(lp::kTypeStartOffboardResp, seq, resp_payload);
        return;
    }

    QDataStream stream(payload);
    stream.setByteOrder(QDataStream::LittleEndian);

    quint16 source_len = 0;
    stream >> source_len;

    if (payload.size() < 2 + source_len) {
        QByteArray resp_payload;
        QDataStream resp_stream(&resp_payload, QIODevice::WriteOnly);
        resp_stream.setByteOrder(QDataStream::LittleEndian);

        const QByteArray msg = QByteArray("invalid payload for StartOffboardReq");
        resp_stream << static_cast<quint8>(0);
        resp_stream << static_cast<quint16>(msg.size());
        resp_payload.append(msg);

        sendResponse(lp::kTypeStartOffboardResp, seq, resp_payload);
        return;
    }

    const QByteArray source_bytes = payload.mid(2, source_len);
    const std::string request_source = source_bytes.toStdString();

    auto request = std::make_shared<drone_msgs::srv::StartOffboard::Request>();
    request->request_source = request_source;

    RCLCPP_INFO(this->get_logger(), "calling /drone/start_offboard, request_source=%s",request_source.c_str());

    start_offboard_client_->async_send_request(
        request,
        [this, seq](rclcpp::Client<drone_msgs::srv::StartOffboard>::SharedFuture future)
        {
            const auto response = future.get();

            QByteArray resp_payload;
            QDataStream resp_stream(&resp_payload, QIODevice::WriteOnly);
            resp_stream.setByteOrder(QDataStream::LittleEndian);

            const QByteArray msg = QByteArray::fromStdString(response->message);
            resp_stream << static_cast<quint8>(response->success ? 1 : 0);
            resp_stream << static_cast<quint16>(msg.size());
            resp_payload.append(msg);

            sendResponse(lp::kTypeStartOffboardResp, seq, resp_payload);
        });
}








/************************ 底层 *************************/

bool AirborneLinkBridge::tryParseOnePacket(Packet &packet)
{
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

QByteArray AirborneLinkBridge::encodeFrame(
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

bool AirborneLinkBridge::validateFrame(const QByteArray &frame) const
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