#include "drone_qt/ground_link_bridge.hpp"

#include <QDataStream>

using namespace std::chrono_literals;

namespace {
constexpr uint8_t kSof1 = 0xAA;//帧头1，固定值0xAA，表示一帧数据的开始
constexpr uint8_t kSof2 = 0x55;//帧头2，固定值0x55，与kSof1一起用于帧同步，确保接收端能够正确识别帧的起始位置
constexpr uint8_t kVersion = 0x01;//协议版本号，当前版本为0x01，接收端可以根据这个版本号来解析不同格式的帧数据
constexpr uint8_t kFlagNeedAck = 0x01;//标志位，表示发送的请求需要对方回复确认帧
constexpr uint8_t kFlagAck = 0x02;//标志位，表示为一个确认帧

constexpr uint8_t kTypeUploadMissionSummaryReq = 0x01;//消息类型：上传 mission summary 请求
constexpr uint8_t kTypeStartOffboardReq = 0x02;//消息类型：start_offboard 请求
constexpr uint8_t kTypeStartTaskReq = 0x03;//消息类型：start_task 请求
constexpr uint8_t kTypeStopPushReq = 0x04;//消息类型：stop_push 请求
constexpr uint8_t kTypeAck = 0x05;//消息类型：确认帧

constexpr uint8_t kTypeUploadMissionSummaryResp = 0x81;//消息类型：上传 mission summary 响应
constexpr uint8_t kTypeStartOffboardResp = 0x82;//消息类型：start_offboard 响应
constexpr uint8_t kTypeStartTaskResp = 0x83;//消息类型：start_task 响应
constexpr uint8_t kTypeStopPushResp = 0x84;//消息类型：stop_push 响应
constexpr uint8_t kTypeDroneStatus = 0x90;//消息类型：无人机状态报告
constexpr uint8_t kTypereturngroup = 0x91;//消息类型：回传路线报告
constexpr uint8_t kTypeTaskStatus = 0x92;//消息类型：任务状态报告
constexpr uint8_t kTypePathReady = 0x93;//消息类型：路径就绪报告
}

GroundLinkBridge::GroundLinkBridge()
    : QObject(),
      rclcpp::Node("ground_link_bridge")
{
    setupRosInterfaces();
    setupSerial();

    //启动一个定时器，周期为100毫秒，用于检查待重试的请求是否超时，并进行重试
    retry_timer_ = this->create_wall_timer(
        100ms,
        std::bind(&GroundLinkBridge::onRetryTimer, this));
}

void GroundLinkBridge::setupRosInterfaces()
{
    /*************** 创建与机载端相对应的通信接口 ***************/
    status_pub_ = this->create_publisher<drone_msgs::msg::DroneStatus>(
        "/drone/status", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());

    task_status_pub_ = this->create_publisher<drone_msgs::msg::TaskStatus>(
        "/drone/task/status", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    path_ready_pub_ = this->create_publisher<drone_msgs::msg::ReadyStatus>(
        "/drone/control/path_ready", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    upload_mission_summary_srv_ = this->create_service<drone_msgs::srv::UploadMissionSummary>(
        "/drone/upload_mission_summary",
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
    /************************************************************/
}

void GroundLinkBridge::setupSerial()
{
    serial_.setPortName("/dev/ttyUSB0");
    serial_.setBaudRate(QSerialPort::Baud57600);
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

uint16_t GroundLinkBridge::nextSeq()
{
    //生成一个新的序列号，确保每个请求都有一个唯一的标识符，便于匹配响应和处理重试逻辑
    return next_seq_++;
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

void GroundLinkBridge::handlePacket(const Packet &packet)
{
    //如果收到的是ACK帧，则调用handleAck函数处理，并返回
    if (packet.type == kTypeAck) {
        handleAck(packet.seq);
        return;
    }

    //根据帧的类型调用不同的处理逻辑，并发送相应的ACK或响应数据
    switch (packet.type) {
    case kTypeDroneStatus:
        handleDroneStatusReport(packet.payload);
        break;
    case kTypeTaskStatus:
        handleTaskStatusReport(packet.payload);
        break;
    case kTypePathReady:
        handlePathReadyReport(packet.payload);
        break;
    case kTypeUploadMissionSummaryResp:
        handleUploadMissionSummaryResponse(packet.payload);
        break;
    case kTypeStartOffboardResp:
        handleStartOffboardResponse(packet.payload);
        break;
    case kTypeStartTaskResp:
        handleStartTaskResponse(packet.payload);
        break;
    case kTypeStopPushResp:
        handleStopPushResponse(packet.payload);
        break;
    default:
        RCLCPP_WARN(this->get_logger(), "unknown packet type: 0x%02X", packet.type);
        break;
    }
}

void GroundLinkBridge::handleAck(uint16_t seq)
{
    //收到ACK帧后，从待重试列表中删除对应的请求信息，表示请求已经成功完成，不需要再重试
    pending_requests_.erase(seq);
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