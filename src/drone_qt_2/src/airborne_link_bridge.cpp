#include "drone_qt_2/airborne_link_bridge.hpp"

using namespace std::chrono_literals;

namespace {
constexpr uint8_t kTypeUploadMissionSummaryReq = 0x01;//消息类型：上传 mission summary 请求
constexpr uint8_t kTypeStartOffboardReq = 0x02;//消息类型：start_offboard 请求
constexpr uint8_t kTypeStartTaskReq = 0x03;//消息类型：start_task 请求
constexpr uint8_t kTypeStopPushReq = 0x04;//消息类型：stop_push 请求
constexpr uint8_t kTypeAck = 0x05;//消息类型：确认帧

constexpr uint8_t kTypeUploadMissionSummaryResp = 0x81;//消息类型：上传 mission summary 响应
constexpr uint8_t kTypeStartOffboardResp = 0x82;//消息类型：start_offboard 响应
constexpr uint8_t kTypeStartTaskResp = 0x83;//消息类型：start_task 响应
constexpr uint8_t kTypeStopPushResp = 0x84;//消息类型：stop_push 响应
constexpr uint8_t kTypeDroneStatus = 0x90;//消息类型：无人机状态
constexpr uint8_t kTypereturngroup = 0x91;//消息类型：回传路线报告
constexpr uint8_t kTypeTaskStatus = 0x92;//消息类型：任务状态
constexpr uint8_t kTypePathReady = 0x93;//消息类型：路径就绪
}

AirborneLinkBridge::AirborneLinkBridge()
    : QObject(),
      rclcpp::Node("airborne_link_bridge")
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
            publishDroneStatus(msg);
        });

    task_status_sub_ = this->create_subscription<drone_msgs::msg::TaskStatus>(
        "/drone/task/status",
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        [this](const drone_msgs::msg::TaskStatus::SharedPtr msg) {
            publishTaskStatus(msg);
        });

    path_ready_sub_ = this->create_subscription<drone_msgs::msg::ReadyStatus>(
        "/drone/control/path_ready",
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        [this](const drone_msgs::msg::ReadyStatus::SharedPtr msg) {
            publishPathReady(msg);
        });
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
    //如果收到的是ACK帧，说明对方已经收到之前发送的消息，无需再处理，只需要记录ACK即可
    if (packet.type == kTypeAck) {
        return;
    }

    sendAck(packet.seq);

    //根据协议帧的类型调用不同的处理函数，处理函数会根据载荷内容执行相应的逻辑，并可能发送响应帧回去
    switch (packet.type) {
    case kTypeUploadMissionSummaryReq:
        handleUploadMissionSummaryRequest(packet.seq, packet.payload);
        break;
    case kTypeStartOffboardReq:
        handleStartOffboardRequest(packet.seq, packet.payload);
        break;
    case kTypeStartTaskReq:
        handleStartTaskRequest(packet.seq, packet.payload);
        break;
    case kTypeStopPushReq:
        handleStopPushRequest(packet.seq, packet.payload);
        break;
    default:
        RCLCPP_WARN(this->get_logger(), "unknown packet type: 0x%02X", packet.type);
        break;
    }
}

void AirborneLinkBridge::handleStartOffboardRequest(uint16_t seq, const QByteArray &payload)
{
    (void)payload;

    if (!start_offboard_client_ || !start_offboard_client_->service_is_ready()) {
        return;
    }

    auto request = std::make_shared<drone_msgs::srv::StartOffboard::Request>();
    request->request_source = "ground_link_bridge";

    start_offboard_client_->async_send_request(
        request,
        [this, seq](rclcpp::Client<drone_msgs::srv::StartOffboard>::SharedFuture future)
        {
            const auto response = future.get();
            QByteArray payload;
            // 第一版文档里省略具体字节打包细节
            sendResponse(kTypeStartOffboardResp, seq, payload);
        });
}