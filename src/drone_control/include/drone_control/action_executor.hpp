#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>

#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <memory>
#include <queue>
#include <sstream>
#include <string>

#include "drone_action.hpp"

using offboard_run::ActionStatus;
using offboard_run::ActionType;
using offboard_run::DroneAction;
using offboard_run::HoldAxis;
using offboard_run::SpatialPoint;

class Vec3dPID {
    public:
    Vec3dPID(double p, double i, double d)
        : p_gain_(p),
          i_gain_(i),
          d_gain_(d),
          integral_(0.0, 0.0, 0.0),
          last_error_(0.0, 0.0, 0.0){}

    Eigen::Vector3d update(const Eigen::Vector3d &error, double dt) {
        integral_ += error * dt;
        const Eigen::Vector3d derivative = (error - last_error_) / dt;
        last_error_ = error;
        return p_gain_ * error + i_gain_ * integral_ + d_gain_ * derivative;
    }

    Eigen::Vector3d update(const Eigen::Vector3d &error,
                            const rclcpp::Time &time_stamp) {
        if (last_time_.nanoseconds() == 0) {
            last_time_ = time_stamp;
            return Eigen::Vector3d::Zero();
        }

        const double dt = (time_stamp - last_time_).seconds();
        last_time_ = time_stamp;
        if (dt <= 0.0) {
            return Eigen::Vector3d::Zero();
        }
        return update(error, dt);
    }

    private:
    double p_gain_;
    double i_gain_;
    double d_gain_;

    rclcpp::Time last_time_;
    Eigen::Vector3d integral_;
    Eigen::Vector3d last_error_;
};

struct TaskRuntimeStatus {
  bool task_running;
  std::string action_name;
  int32_t action_step;
};

class ActionExecutor {
    public:
    ActionExecutor(const rclcpp::Node::SharedPtr &node,
                    tf2_ros::Buffer &tf_buffer) : node_(node), tf_buffer_(tf_buffer) 
                    {
                        setpoint_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
                            "/mavros/setpoint_position/local", rclcpp::QoS(10).reliable()
                        );

                        step_pub_ = node_->create_publisher<std_msgs::msg::String>("/step", 10);
                        mission_status_pub_ =
                            node_->create_publisher<std_msgs::msg::String>("/mission_status", 10);

                        state_sub_ = node_->create_subscription<mavros_msgs::msg::State>(
                            "/mavros/state", 10,
                            std::bind(&ActionExecutor::state_callback, this, std::placeholders::_1));
                        
                        pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
                            "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
                            std::bind(&ActionExecutor::pose_callback, this, std::placeholders::_1));

                        camera_aim_sub_ = node_->create_subscription<geometry_msgs::msg::Point>(
                            "/camera_aiming_center", rclcpp::SensorDataQoS(),
                            std::bind(&ActionExecutor::cameraAim_callback, this, std::placeholders::_1));

                        arming_client_ = node_->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
                        set_mode_client_ = node->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
                        land_client_ = node_->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");

                        last_finish_pose_.header.frame_id = "world_enu";
                        last_finish_pose_.pose.orientation.w = 1.0;

                        last_camera_aim_time_ = node_->now();

                        RCLCPP_INFO(node_->get_logger(),
                                    "动作执行器初始化完成，已准备执行任务。");
                    }

                    void addAction(const std::shared_ptr<DroneAction> &action) {
                        action_queue_.push(action);
                        RCLCPP_INFO(node_->get_logger(), "已添加动作，当前队列长度：%zu", action_queue_.size());
                    }

                    void clearAction() {
                        while (!action_queue_.empty()) {
                            action_queue_.pop();
                        }
                        action_id_ = 0;
                        current_action_.reset();
                        resetActionRuntimeState();
                    }

                    void emergencyStop() {
                        clearAction();
                        if (current_pose_received_) {
                            sendPositionSetpoint(current_pose_);
                        }
                    }

                    void sendPositionSetpoint(const geometry_msgs::msg::PoseStamped &pose)
                    {
                        geometry_msgs::msg::PoseStamped target = pose;
                        target.header.stamp = node_->now();
                        if (target.header.frame_id.empty()) {
                            target.header.frame_id = "world_enu";
                        }
                        setpoint_pub_->publish(target);
                    }

                    void sendDummyPose(double altitude = 0.0) {
                        if (current_pose_received_) {
                            sendPositionSetpoint(current_pose_);
                            return;
                        }

                        geometry_msgs::msg::PoseStamped dummy_pose;
                        dummy_pose.header.frame_id = "world_enu";
                        dummy_pose.header.stamp = node_->now();
                        dummy_pose.pose.position.z = altitude;
                        dummy_pose.pose.orientation.w = 1.0;
                        sendPositionSetpoint(dummy_pose);
                    }

                    bool isIdle() const
                    {
                        return !current_action_ && action_queue_.empty();
                    }

                    TaskRuntimeStatus getTaskRuntimeStatus() const {
                        TaskRuntimeStatus status;
                        status.task_running = static_cast<bool>(current_action_);
                        status.action_step = current_action_ ? action_id_ : 0;
                        status.action_name = current_action_ ? actionTypeToString(current_action_->getType()) : "idle";
                        return status;
                    }

                    void controlLoop() {
                        if (!current_pose_received_) {
                            sendDummyPose();
                            return;
                        }

                        if(!current_state_.connected) {
                            sendPositionSetpoint(current_pose_);
                            return;
                        }

                        const bool executing_land =
                            current_action_ &&
                            current_action_->getType() == ActionType::LAND;
                        if(!current_state_.armed && !executing_land) {
                            sendPositionSetpoint(current_pose_);
                            return;
                        }

                        std_msgs::msg::String step_msg;
                        step_msg.data = std::to_string(action_id_);
                        step_pub_->publish(step_msg);

                        if(!current_action_ && !action_queue_.empty()) {
                            current_action_ = action_queue_.front();
                            action_queue_.pop();
                            action_id_++;
                            current_action_->setStatus(ActionStatus::EXECUTING);
                            current_action_->setStartTime(node_->now());
                            force_status_publish_ = true;
                            RCLCPP_INFO(node_->get_logger(), "开始执行动作，动作类型编号：%d", static_cast<int>(current_action_->getType()));
                        }

                        if (current_action_) {
                            executeAction(current_action_);
                        }
                    }

    private:
    Eigen::Vector3d bodyVectorToEnu(const Eigen::Vector3d &body_vec) const {
        Eigen::Quaterniond q_current(
            current_pose_.pose.orientation.w,
            current_pose_.pose.orientation.x,
            current_pose_.pose.orientation.y,
            current_pose_.pose.orientation.z
        );
        return q_current * body_vec;
    }

    void resetActionRuntimeState() {
        aim_close_count_ = 0;
        land_mode_request_sent_ = false;
        land_low_altitude_count_ = 0;
    }

    void completeCurrentAction(const std::string &message) {
        if (current_action_) {
            current_action_->setStatus(ActionStatus::COMPLETED);
        }
        last_finish_pose_ = current_pose_;
        current_action_.reset();
        resetActionRuntimeState();
        broadcastStatus(message);
    }

    void failCurrentAction(const std::string &message) {
        if (current_action_) {
            current_action_->setStatus(ActionStatus::FAILED);
        }
        current_action_.reset();
        resetActionRuntimeState();
        RCLCPP_ERROR(node_->get_logger(), "%s", message.c_str());
    }

    double getYawDeg(const geometry_msgs::msg::PoseStamped &pose) const {
        tf2::Quaternion q;
        tf2::fromMsg(pose.pose.orientation, q);
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        return yaw * 57.29577951308232;
    }

    std::string formatPose(const geometry_msgs::msg::PoseStamped &pose) const {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2)
            << "x=" << pose.pose.position.x << " m, "
            << "y=" << pose.pose.position.y << " m, "
            << "z=" << pose.pose.position.z << " m, "
            << "yaw=" << getYawDeg(pose) << " deg";
        return oss.str();
    }

    bool transformToWorldBody(const geometry_msgs::msg::PoseStamped &source,
                              geometry_msgs::msg::PoseStamped &target) {
        try {
            target = tf_buffer_.transform(source, "world_body");
            return true;
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "状态播报坐标变换失败：%s，暂时使用 ENU 坐标。",
                                 ex.what());
            target = source;
            return false;
        }
    }

    std::string formatStatusPose(
        const geometry_msgs::msg::PoseStamped &pose,
        const char *fallback_frame_text,
        bool &using_world_body) {
        geometry_msgs::msg::PoseStamped world_body_pose;
        if (transformToWorldBody(pose, world_body_pose)) {
            using_world_body = true;
            return formatPose(world_body_pose);
        }

        using_world_body = false;
        return std::string(fallback_frame_text) + " " + formatPose(world_body_pose);
    }

    std::string formatSeconds(double seconds) const {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1) << seconds;
        return oss.str();
    }

    std::string formatMeters(double meters) const {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2) << meters;
        return oss.str();
    }

    void publishStatus(const std::string &message) {
        std_msgs::msg::String status_msg;
        status_msg.data = message;
        mission_status_pub_->publish(status_msg);
    }

    void broadcastStatus(const std::string &message) {
        publishStatus(message);
        RCLCPP_INFO(node_->get_logger(), "%s", message.c_str());
    }

    void broadcastStatusThrottled(const std::string &message) {
        const rclcpp::Time now = node_->now();
        const bool should_publish =
            force_status_publish_ ||
            last_status_publish_time_.nanoseconds() == 0 ||
            (now - last_status_publish_time_).seconds() >= status_publish_period_s_;

        if (!should_publish) {
            return;
        }

        last_status_publish_time_ = now;
        force_status_publish_ = false;
        broadcastStatus(message);
    }

    void executeAction(const std::shared_ptr<DroneAction> &action) {
        switch(action->getType()) {
            case ActionType::MOVE_TO_POSITION:
                executeMoveToPosition(action);
                break;
            case ActionType::HOVER:
                executeHover(action);
                break;
            case ActionType::CAMERA_AIM:
                executeCameraAim(action);
                break;
            case ActionType::LAND:
                executeLand(action);
                break;
            case ActionType::TAKEOFF:
                executeTakeoff(action);
                break;
            default:
                RCLCPP_ERROR(node_->get_logger(), "未知动作类型，已保持当前位置。");
                sendPositionSetpoint(current_pose_);
                break;
        }
    }

    void executeMoveToPosition(const std::shared_ptr<DroneAction> &action) {
        geometry_msgs::msg::PoseStamped target = action->getTargetPose();

        if (action->getFrame() == DroneAction::Frame::WORLD_BODY) {
            try {
                target = tf_buffer_.transform(target, "world_enu");
            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN(node_->get_logger(), "坐标变换失败：%s，已保持当前位置。", ex.what());
                sendPositionSetpoint(current_pose_);
                return;
            }
        } else if (action->getFrame() == DroneAction::Frame::BODY) {
            const Eigen::Vector3d delta = bodyVectorToEnu(Eigen::Vector3d(target.pose.position.x, 
                target.pose.position.y, target.pose.position.z));
            target = last_finish_pose_;
            target.pose.position.x += delta.x();
            target.pose.position.y += delta.y();
            target.pose.position.z += delta.z();

        }

        const SpatialPoint current(current_pose_);
        const SpatialPoint target_point(target);
        const double distance_to_target = current.distance(target_point);
        if (distance_to_target < action->getPositionTolerance()) {
            aim_close_count_++;
            if (aim_close_count_ > 20) {
                completeCurrentAction("已到达目标位置。");
                return;
            }
        } else {
            aim_close_count_ = 0;
        }

        bool current_uses_world_body = false;
        bool target_uses_world_body = false;
        const std::string current_text =
            formatStatusPose(current_pose_, "ENU", current_uses_world_body);
        const std::string target_text =
            formatStatusPose(target, "ENU", target_uses_world_body);
        const std::string frame_text =
            current_uses_world_body && target_uses_world_body
                ? "初始点坐标系 world_body"
                : "坐标变换未就绪，混合坐标";

        broadcastStatusThrottled(
            "航点飞行中（" + frame_text + "）：当前 " + current_text +
            "，目标 " + target_text +
            "，剩余距离=" + formatSeconds(distance_to_target) + " m。");
        sendPositionSetpoint(target);
    }

    void executeHover(const std::shared_ptr<DroneAction> &action) {
        sendPositionSetpoint(last_finish_pose_);
        const double elapsed = (node_->now() - action->getStartTime()).seconds();
        const double remaining = std::max(0.0, action->getHoverTime() - elapsed);
        if (elapsed > action->getHoverTime()) {
            completeCurrentAction("悬停动作已完成。");
            return;
        }

        bool hold_uses_world_body = false;
        const std::string hold_text =
            formatStatusPose(last_finish_pose_, "ENU", hold_uses_world_body);
        const std::string frame_text =
            hold_uses_world_body ? "初始点坐标系 world_body"
                                 : "坐标变换未就绪，ENU";

        broadcastStatusThrottled(
            "悬停中（" + frame_text + "）：保持位置 " + hold_text +
            "，已悬停=" + formatSeconds(elapsed) + " s，剩余=" +
            formatSeconds(remaining) + " s。");
    }

    void executeCameraAim(const std::shared_ptr<DroneAction> &action) {
        if((node_->now() - last_camera_aim_time_).seconds() > 0.5) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "相机瞄准数据超时，已退回位置移动控制。");
            executeMoveToPosition(action);
            return;
        }

        if (std::abs(camera_aim_diff_.x) < action->getCameraAimTolerance() &&
            std::abs(camera_aim_diff_.y) < action->getCameraAimTolerance()) {
                aim_close_count_++;
                if (aim_close_count_ > 20) {
                    completeCurrentAction("相机瞄准动作已完成。");
                    return;
                }
            } else {
                aim_close_count_ = 0;
            }
        const Eigen::Vector3d camera_aim_vector{
            camera_aim_diff_.x, camera_aim_diff_.y, camera_aim_diff_.z
        };

        Eigen::Vector3d body_delta = pid_cam_aim_.update(camera_aim_vector, node_->now());

        constexpr double kMaxStep = 0.05;
        for (int i = 0; i < 3; ++i) {
            body_delta[i] = std::clamp(body_delta[i], -kMaxStep, kMaxStep);
        }

        const Eigen::Vector3d enu_delta = bodyVectorToEnu(body_delta);
        geometry_msgs::msg::PoseStamped target_pose = current_pose_;
        target_pose.pose.position.x += enu_delta.x();
        target_pose.pose.position.y += enu_delta.y();
        target_pose.pose.position.z += enu_delta.z();

        switch(action->getHoldAxis()) {
            case HoldAxis::X:
                target_pose.pose.position.x = action->getTargetPose().pose.position.x;
            break;

            case HoldAxis::Y:
                target_pose.pose.position.y = action->getTargetPose().pose.position.y;
            break;

            case HoldAxis::Z:
            target_pose.pose.position.z = action->getTargetPose().pose.position.z;
            break;
        }
        sendPositionSetpoint(target_pose);
    }

    void executeLand(const std::shared_ptr<DroneAction> &action) {
        if (!action->isStartPoseInitialized()) {
            action->setStartPose(current_pose_);
            force_status_publish_ = true;
        }

        if (!land_mode_request_sent_) {
            if (!callSetMode("AUTO.LAND")) {
                broadcastStatusThrottled(
                    "等待进入 AUTO.LAND：当前高度（ENU z）=" +
                    formatMeters(current_pose_.pose.position.z) + " m。");
                sendPositionSetpoint(current_pose_);
                return;
            }
            land_mode_request_sent_ = true;
            force_status_publish_ = true;
            RCLCPP_INFO(node_->get_logger(), "降落模式切换请求已发送。");
        }

        const double current_z = current_pose_.pose.position.z;
        const double start_z = action->getStartPose().pose.position.z;
        const double descended = std::max(0.0, start_z - current_z);
        constexpr double kLandCompleteAltitudeM = 0.15;
        constexpr int kLandCompleteStableCycles = 20;

        if (current_z <= kLandCompleteAltitudeM) {
            ++land_low_altitude_count_;
        } else {
            land_low_altitude_count_ = 0;
        }

        if (!current_state_.armed ||
            land_low_altitude_count_ >= kLandCompleteStableCycles) {
            const std::string reason =
                !current_state_.armed ? "飞控已上锁" : "高度达到近地阈值";
            completeCurrentAction(
                "降落动作已完成（" + reason + "）：当前高度（ENU z）=" +
                formatMeters(current_z) + " m，相对降落开始已下降=" +
                formatMeters(descended) + " m。");
            return;
        }

        broadcastStatusThrottled(
            "降落中：当前高度（ENU z）=" + formatMeters(current_z) +
            " m，相对降落开始已下降=" + formatMeters(descended) +
            " m，飞控模式=" + current_state_.mode + "。");
    }

    void executeTakeoff(const std::shared_ptr<DroneAction> &action) {
        geometry_msgs::msg::PoseStamped takeoff_pose;
        if (action->isStartPoseInitialized()) {
            takeoff_pose = action->getStartPose();
        } else {
            action->setStartPose(current_pose_);
            takeoff_pose = current_pose_;
        }

        takeoff_pose.header.frame_id = "world_enu";
        takeoff_pose.header.stamp = node_->now();
        const double target_z =
            action->getStartPose().pose.position.z + action->getTargetAltitude();
        takeoff_pose.pose.position.z = target_z;

        const SpatialPoint current(current_pose_);
        const SpatialPoint target(takeoff_pose);
        if (std::abs(current.z - target.z) < action->getPositionTolerance()){
            completeCurrentAction("起飞动作已完成。");
            return;
        }
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "正在执行起飞：当前 ENU z %.2f m，目标 ENU z %.2f m，相对起飞高度 %.2f m。",
                             current.z, target.z, action->getTargetAltitude());
        sendPositionSetpoint(takeoff_pose);
        
    }

    bool callSetMode(const std::string &mode) {
        if (!set_mode_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(node_->get_logger(), "飞行模式切换服务不可用。");
            return false;
        }

        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = mode;
        auto future = set_mode_client_->async_send_request(request);
        const auto result = rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(2));
        if (result != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_WARN(node_->get_logger(), "飞行模式切换请求超时，目标模式：%s。", mode.c_str());
            return false;
        }
        const bool mode_sent = future.get()->mode_sent;
        if (!mode_sent) {
            RCLCPP_WARN(node_->get_logger(), "飞控拒绝切换飞行模式，目标模式：%s。", mode.c_str());
        }
        return mode_sent;
    }

    void state_callback(mavros_msgs::msg::State::SharedPtr msg){
        current_state_ = *msg;
    }

    void pose_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_pose_ = *msg;
        current_pose_.header.frame_id = "world_enu";
        current_pose_received_ = true;
        if(!last_finish_pose_initialized_){
            last_finish_pose_ = current_pose_;
            last_finish_pose_initialized_ = true;
        }
    }

    void cameraAim_callback(geometry_msgs::msg::Point::SharedPtr msg){
        camera_aim_diff_ = *msg;
        last_camera_aim_time_ = node_->now();
    }

    std::string actionTypeToString(ActionType type) const {
    switch (type) {
        case ActionType::MOVE_TO_POSITION:
            return "move";
        case ActionType::HOVER:
            return "hover";
        case ActionType::CAMERA_AIM:
            return "camera_aim";
        case ActionType::LAND:
            return "land";
        case ActionType::TAKEOFF:
            return "takeoff";
        default:
            return "unknown";
    }
}

    rclcpp::Node::SharedPtr node_;
    tf2_ros::Buffer &tf_buffer_;

    Vec3dPID pid_cam_aim_{0.001, 0.0, 0.001};

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr step_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_status_pub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr camera_aim_sub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;

    mavros_msgs::msg::State current_state_;
    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::Point camera_aim_diff_;
    rclcpp::Time last_camera_aim_time_;
    bool current_pose_received_ = false;
    rclcpp::Time last_status_publish_time_;
    bool force_status_publish_ = true;
    double status_publish_period_s_ = 1.0;
    
    int aim_close_count_  = 0;
    bool land_mode_request_sent_ = false;
    int land_low_altitude_count_ = 0;

    std::queue<std::shared_ptr<DroneAction>> action_queue_;
    std::shared_ptr<DroneAction> current_action_;

    geometry_msgs::msg::PoseStamped last_finish_pose_;
    bool last_finish_pose_initialized_ = false;

    int action_id_ = 0;
};
