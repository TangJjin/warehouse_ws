#include "drone_qt_2/mission_yaml_builder.hpp"

#include <QTextStream>
#include <QtMath>

namespace {
bool almostEqual(double a, double b, double eps = 1e-6)
{
    return qAbs(a - b) < eps;
}

struct MissionMoveStep
{
    double x;
    double y;
    double z;
    double yaw;
    bool final_waypoint = false;
    bool vision_hover = false;
};

void appendMoveIfChanged(std::vector<MissionMoveStep> &steps,
                         AirborneWorldCoord &current,
                         const AirborneWorldCoord &target,
                         bool final_waypoint = false,
                         bool vision_hover = false)
{
    if (almostEqual(current.x, target.x) &&
        almostEqual(current.y, target.y) &&
        almostEqual(current.z, target.z) &&
        almostEqual(current.yaw, target.yaw)) {
        return;
    }

    steps.push_back(MissionMoveStep{target.x, target.y, target.z, target.yaw, final_waypoint, vision_hover});
    current = target;
}

std::vector<MissionMoveStep> expandMissionSteps(const std::vector<AirborneWorldCoord> &points)
{
    std::vector<MissionMoveStep> steps;
    if (points.empty()) {
        return steps;
    }

    AirborneWorldCoord current{0.0, 0.0, 0.0, 0.0};

    for (const auto &next : points) {
        if (!almostEqual(current.y, next.y)) {
            appendMoveIfChanged(steps, current, AirborneWorldCoord{0.0, current.y, current.z, current.yaw});
            appendMoveIfChanged(steps, current, AirborneWorldCoord{0.0, next.y, current.z, current.yaw});
        }

        appendMoveIfChanged(steps, current, AirborneWorldCoord{current.x, current.y, next.z, current.yaw});
        appendMoveIfChanged(steps, current, AirborneWorldCoord{current.x, current.y, current.z, next.yaw});
        appendMoveIfChanged(steps, current, AirborneWorldCoord{next.x, next.y, next.z, next.yaw}, true, next.x > 0.0);
    }

    appendMoveIfChanged(steps, current, AirborneWorldCoord{0.0, current.y, current.z, current.yaw});
    appendMoveIfChanged(steps, current, AirborneWorldCoord{0.0, 0.0, current.z, current.yaw});
    appendMoveIfChanged(steps, current, AirborneWorldCoord{0.0, 0.0, current.z, 0.0});

    if (almostEqual(current.z, 1.5)) {
        appendMoveIfChanged(steps, current, AirborneWorldCoord{0.0, 0.0, 0.70, 0.0});
    }
    appendMoveIfChanged(steps, current, AirborneWorldCoord{0.0, 0.0, 0.0, 0.0});

    return steps;
}

void writeMoveStep(QTextStream &out, const QString &frame, const MissionMoveStep &step, double tolerance)
{
    out << "    - type: \"move\"\n";
    out << "      frame: \"" << frame << "\"\n";
    out << "      position: [" << step.x << ", " << step.y << ", " << step.z << "]\n";
    out << "      yaw: " << step.yaw << "\n";
    out << "      tolerance: " << tolerance << "\n";
    out << "      yaw_tolerance_deg: 4.0\n";
    out << "      max_xy_speed_mps: 0.50\n";
    out << "      max_z_speed_mps: 0.30\n";
    out << "      max_yaw_rate_deg_s: 40.0\n\n";
}

void writeMoveHover(QTextStream &out, double duration, bool vision_hover)
{
    out << "    - type: \"hover\"\n";
    out << "      duration: " << duration << "\n";
    if (vision_hover) {
        out << "      vision_hover: true\n";
    }
    out << "\n";
}
}

std::vector<AirborneWorldCoord> AirborneMissionYamlBuilder::compressStraightSegments(const std::vector<AirborneWorldCoord> &points)
{
    if (points.size() <= 2) {
        return points;
    }

    std::vector<AirborneWorldCoord> result;
    result.reserve(points.size());
    result.push_back(points.front());

    for (std::size_t i = 1; i + 1 < points.size(); ++i) {
        const auto &prev = points[i - 1];
        const auto &curr = points[i];
        const auto &next = points[i + 1];

        const double dx1 = curr.x - prev.x;
        const double dy1 = curr.y - prev.y;
        const double dx2 = next.x - curr.x;
        const double dy2 = next.y - curr.y;

        const bool same_vertical = almostEqual(dx1, 0.0) && almostEqual(dx2, 0.0);
        const bool same_horizontal = almostEqual(dy1, 0.0) && almostEqual(dy2, 0.0);

        if (same_vertical || same_horizontal) {
            continue;
        }

        result.push_back(curr);
    }

    result.push_back(points.back());
    return result;
}

AirborneMissionYamlBuilder::Options AirborneMissionYamlBuilder::fromMissionSummary(const drone_msgs::msg::MissionSummary &summary)
{
    Options options;
    options.takeoff_altitude = summary.takeoff_altitude;
    options.move_altitude = summary.move_altitude;
    options.start_altitude = summary.start_altitude;
    options.yaw = summary.yaw;
    options.tolerance = summary.tolerance;
    options.takeoff_hover_duration = summary.takeoff_hover_duration;
    options.landing_hover_duration = summary.landing_hover_duration;
    options.move_hover_duration = summary.move_hover_duration;
    options.add_hover_between_takeoff = summary.add_hover_between_takeoff;
    options.add_hover_between_landing = summary.add_hover_between_landing;
    options.add_hover_between_moves = summary.add_hover_between_moves;
    options.use_camera_aim = summary.use_camera_aim;
    options.auto_start_mission = summary.auto_start_mission;
    options.compress_straight_segments = summary.compress_straight_segments;
    options.frame = summary.frame;

    options.cam_tolerance = summary.cam_tolerance;
    options.camera_aim_pid_p = summary.camera_aim_pid_p;
    options.camera_aim_pid_i = summary.camera_aim_pid_i;
    options.camera_aim_pid_d = summary.camera_aim_pid_d;
    options.camera_aim_target_timeout_s = summary.camera_aim_target_timeout_s;
    options.camera_aim_stable_cycles = summary.camera_aim_stable_cycles;
    options.camera_aim_max_step = summary.camera_aim_max_step;
    options.camera_aim_wait_first_targets_timeout_s = summary.camera_aim_wait_first_targets_timeout_s;
    options.camera_aim_no_target_confirm_s = summary.camera_aim_no_target_confirm_s;
    options.camera_aim_record_result_timeout_s = summary.camera_aim_record_result_timeout_s;
    options.camera_aim_scan_point_timeout_s = summary.camera_aim_scan_point_timeout_s;
    return options;
}

uint32_t AirborneMissionYamlBuilder::countMissionActions(const std::vector<AirborneWorldCoord> &points, const Options &options)
{
    if (points.empty()) {
        return 0;
    }

    const auto steps = expandMissionSteps(points);

    uint32_t count = 0;
    count += 1;
    if (options.add_hover_between_takeoff) {
        count += 1;
    }
    for (const auto &step : steps) {
        Q_UNUSED(step);
        count += 1;
        if (options.add_hover_between_moves) {
            count += 1;
        }
        if (options.use_camera_aim && step.final_waypoint) {
            count += 1;
        }
    }
    if (options.add_hover_between_landing) {
        count += 1;
    }
    count += 1;
    return count;
}

QString AirborneMissionYamlBuilder::buildMissionYaml(const std::vector<AirborneWorldCoord> &points, const Options &options)
{
    // 没有任何航点时，直接返回空 yaml，避免生成无效任务。
    if (points.empty()) {
        return QString();
    }

    // 先把“航点之间怎么拆 move”的规则展开成一串实际动作步骤。
    // 这里的 steps 已经包含：
    // 1. y 变化时先去 x=0、再走 y
    // 2. 同 y 时先改高度、再改 yaw、最后走 x
    // 3. 所有航点结束后的回收动作
    const auto steps = expandMissionSteps(points);

    // 开始拼接最终的 mission yaml 文本。
    QString yaml;
    QTextStream out(&yaml);
    out.setRealNumberNotation(QTextStream::FixedNotation);
    out.setRealNumberPrecision(2);

    // 任务头：mission 基本结构和起飞动作，这一段按你的要求保持原样。
    out << "# Mission configuration\n";
    out << "mission:\n";
    out << "  takeoff_altitude: " << options.takeoff_altitude << "\n";
    out << "  actions:\n";
    out << "    - type: \"takeoff\"\n";
    out << "      altitude: " << options.start_altitude << "\n\n";

    // 起飞后的悬停，这一段也按你的要求保持原样。
    if (options.add_hover_between_takeoff) {
        out << "    - type: \"hover\"\n";
        out << "      duration: " << options.takeoff_hover_duration << "\n\n";
    }

    const QString frame = QString::fromStdString(options.frame);
    for (const auto &step : steps) {
        // 输出一个 move 动作。
        // 每个 move 已经是按你的规则拆好的最终步骤，不再在这里额外判断。
        writeMoveStep(out, frame, step, options.tolerance);

        // 如果配置允许，每个 move 后面都跟一个 hover。
        // 其中只有“最终到达原始航点且 x>0”的 hover 会带 vision_hover: true。
        if (options.add_hover_between_moves) {
            writeMoveHover(out, options.move_hover_duration, step.final_waypoint && step.vision_hover);
        }

        // camera_aim 只挂在“最终到达原始航点”的动作后面，
        // 中间过渡点（原地改高、原地转向、去 x=0、沿 y 过渡）不会插这个动作。
        if (options.use_camera_aim && step.final_waypoint) {
            out << "    - type: \"camera_aim\"\n";
            out << "      frame: \"" << frame << "\"\n";
            out << "      position: [" << step.x << ", " << step.y << ", " << step.z << "]\n";
            out << "      axis: \"z\"\n";
            out << "      tolerance: " << options.cam_tolerance << "\n\n";
        }
    }

    // 降落前悬停，这一段保持原有逻辑不变。
    if (options.add_hover_between_landing) {
        out << "    - type: \"hover\"\n";
        out << "      duration: " << options.landing_hover_duration << "\n\n";
    }

    // land 和 system 整段保持不动。
    out << "    - type: \"land\"\n\n";
    out << "system:\n";
    out << "  use_camera_aim: " << (options.use_camera_aim ? "true" : "false") << "\n";

    out << "  camera_aim_pid_p: " << options.camera_aim_pid_p << "\n";
    out << "  camera_aim_pid_i: " << options.camera_aim_pid_i << "\n";
    out << "  camera_aim_pid_d: " << options.camera_aim_pid_d << "\n";
    out << "  camera_aim_target_timeout_s: " << options.camera_aim_target_timeout_s << "\n";
    out << "  camera_aim_stable_cycles: " << options.camera_aim_stable_cycles << "\n";
    out << "  camera_aim_max_step: " << options.camera_aim_max_step << "\n";
    out << "  camera_aim_wait_first_targets_timeout_s: " << options.camera_aim_wait_first_targets_timeout_s << "\n";
    out << "  camera_aim_no_target_confirm_s: " << options.camera_aim_no_target_confirm_s << "\n";
    out << "  camera_aim_record_result_timeout_s: " << options.camera_aim_record_result_timeout_s << "\n";
    out << "  camera_aim_scan_point_timeout_s: " << options.camera_aim_scan_point_timeout_s << "\n";

    out << "  auto_start_mission: " << (options.auto_start_mission ? "true" : "false") << "\n";

    return yaml;
}
