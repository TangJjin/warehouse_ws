#include "drone_qt_2/mission_yaml_builder.hpp"

#include <QTextStream>
#include <QtMath>

namespace {
bool almostEqual(double a, double b, double eps = 1e-6)
{
    return qAbs(a - b) < eps;
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
    std::vector<AirborneWorldCoord> final_points = points;
    if (options.compress_straight_segments) {
        final_points = compressStraightSegments(points);
    }

    uint32_t count = 0;
    count += 1;
    if (options.add_hover_between_takeoff) {
        count += 1;
    }
    for (std::size_t i = 0; i < final_points.size(); ++i) {
        count += 1;
        if (options.add_hover_between_moves) {
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
    std::vector<AirborneWorldCoord> final_points = points;
    if (options.compress_straight_segments) {
        final_points = compressStraightSegments(points);
    }

    QString yaml;
    QTextStream out(&yaml);
    out.setRealNumberNotation(QTextStream::FixedNotation);
    out.setRealNumberPrecision(2);

    if (final_points.size() < 2) {
        return;
    }

    out << "# Mission configuration\n";
    out << "mission:\n";
    out << "  takeoff_altitude: " << options.takeoff_altitude << "\n";
    out << "  actions:\n";
    out << "    - type: \"takeoff\"\n";
    out << "      altitude: " << options.start_altitude << "\n\n";

    if (options.add_hover_between_takeoff) {
        out << "    - type: \"hover\"\n";
        out << "      duration: " << options.takeoff_hover_duration << "\n\n";
    }

    for (size_t i = 0; i < final_points.size() - 1; ++i) {
        const auto &point = final_points[i];

        out << "    - type: \"move\"\n";
        out << "      frame: \"" << QString::fromStdString(options.frame) << "\"\n";
        out << "      position: [" << point.x << ", " << point.y << ", " << options.move_altitude << "]\n";
        out << "      yaw: " << options.yaw << "\n";
        out << "      tolerance: " << options.tolerance << "\n\n";

        if (options.add_hover_between_moves) {
            out << "    - type: \"hover\"\n";
            out << "      duration: " << options.move_hover_duration << "\n\n";
        }

        out << "    - type: \"camera_aim\"\n";
        out << "      frame: \"" << QString::fromStdString(options.frame) << "\"\n";
        out << "      position: [" << point.x << ", " << point.y << ", " << options.move_altitude << "]\n";
        out << "      axis: \"z\"\n";
        out << "      tolerance: " << options.cam_tolerance << "\n\n";
    }

    out << "    - type: \"move\"\n";
    out << "      frame: \"" << QString::fromStdString(options.frame) << "\"\n";
    out << "      position: [0.00, 0.50, 0.50]\n";
    out << "      yaw: " << options.yaw << "\n";
    out << "      tolerance: " << options.tolerance << "\n\n";

    out << "    - type: \"hover\"\n";
    out << "      duration: " << options.move_hover_duration << "\n\n";

    out << "    - type: \"move\"\n";
    out << "      frame: \"" << QString::fromStdString(options.frame) << "\"\n";
    out << "      position: [0.00, 0.00, 0.00]\n";
    out << "      yaw: " << options.yaw << "\n";
    out << "      tolerance: " << options.tolerance << "\n\n";

    if (options.add_hover_between_landing) {
        out << "    - type: \"hover\"\n";
        out << "      duration: " << options.landing_hover_duration << "\n\n";
    }

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