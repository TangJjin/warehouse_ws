#include "drone_qt/mission_yaml_builder.hpp"

#include <QTextStream>
#include <QtMath>


namespace {
    bool almostEqual(double a, double b, double eps = 1e-6)
    {
        return qAbs(a - b) < eps;
    }
}

QVector<WorldCoord> MissionYamlBuilder::compressStraightSegments(const QVector<WorldCoord> &points)
{
    //如果点数小于等于2，直接返回原始点集
    if (points.size() <= 2) {
        return points;
    }

    //遍历点集，检查每个点是否与前后点在同一直线上，如果是，则跳过该点
    QVector<WorldCoord> result;
    result.push_back(points.front());

    for (int i = 1; i < points.size() - 1; ++i) {
        const WorldCoord &prev = points[i - 1];
        const WorldCoord &curr = points[i];
        const WorldCoord &next = points[i + 1];

        //计算前后点的向量，检查它们是否在同一直线上
        const double dx1 = curr.x - prev.x;
        const double dy1 = curr.y - prev.y;
        const double dx2 = next.x - curr.x;
        const double dy2 = next.y - curr.y;

        //如果两个向量的x或y分量都接近0，说明它们在同一直线上，可以跳过当前点
        const bool same_vertical = almostEqual(dx1, 0.0) && almostEqual(dx2, 0.0);
        const bool same_horizontal = almostEqual(dy1, 0.0) && almostEqual(dy2, 0.0);

        if (same_vertical || same_horizontal) {
            //当前点在前后点的连线上，可以跳过
            continue;
        }

        //当前点不在前后点的连线上，保留该点
        result.push_back(curr);
    }

    //最后一个点总是保留
    result.push_back(points.back());
    return result;
}

QString MissionYamlBuilder::buildMissionYaml(const QVector<WorldCoord> &points, const Options &options)
{
    //如果启用了压缩直线段选项，先对点集进行压缩处理
    QVector<WorldCoord> final_points = points;
    if (options.compress_straight_segments) {
        //压缩直线段，去掉中间点
        final_points = compressStraightSegments(points);
    }

    QString yaml;
    QTextStream out(&yaml);
    //设置输出格式，保留两位小数
    out.setRealNumberNotation(QTextStream::FixedNotation);
    out.setRealNumberPrecision(2);

    //初始yaml内容
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

    //遍历点集，生成移动和悬停的yaml内容
    for (const auto &point : final_points) {
        out << "    - type: \"move\"\n";
        out << "      frame: \"" << options.frame << "\"\n";
        out << "      position: ["
            << point.x << ", "
            << point.y << ", "
            << options.move_altitude << "]\n";
        out << "      yaw: " << options.yaw << "\n";
        out << "      tolerance: " << options.tolerance << "\n\n";

        if (options.add_hover_between_moves) {
            out << "    - type: \"hover\"\n";
            out << "      duration: " << options.move_hover_duration << "\n\n";
        }
    }

    //最后添加降落指令
    out << "    - type: \"land\"\n\n";
    out << "system:\n";
    out << "  use_camera_aim: " << (options.use_camera_aim ? "true" : "false") << "\n";
    out << "  auto_start_mission: " << (options.auto_start_mission ? "true" : "false") << "\n";

    return yaml;
}