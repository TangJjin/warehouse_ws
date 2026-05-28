#pragma once

#include <QString>
#include <QVector>
#include "drone_qt/position_view_widget.hpp"

class MissionYamlBuilder
{
public:
    struct Options
    {
        double takeoff_altitude{1.2};//起飞高度
        double move_altitude{1.2};//移动高度
        double start_altitude{0.0};//解锁高度
        double yaw{0.0};//偏航角
        double tolerance{0.12};//误差容忍
        double takeoff_hover_duration{5.0};//起飞悬停时长
        double landing_hover_duration{5.0};//降落悬停时长
        double move_hover_duration{5.0};//移动悬停时长
        bool add_hover_between_takeoff{true};//是否在起飞后添加悬停
        bool add_hover_between_landing{false};//是否在降落前添加悬停
        bool add_hover_between_moves{true};//是否在移动之间添加悬停
        bool use_camera_aim{false};//是否开启相机
        bool auto_start_mission{false};//是否自动启动人物
        bool compress_straight_segments{false};//是否压缩直线段
        QString frame{"world_body"};
    };
    //长直线去掉中间点的函数
    static QVector<WorldCoord> compressStraightSegments(const QVector<WorldCoord> &points);
    //yaml导出函数
    static QString buildMissionYaml(const QVector<WorldCoord> &points, const Options &options);
};
