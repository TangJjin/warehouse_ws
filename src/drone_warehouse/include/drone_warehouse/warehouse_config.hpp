#pragma once

#include <QColor>
#include <QRectF>
#include <QSerialPort>
#include <QString>
#include <QVector>

// 判断无人机当前位姿属于货架哪一面的区域。
struct ShelfPoseRegionConfig
{
    QString side;          // 货架面："front" 或 "back"。
    double x_min = 0.0;    // 进入该货架面的最小 X 坐标。
    double x_max = 0.0;    // 进入该货架面的最大 X 坐标。
    double yaw_min = 0.0;  // 进入该货架面的最小航向角，单位：度。
    double yaw_max = 0.0;  // 进入该货架面的最大航向角，单位：度。
};

// 单个货架的低频可调配置。
struct ShelfConfig
{
    QString code;                 // 场景货架编号，例如 "A01"。
    QString display_name;         // 地面站显示名称，例如“货架1”。
    QString front_slot_prefix;    // 正面位置码前缀，例如 "A"。
    QString back_slot_prefix;     // 背面位置码前缀，例如 "B"。

    QRectF base_rect;              // 场景货架底面：X、Y、宽度、长度。
    double height = 0.0;           // 场景货架高度，使用场景坐标单位。
    QColor scene_color;            // 场景绘制颜色。
    QString button_status_color;   // 货架按钮默认状态颜色。

    double front_waypoint_y_m = 0.0; // 正面航点 Y 坐标，单位：米。
    double back_waypoint_y_m = 0.0;  // 背面航点 Y 坐标，单位：米。

    QVector<ShelfPoseRegionConfig> pose_regions; // 该货架的位姿识别区域。
};

// 所有货架共用的槽位配置。
struct SlotGridConfig
{
    int rows = 0;       // 每一面的槽位行数。
    int columns = 0;    // 每一面的槽位列数。

    QVector<double> waypoint_row_z_m;    // 每一行航点高度，单位：米。
    QVector<double> waypoint_front_x_m;  // 正面每一列航点 X，单位：米。
    QVector<double> waypoint_back_x_m;   // 背面每一列航点 X，单位：米。

    double front_yaw_rad = 0.0; // 正面航点航向，单位：弧度。
    double back_yaw_rad = 0.0;  // 背面航点航向，单位：弧度。

    double pose_y_min = 0.0; // 位姿映射到槽位列时，Y 坐标最小值。
    double pose_y_max = 0.0; // 位姿映射到槽位列时，Y 坐标最大值。
    double pose_z_min = 0.0; // 位姿映射到槽位行时，Z 坐标最小值。
    double pose_z_max = 0.0; // 位姿映射到槽位行时，Z 坐标最大值。

    int slotCountPerSide() const
    {
        return rows * columns;
    }
};

// 一组 ROS 节点、话题和服务名称，可分别用于地面站或串口桥接进程。
struct RosTopicConfig
{
    QString node_name;                // 地面站 ROS 节点名。
    QString drone_status;             // 无人机状态话题。
    QString task_status;              // 任务执行状态话题。
    QString path_ready;               // 航线准备完成话题。
    QString return_world_group;       // 返回规划航线话题。
    QString barcode_capture;          // 带图片的扫码结果话题。
    QString vision_barcode;           // 无线链路转发的扫码结果话题。
    QString local_position;           // 无人机本地位姿话题。
    QString pose_delta;               // 位姿和航向误差话题。

    QString start_task_service;       // 启动任务服务。
    QString stop_push_service;        // 停止任务服务。
    QString start_offboard_service;   // 启动 Offboard 服务。
    QString upload_mission_service;   // 上传任务参数和航点服务。
};

// ground_link_bridge 使用的通信串口配置；扫码串口不在这里管理。
struct SerialPortConfig
{
    QString port_name; // 串口设备路径，例如 /dev/serial/by-id/...。
    qint32 baud_rate = QSerialPort::Baud115200; // 波特率。
    QSerialPort::DataBits data_bits = QSerialPort::Data8; // 数据位。
    QSerialPort::Parity parity = QSerialPort::NoParity; // 校验位。
    QSerialPort::StopBits stop_bits = QSerialPort::OneStop; // 停止位。
    QSerialPort::FlowControl flow_control = QSerialPort::NoFlowControl; // 流控方式。
};

// 地面站启动时选择的通信链路。
enum class ConnectionMode
{
    Wifi,      // 通过 WiFi 直连无人机 ROS 话题。
    Telemetry  // 通过 ground_link_bridge 数传串口转发。
};

// 连接方式和数传串口放在同一个结构中，方便界面整体保存。
struct ConnectionConfig
{
    ConnectionMode mode = ConnectionMode::Wifi; // 当前保存的连接方式。
    SerialPortConfig telemetry_serial;          // 数传连接使用的通信串口。
};

// 上传给无人机的任务飞行和相机对准参数。
struct MissionConfig
{
    double takeoff_altitude = 0.0; // 起飞高度，单位：米。
    double move_altitude = 0.0; // 移动高度，单位：米。
    double start_altitude = 0.0; // 解锁/任务起始高度，单位：米。
    double yaw = 0.0; // 默认任务航向。
    double tolerance = 0.0; // 航点到达误差容忍值。

    double takeoff_hover_duration = 0.0; // 起飞后悬停时间，单位：秒。
    double landing_hover_duration = 0.0; // 降落前悬停时间，单位：秒。
    double move_hover_duration = 0.0; // 移动点之间悬停时间，单位：秒。
    bool add_hover_between_takeoff = false; // 是否在起飞阶段加入悬停。
    bool add_hover_between_landing = false; // 是否在降落阶段加入悬停。
    bool add_hover_between_moves = false; // 是否在移动点之间加入悬停。

    bool use_camera_aim = false; // 是否启用相机对准。
    bool auto_start_mission = false; // 上传完成后是否自动开始任务。
    bool compress_waypoint_segments = true; // 航点飞行是否压缩连续直线段。
    bool compress_non_waypoint_segments = false; // 非航点任务是否压缩连续直线段。
    QString frame; // 任务航点使用的坐标系名称。

    double cam_tolerance = 0.0; // 相机对准允许误差。
    double camera_aim_pid_p = 0.0; // 相机对准 PID 比例系数。
    double camera_aim_pid_i = 0.0; // 相机对准 PID 积分系数。
    double camera_aim_pid_d = 0.0; // 相机对准 PID 微分系数。
    double camera_aim_target_timeout_s = 0.0; // 单次目标等待超时，单位：秒。
    quint16 camera_aim_stable_cycles = 0; // 判定稳定所需连续周期数。
    double camera_aim_max_step = 0.0; // 单次相机对准最大调整量。
    double camera_aim_wait_first_targets_timeout_s = 0.0; // 首批目标等待超时，单位：秒。
    double camera_aim_no_target_confirm_s = 0.0; // 无目标确认时间，单位：秒。
    double camera_aim_record_result_timeout_s = 0.0; // 记录结果超时，单位：秒。
    double camera_aim_scan_point_timeout_s = 0.0; // 单个扫描点总超时，单位：秒。
};

// 地面站全部仓库配置。
struct WarehouseConfig
{
    QVector<ShelfConfig> shelves; // 所有货架配置。
    SlotGridConfig slots;         // 共用槽位结构和航点映射。
    RosTopicConfig ros;           // warehouse_gcs 直连 WiFi 时使用的 ROS 接口。
    RosTopicConfig bridge_ros;    // ground_link_bridge 对外提供的串口转发 ROS 接口。
    ConnectionConfig connection;   // 当前连接方式和数传串口参数。
    MissionConfig mission;        // 上传任务时使用的飞行和相机参数。
};
// 创建首次运行使用的默认仓库配置；warehouse_config.json 不存在时以此生成文件。
WarehouseConfig createDefaultWarehouseConfig();

// 校验货架、槽位、ROS、通信串口和任务参数；配置有效时返回空字符串。
QString validateWarehouseConfig(const WarehouseConfig &config);

// 保存连接方式前更新 config.ros；bridge_ros 始终保持不变。
void applyConnectionModeToRosConfig(
    WarehouseConfig &config,
    ConnectionMode mode);

// 两个程序共用的数据目录。可通过 WAREHOUSE_GCS_DATA_DIR 环境变量覆盖。
QString warehouseDataDirectory();

// 仓库配置文件和槽位运行数据文件的完整路径。
QString warehouseConfigFilePath();
QString shelfPanelDataFilePath();

// 将仓库配置原子写入 JSON 文件。
bool saveWarehouseConfig(const WarehouseConfig &config,
                         QString *error_message = nullptr);

// 启动时读取 JSON；文件不存在时会用代码默认值创建一份。
bool loadWarehouseConfig(WarehouseConfig &config,
                         QString *error_message = nullptr);
