#include "drone_warehouse/warehouse_config.hpp"

#include "drone_warehouse/color_palette.hpp"

#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>
#include <QSet>
#include <QStringList>

#include <cmath>
#include <limits>

namespace
{
QString validateRosConfig(const RosTopicConfig &config, const QString &owner)
{
    const QStringList required_values = {
        config.node_name,
        config.drone_status,
        config.task_status,
        config.path_ready,
        config.return_world_group,
        config.barcode_capture,
        config.vision_barcode,
        config.local_position,
        config.pose_delta,
        config.start_task_service,
        config.stop_push_service,
        config.start_offboard_service,
        config.upload_mission_service
    };

    for (const QString &value : required_values)
    {
        if (value.trimmed().isEmpty())
        {
            return owner + " has an empty ROS node, topic, or service name";
        }
    }
    return {};
}
}

WarehouseConfig createDefaultWarehouseConfig()
{
    WarehouseConfig config;

    // 每个货架面的槽位数量等于 rows * columns。
    config.slot_grid.rows = 4;
    config.slot_grid.columns = 3;

    // 航点坐标单位为米；数组顺序必须与槽位行号或列号一致。
    config.slot_grid.waypoint_row_z_m = {1.50, 1.10, 0.70, 0.30};
    config.slot_grid.waypoint_front_x_m = {0.75, 1.25, 1.75};
    config.slot_grid.waypoint_back_x_m = {1.75, 1.25, 0.75};
    config.slot_grid.front_yaw_rad = 1.57;
    config.slot_grid.back_yaw_rad = 4.71;

    // 将无人机原始位姿映射到槽位行列时使用的坐标范围。
    config.slot_grid.pose_y_min = -100.0;
    config.slot_grid.pose_y_max = 50.0;
    config.slot_grid.pose_z_min = 0.0;
    config.slot_grid.pose_z_max = 160.0;

    ShelfConfig shelf1;
    shelf1.code = "A01";                          // 场景货架编号。
    shelf1.display_name = "A01";                  // 地面站显示名称。
    shelf1.front_slot_prefix = "A";               // 例如 A-0-0。
    shelf1.back_slot_prefix = "B";                // 例如 B-0-0。
    shelf1.base_rect = QRectF(-90, -100, 30, 150);// X、Y、宽度、长度。
    shelf1.height = 160.0;                        // 使用场景坐标单位。
    shelf1.scene_color =
        ColorPalette::withAlpha(ColorPalette::BlueGrayDark, 180);
    shelf1.button_status_color = "#7f8c9a";       // 默认状态颜色。
    shelf1.front_waypoint_y_m = 0.0;              // 正面航点 Y，单位：米。
    shelf1.back_waypoint_y_m = 1.5;               // 背面航点 Y，单位：米。
    shelf1.pose_regions = {
        {"front", 110.0, 140.0, 80.0, 100.0},
        {"back", -10.0, 10.0, -100.0, -80.0}
    };

    ShelfConfig shelf2;
    shelf2.code = "A02";
    shelf2.display_name = "A02";              
    shelf2.front_slot_prefix = "C";
    shelf2.back_slot_prefix = "D";
    shelf2.base_rect = QRectF(60, -100, 30, 150);
    shelf2.height = 160.0;
    shelf2.scene_color =
        ColorPalette::withAlpha(ColorPalette::BlueGrayDark, 180);
    shelf2.button_status_color = "#7f8c9a";
    shelf2.front_waypoint_y_m = 1.5;
    shelf2.back_waypoint_y_m = 3.0;
    shelf2.pose_regions = {
        {"front", -10.0, 10.0, 80.0, 100.0},
        {"back", -140.0, -110.0, -100.0, -80.0}
    };

    config.shelves = {shelf1, shelf2};

    // warehouse_gcs 默认通过 WiFi 直连下列 ROS 接口。
    config.ros.node_name = "ground_qt_node";
    config.ros.drone_status = "/drone/status";
    config.ros.task_status = "/drone/task/status";
    config.ros.path_ready = "/drone/control/path_ready";
    config.ros.return_world_group = "/drone/return/world_group";
    config.ros.barcode_capture = "/drone/barcode_capture";
    config.ros.vision_barcode = "/drone/vision/barcode";
    config.ros.local_position = "/drone/local_position";
    config.ros.pose_delta = "/drone/pose_yaw_compare/delta";
    config.ros.start_task_service = "/drone/start_task";
    config.ros.stop_push_service = "/drone/stop_push";
    config.ros.start_offboard_service = "/drone/start_offboard";
    config.ros.upload_mission_service = "/drone/upload_mission_summary";

    // ground_link_bridge 使用独立的 /serial 命名空间转发串口通信。
    config.bridge_ros.node_name = "ground_link_bridge";
    config.bridge_ros.drone_status = "/serial/drone/status";
    config.bridge_ros.task_status = "/serial/drone/task/status";
    config.bridge_ros.path_ready = "/serial/drone/control/path_ready";
    config.bridge_ros.return_world_group = "/serial/drone/return/world_group";
    config.bridge_ros.barcode_capture = "/drone/barcode_capture";
    config.bridge_ros.vision_barcode = "/serial/drone/vision/barcode";
    config.bridge_ros.local_position = "/serial/drone/local_position";
    config.bridge_ros.pose_delta = "/serial/drone/pose_yaw_compare/delta";
    config.bridge_ros.start_task_service = "/serial/drone/start_task";
    config.bridge_ros.stop_push_service = "/serial/drone/stop_push";
    config.bridge_ros.start_offboard_service = "/serial/drone/start_offboard";
    config.bridge_ros.upload_mission_service = "/serial/drone/upload_mission_summary";

    // 默认使用 WiFi；数传串口保留原 ground_link_bridge 参数。
    // 扫码串口仍由 ShelfInfoDialog 独立管理，不属于这里的连接设置。
    config.connection.mode = ConnectionMode::Wifi;
    config.connection.telemetry_serial.port_name = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0";
    config.connection.telemetry_serial.baud_rate = QSerialPort::Baud115200;
    config.connection.telemetry_serial.data_bits = QSerialPort::Data8;
    config.connection.telemetry_serial.parity = QSerialPort::NoParity;
    config.connection.telemetry_serial.stop_bits = QSerialPort::OneStop;
    config.connection.telemetry_serial.flow_control = QSerialPort::NoFlowControl;

    // 任务飞行基础参数。
    config.mission.takeoff_altitude = 0.0;
    config.mission.move_altitude = 1.2;
    config.mission.start_altitude = 0.0;
    config.mission.yaw = 0.0;
    config.mission.tolerance = 0.10;
    config.mission.takeoff_hover_duration = 0.0;
    config.mission.landing_hover_duration = 1.0;
    config.mission.move_hover_duration = 3.0;
    config.mission.add_hover_between_takeoff = false;
    config.mission.add_hover_between_landing = true;
    config.mission.add_hover_between_moves = true;
    config.mission.use_camera_aim = false;
    config.mission.auto_start_mission = false;
    config.mission.compress_waypoint_segments = true;
    config.mission.compress_non_waypoint_segments = false;
    config.mission.frame = "world_body";

    // 相机对准参数。
    config.mission.cam_tolerance = 10.0;
    config.mission.camera_aim_pid_p = 0.01;
    config.mission.camera_aim_pid_i = 0.00;
    config.mission.camera_aim_pid_d = 0.01;
    config.mission.camera_aim_target_timeout_s = 1.0;
    config.mission.camera_aim_stable_cycles = 15;
    config.mission.camera_aim_max_step = 0.05;
    config.mission.camera_aim_wait_first_targets_timeout_s = 8.0;
    config.mission.camera_aim_no_target_confirm_s = 3.0;
    config.mission.camera_aim_record_result_timeout_s = 10.0;
    config.mission.camera_aim_scan_point_timeout_s = 30.0;

    return config;
}

QString validateWarehouseConfig(const WarehouseConfig &config)
{
    if (config.shelves.isEmpty())
    {
        return "at least one shelf must be configured";
    }
    if (config.slot_grid.rows <= 0 || config.slot_grid.columns <= 0)
    {
        return "slot rows and columns must be greater than zero";
    }
    if (config.slot_grid.waypoint_row_z_m.size() != config.slot_grid.rows)
    {
        return "slot row count does not match waypoint height count";
    }
    if (config.slot_grid.waypoint_front_x_m.size() != config.slot_grid.columns ||
        config.slot_grid.waypoint_back_x_m.size() != config.slot_grid.columns)
    {
        return "slot column count does not match waypoint X count";
    }
    if (config.slot_grid.pose_y_max <= config.slot_grid.pose_y_min ||
        config.slot_grid.pose_z_max <= config.slot_grid.pose_z_min)
    {
        return "slot pose mapping ranges are invalid";
    }

    if (config.connection.mode != ConnectionMode::Wifi &&
        config.connection.mode != ConnectionMode::Telemetry)
    {
        return "connection mode is invalid";
    }

    const SerialPortConfig &telemetry_serial =
        config.connection.telemetry_serial;
    if (telemetry_serial.port_name.trimmed().isEmpty() ||
        telemetry_serial.baud_rate <= 0 ||
        telemetry_serial.data_bits == QSerialPort::UnknownDataBits ||
        telemetry_serial.parity == QSerialPort::UnknownParity ||
        telemetry_serial.stop_bits == QSerialPort::UnknownStopBits ||
        telemetry_serial.flow_control == QSerialPort::UnknownFlowControl)
    {
        return "ground_link_bridge has invalid serial port settings";
    }

    const QVector<double> mission_values = {
        config.mission.takeoff_altitude,
        config.mission.move_altitude,
        config.mission.start_altitude,
        config.mission.yaw,
        config.mission.tolerance,
        config.mission.takeoff_hover_duration,
        config.mission.landing_hover_duration,
        config.mission.move_hover_duration,
        config.mission.cam_tolerance,
        config.mission.camera_aim_pid_p,
        config.mission.camera_aim_pid_i,
        config.mission.camera_aim_pid_d,
        config.mission.camera_aim_target_timeout_s,
        config.mission.camera_aim_max_step,
        config.mission.camera_aim_wait_first_targets_timeout_s,
        config.mission.camera_aim_no_target_confirm_s,
        config.mission.camera_aim_record_result_timeout_s,
        config.mission.camera_aim_scan_point_timeout_s
    };
    for (double value : mission_values)
    {
        if (!std::isfinite(value))
        {
            return "mission config contains a non-finite number";
        }
    }
    if (config.mission.takeoff_altitude < 0.0 ||
        config.mission.move_altitude < 0.0 ||
        config.mission.start_altitude < 0.0 ||
        config.mission.tolerance <= 0.0 ||
        config.mission.takeoff_hover_duration < 0.0 ||
        config.mission.landing_hover_duration < 0.0 ||
        config.mission.move_hover_duration < 0.0 ||
        config.mission.frame.trimmed().isEmpty())
    {
        return "mission flight settings are invalid";
    }
    if (config.mission.use_camera_aim &&
        (config.mission.cam_tolerance <= 0.0 ||
         config.mission.camera_aim_target_timeout_s <= 0.0 ||
         config.mission.camera_aim_stable_cycles == 0 ||
         config.mission.camera_aim_max_step <= 0.0 ||
         config.mission.camera_aim_wait_first_targets_timeout_s <= 0.0 ||
         config.mission.camera_aim_no_target_confirm_s < 0.0 ||
         config.mission.camera_aim_record_result_timeout_s <= 0.0 ||
         config.mission.camera_aim_scan_point_timeout_s <= 0.0))
    {
        return "mission camera-aim settings are invalid";
    }

    QSet<QString> shelf_codes;
    QSet<QString> slot_prefixes;
    for (const ShelfConfig &shelf : config.shelves)
    {
        const QString shelf_code = shelf.code.trimmed().toUpper();
        if (shelf_code.isEmpty() || shelf.display_name.trimmed().isEmpty())
        {
            return "shelf code and display name are required";
        }
        if (shelf_codes.contains(shelf_code))
        {
            return shelf.code + " has a duplicate shelf code";
        }
        shelf_codes.insert(shelf_code);
        if (!shelf.scene_color.isValid() || shelf.height <= 0.0 ||
            shelf.base_rect.width() <= 0.0 || shelf.base_rect.height() <= 0.0)
        {
            return shelf.code + " has invalid geometry, height, or color";
        }

        const QString front_prefix = shelf.front_slot_prefix.trimmed().toUpper();
        const QString back_prefix = shelf.back_slot_prefix.trimmed().toUpper();
        if (front_prefix.size() != 1 || back_prefix.size() != 1 ||
            slot_prefixes.contains(front_prefix) || slot_prefixes.contains(back_prefix) ||
            front_prefix == back_prefix)
        {
            return shelf.code + " has an invalid or duplicate slot prefix";
        }
        slot_prefixes.insert(front_prefix);
        slot_prefixes.insert(back_prefix);

        QSet<QString> pose_sides;
        for (const ShelfPoseRegionConfig &region : shelf.pose_regions)
        {
            if ((region.side != "front" && region.side != "back") ||
                region.x_max < region.x_min || region.yaw_max < region.yaw_min)
            {
                return shelf.code + " has an invalid pose recognition region";
            }
            pose_sides.insert(region.side);
        }
        if (!pose_sides.contains("front") || !pose_sides.contains("back"))
        {
            return shelf.code + " requires front and back pose recognition regions";
        }
    }

    QString error = validateRosConfig(config.ros, "warehouse_gcs");
    if (!error.isEmpty())
    {
        return error;
    }
    return validateRosConfig(config.bridge_ros, "ground_link_bridge");
}

namespace
{
bool jsonError(QString *error_message, const QString &message)
{
    if (error_message)
    {
        *error_message = message;
    }
    return false;
}

bool readObject(const QJsonObject &parent,
                const QString &key,
                QJsonObject &value,
                QString *error_message)
{
    const QJsonValue json_value = parent.value(key);
    if (!json_value.isObject())
    {
        return jsonError(error_message, key + " 必须是 JSON 对象");
    }
    value = json_value.toObject();
    return true;
}

bool readArray(const QJsonObject &parent,
               const QString &key,
               QJsonArray &value,
               QString *error_message)
{
    const QJsonValue json_value = parent.value(key);
    if (!json_value.isArray())
    {
        return jsonError(error_message, key + " 必须是 JSON 数组");
    }
    value = json_value.toArray();
    return true;
}

bool readString(const QJsonObject &object,
                const QString &key,
                QString &value,
                QString *error_message)
{
    const QJsonValue json_value = object.value(key);
    if (!json_value.isString())
    {
        return jsonError(error_message, key + " 必须是字符串");
    }
    value = json_value.toString();
    return true;
}

bool readDouble(const QJsonObject &object,
                const QString &key,
                double &value,
                QString *error_message)
{
    const QJsonValue json_value = object.value(key);
    if (!json_value.isDouble())
    {
        return jsonError(error_message, key + " 必须是数字");
    }
    value = json_value.toDouble();
    return true;
}

bool readInt(const QJsonObject &object,
             const QString &key,
             int &value,
             QString *error_message)
{
    double number = 0.0;
    if (!readDouble(object, key, number, error_message))
    {
        return false;
    }
    if (!std::isfinite(number) || std::floor(number) != number ||
        number < static_cast<double>(std::numeric_limits<int>::min()) ||
        number > static_cast<double>(std::numeric_limits<int>::max()))
    {
        return jsonError(error_message, key + " 必须是整数");
    }
    value = static_cast<int>(number);
    return true;
}

bool readUInt16(const QJsonObject &object,
                const QString &key,
                quint16 &value,
                QString *error_message)
{
    int number = 0;
    if (!readInt(object, key, number, error_message))
    {
        return false;
    }
    if (number < 0 || number > std::numeric_limits<quint16>::max())
    {
        return jsonError(error_message, key + " 超出 0 到 65535 的范围");
    }
    value = static_cast<quint16>(number);
    return true;
}

bool readBool(const QJsonObject &object,
              const QString &key,
              bool &value,
              QString *error_message)
{
    const QJsonValue json_value = object.value(key);
    if (!json_value.isBool())
    {
        return jsonError(error_message, key + " 必须是 true 或 false");
    }
    value = json_value.toBool();
    return true;
}

QJsonArray doubleVectorToJson(const QVector<double> &values)
{
    QJsonArray array;
    for (double value : values)
    {
        array.append(value);
    }
    return array;
}

bool doubleVectorFromJson(const QJsonObject &object,
                          const QString &key,
                          QVector<double> &values,
                          QString *error_message)
{
    QJsonArray array;
    if (!readArray(object, key, array, error_message))
    {
        return false;
    }

    QVector<double> loaded_values;
    loaded_values.reserve(array.size());
    for (int index = 0; index < array.size(); ++index)
    {
        if (!array.at(index).isDouble())
        {
            return jsonError(
                error_message,
                QString("%1[%2] 必须是数字").arg(key).arg(index));
        }
        loaded_values.push_back(array.at(index).toDouble());
    }
    values = loaded_values;
    return true;
}

QJsonObject rosConfigToJson(const RosTopicConfig &config)
{
    QJsonObject object;
    object.insert("node_name", config.node_name);
    object.insert("drone_status", config.drone_status);
    object.insert("task_status", config.task_status);
    object.insert("path_ready", config.path_ready);
    object.insert("return_world_group", config.return_world_group);
    object.insert("barcode_capture", config.barcode_capture);
    object.insert("vision_barcode", config.vision_barcode);
    object.insert("local_position", config.local_position);
    object.insert("pose_delta", config.pose_delta);
    object.insert("start_task_service", config.start_task_service);
    object.insert("stop_push_service", config.stop_push_service);
    object.insert("start_offboard_service", config.start_offboard_service);
    object.insert("upload_mission_service", config.upload_mission_service);
    return object;
}

bool rosConfigFromJson(const QJsonObject &object,
                       RosTopicConfig &config,
                       QString *error_message)
{
    return readString(object, "node_name", config.node_name, error_message) &&
           readString(object, "drone_status", config.drone_status, error_message) &&
           readString(object, "task_status", config.task_status, error_message) &&
           readString(object, "path_ready", config.path_ready, error_message) &&
           readString(object, "return_world_group", config.return_world_group, error_message) &&
           readString(object, "barcode_capture", config.barcode_capture, error_message) &&
           readString(object, "vision_barcode", config.vision_barcode, error_message) &&
           readString(object, "local_position", config.local_position, error_message) &&
           readString(object, "pose_delta", config.pose_delta, error_message) &&
           readString(object, "start_task_service", config.start_task_service, error_message) &&
           readString(object, "stop_push_service", config.stop_push_service, error_message) &&
           readString(object, "start_offboard_service", config.start_offboard_service, error_message) &&
           readString(object, "upload_mission_service", config.upload_mission_service, error_message);
}

QString parityToString(QSerialPort::Parity parity)
{
    switch (parity)
    {
    case QSerialPort::NoParity:
        return "none";
    case QSerialPort::EvenParity:
        return "even";
    case QSerialPort::OddParity:
        return "odd";
    case QSerialPort::SpaceParity:
        return "space";
    case QSerialPort::MarkParity:
        return "mark";
    default:
        return "unknown";
    }
}

bool parityFromString(const QString &text,
                      QSerialPort::Parity &parity,
                      QString *error_message)
{
    const QString value = text.trimmed().toLower();
    if (value == "none")
    {
        parity = QSerialPort::NoParity;
    }
    else if (value == "even")
    {
        parity = QSerialPort::EvenParity;
    }
    else if (value == "odd")
    {
        parity = QSerialPort::OddParity;
    }
    else if (value == "space")
    {
        parity = QSerialPort::SpaceParity;
    }
    else if (value == "mark")
    {
        parity = QSerialPort::MarkParity;
    }
    else
    {
        return jsonError(error_message, "connection.telemetry_serial.parity 取值无效");
    }
    return true;
}

QString flowControlToString(QSerialPort::FlowControl flow_control)
{
    switch (flow_control)
    {
    case QSerialPort::NoFlowControl:
        return "none";
    case QSerialPort::HardwareControl:
        return "hardware";
    case QSerialPort::SoftwareControl:
        return "software";
    default:
        return "unknown";
    }
}

bool flowControlFromString(const QString &text,
                           QSerialPort::FlowControl &flow_control,
                           QString *error_message)
{
    const QString value = text.trimmed().toLower();
    if (value == "none")
    {
        flow_control = QSerialPort::NoFlowControl;
    }
    else if (value == "hardware")
    {
        flow_control = QSerialPort::HardwareControl;
    }
    else if (value == "software")
    {
        flow_control = QSerialPort::SoftwareControl;
    }
    else
    {
        return jsonError(error_message, "connection.telemetry_serial.flow_control 取值无效");
    }
    return true;
}

double stopBitsToJsonNumber(QSerialPort::StopBits stop_bits)
{
    switch (stop_bits)
    {
    case QSerialPort::OneStop:
        return 1.0;
    case QSerialPort::OneAndHalfStop:
        return 1.5;
    case QSerialPort::TwoStop:
        return 2.0;
    default:
        return -1.0;
    }
}

bool stopBitsFromJsonNumber(double value,
                            QSerialPort::StopBits &stop_bits,
                            QString *error_message)
{
    if (value == 1.0)
    {
        stop_bits = QSerialPort::OneStop;
    }
    else if (value == 1.5)
    {
        stop_bits = QSerialPort::OneAndHalfStop;
    }
    else if (value == 2.0)
    {
        stop_bits = QSerialPort::TwoStop;
    }
    else
    {
        return jsonError(error_message, "connection.telemetry_serial.stop_bits 只能是 1、1.5 或 2");
    }
    return true;
}

QJsonObject serialConfigToJson(const SerialPortConfig &config)
{
    QJsonObject object;
    object.insert("port_name", config.port_name);
    object.insert("baud_rate", config.baud_rate);
    object.insert("data_bits", static_cast<int>(config.data_bits));
    object.insert("parity", parityToString(config.parity));
    object.insert("stop_bits", stopBitsToJsonNumber(config.stop_bits));
    object.insert("flow_control", flowControlToString(config.flow_control));
    return object;
}

bool serialConfigFromJson(const QJsonObject &object,
                          SerialPortConfig &config,
                          QString *error_message)
{
    int baud_rate = 0;
    int data_bits = 0;
    QString parity;
    double stop_bits = 0.0;
    QString flow_control;
    if (!readString(object, "port_name", config.port_name, error_message) ||
        !readInt(object, "baud_rate", baud_rate, error_message) ||
        !readInt(object, "data_bits", data_bits, error_message) ||
        !readString(object, "parity", parity, error_message) ||
        !readDouble(object, "stop_bits", stop_bits, error_message) ||
        !readString(object, "flow_control", flow_control, error_message))
    {
        return false;
    }

    config.baud_rate = baud_rate;
    switch (data_bits)
    {
    case 5:
        config.data_bits = QSerialPort::Data5;
        break;
    case 6:
        config.data_bits = QSerialPort::Data6;
        break;
    case 7:
        config.data_bits = QSerialPort::Data7;
        break;
    case 8:
        config.data_bits = QSerialPort::Data8;
        break;
    default:
        return jsonError(error_message, "connection.telemetry_serial.data_bits 只能是 5、6、7 或 8");
    }

    return parityFromString(parity, config.parity, error_message) &&
           stopBitsFromJsonNumber(stop_bits, config.stop_bits, error_message) &&
           flowControlFromString(flow_control, config.flow_control, error_message);
}

QString connectionModeToString(ConnectionMode mode)
{
    switch (mode)
    {
    case ConnectionMode::Wifi:
        return "wifi";
    case ConnectionMode::Telemetry:
        return "telemetry";
    }
    return "unknown";
}

bool connectionModeFromString(const QString &text,
                              ConnectionMode &mode,
                              QString *error_message)
{
    const QString value = text.trimmed().toLower();
    if (value == "wifi")
    {
        mode = ConnectionMode::Wifi;
        return true;
    }
    if (value == "telemetry")
    {
        mode = ConnectionMode::Telemetry;
        return true;
    }
    return jsonError(
        error_message,
        "connection.mode 只能是 wifi 或 telemetry");
}

QJsonObject connectionConfigToJson(const ConnectionConfig &config)
{
    QJsonObject object;
    object.insert("mode", connectionModeToString(config.mode));
    object.insert(
        "telemetry_serial",
        serialConfigToJson(config.telemetry_serial));
    return object;
}

bool connectionConfigFromJson(const QJsonObject &object,
                              ConnectionConfig &config,
                              QString *error_message)
{
    QString mode_text;
    QJsonObject serial_object;
    if (!readString(object, "mode", mode_text, error_message) ||
        !readObject(
            object, "telemetry_serial", serial_object, error_message))
    {
        return false;
    }

    return connectionModeFromString(
               mode_text, config.mode, error_message) &&
           serialConfigFromJson(
               serial_object,
               config.telemetry_serial,
               error_message);
}
QJsonObject missionConfigToJson(const MissionConfig &config)
{
    QJsonObject object;
    object.insert("takeoff_altitude", config.takeoff_altitude);
    object.insert("move_altitude", config.move_altitude);
    object.insert("start_altitude", config.start_altitude);
    object.insert("yaw", config.yaw);
    object.insert("tolerance", config.tolerance);
    object.insert("takeoff_hover_duration", config.takeoff_hover_duration);
    object.insert("landing_hover_duration", config.landing_hover_duration);
    object.insert("move_hover_duration", config.move_hover_duration);
    object.insert("add_hover_between_takeoff", config.add_hover_between_takeoff);
    object.insert("add_hover_between_landing", config.add_hover_between_landing);
    object.insert("add_hover_between_moves", config.add_hover_between_moves);
    object.insert("use_camera_aim", config.use_camera_aim);
    object.insert("auto_start_mission", config.auto_start_mission);
    object.insert("compress_waypoint_segments", config.compress_waypoint_segments);
    object.insert("compress_non_waypoint_segments", config.compress_non_waypoint_segments);
    object.insert("frame", config.frame);
    object.insert("cam_tolerance", config.cam_tolerance);
    object.insert("camera_aim_pid_p", config.camera_aim_pid_p);
    object.insert("camera_aim_pid_i", config.camera_aim_pid_i);
    object.insert("camera_aim_pid_d", config.camera_aim_pid_d);
    object.insert("camera_aim_target_timeout_s", config.camera_aim_target_timeout_s);
    object.insert("camera_aim_stable_cycles", config.camera_aim_stable_cycles);
    object.insert("camera_aim_max_step", config.camera_aim_max_step);
    object.insert("camera_aim_wait_first_targets_timeout_s",
                  config.camera_aim_wait_first_targets_timeout_s);
    object.insert("camera_aim_no_target_confirm_s",
                  config.camera_aim_no_target_confirm_s);
    object.insert("camera_aim_record_result_timeout_s",
                  config.camera_aim_record_result_timeout_s);
    object.insert("camera_aim_scan_point_timeout_s",
                  config.camera_aim_scan_point_timeout_s);
    return object;
}

bool missionConfigFromJson(const QJsonObject &object,
                           MissionConfig &config,
                           QString *error_message)
{
    return readDouble(object, "takeoff_altitude", config.takeoff_altitude, error_message) &&
           readDouble(object, "move_altitude", config.move_altitude, error_message) &&
           readDouble(object, "start_altitude", config.start_altitude, error_message) &&
           readDouble(object, "yaw", config.yaw, error_message) &&
           readDouble(object, "tolerance", config.tolerance, error_message) &&
           readDouble(object, "takeoff_hover_duration", config.takeoff_hover_duration, error_message) &&
           readDouble(object, "landing_hover_duration", config.landing_hover_duration, error_message) &&
           readDouble(object, "move_hover_duration", config.move_hover_duration, error_message) &&
           readBool(object, "add_hover_between_takeoff", config.add_hover_between_takeoff, error_message) &&
           readBool(object, "add_hover_between_landing", config.add_hover_between_landing, error_message) &&
           readBool(object, "add_hover_between_moves", config.add_hover_between_moves, error_message) &&
           readBool(object, "use_camera_aim", config.use_camera_aim, error_message) &&
           readBool(object, "auto_start_mission", config.auto_start_mission, error_message) &&
           readBool(object, "compress_waypoint_segments", config.compress_waypoint_segments, error_message) &&
           readBool(object, "compress_non_waypoint_segments", config.compress_non_waypoint_segments, error_message) &&
           readString(object, "frame", config.frame, error_message) &&
           readDouble(object, "cam_tolerance", config.cam_tolerance, error_message) &&
           readDouble(object, "camera_aim_pid_p", config.camera_aim_pid_p, error_message) &&
           readDouble(object, "camera_aim_pid_i", config.camera_aim_pid_i, error_message) &&
           readDouble(object, "camera_aim_pid_d", config.camera_aim_pid_d, error_message) &&
           readDouble(object, "camera_aim_target_timeout_s", config.camera_aim_target_timeout_s, error_message) &&
           readUInt16(object, "camera_aim_stable_cycles", config.camera_aim_stable_cycles, error_message) &&
           readDouble(object, "camera_aim_max_step", config.camera_aim_max_step, error_message) &&
           readDouble(object, "camera_aim_wait_first_targets_timeout_s",
                      config.camera_aim_wait_first_targets_timeout_s, error_message) &&
           readDouble(object, "camera_aim_no_target_confirm_s",
                      config.camera_aim_no_target_confirm_s, error_message) &&
           readDouble(object, "camera_aim_record_result_timeout_s",
                      config.camera_aim_record_result_timeout_s, error_message) &&
           readDouble(object, "camera_aim_scan_point_timeout_s",
                      config.camera_aim_scan_point_timeout_s, error_message);
}

QJsonObject warehouseConfigToJson(const WarehouseConfig &config)
{
    QJsonArray shelves;
    for (const ShelfConfig &shelf : config.shelves)
    {
        QJsonArray pose_regions;
        for (const ShelfPoseRegionConfig &region : shelf.pose_regions)
        {
            QJsonObject region_object;
            region_object.insert("side", region.side);
            region_object.insert("x_min", region.x_min);
            region_object.insert("x_max", region.x_max);
            region_object.insert("yaw_min", region.yaw_min);
            region_object.insert("yaw_max", region.yaw_max);
            pose_regions.append(region_object);
        }

        QJsonObject rect;
        rect.insert("x", shelf.base_rect.x());
        rect.insert("y", shelf.base_rect.y());
        rect.insert("width", shelf.base_rect.width());
        rect.insert("length", shelf.base_rect.height());

        QJsonObject shelf_object;
        shelf_object.insert("code", shelf.code);
        shelf_object.insert("display_name", shelf.display_name);
        shelf_object.insert("front_slot_prefix", shelf.front_slot_prefix);
        shelf_object.insert("back_slot_prefix", shelf.back_slot_prefix);
        shelf_object.insert("base_rect", rect);
        shelf_object.insert("height", shelf.height);
        shelf_object.insert("scene_color", shelf.scene_color.name(QColor::HexArgb));
        shelf_object.insert("button_status_color", shelf.button_status_color);
        shelf_object.insert("front_waypoint_y_m", shelf.front_waypoint_y_m);
        shelf_object.insert("back_waypoint_y_m", shelf.back_waypoint_y_m);
        shelf_object.insert("pose_regions", pose_regions);
        shelves.append(shelf_object);
    }

    QJsonObject slot_grid_object;
    slot_grid_object.insert("rows", config.slot_grid.rows);
    slot_grid_object.insert("columns", config.slot_grid.columns);
    slot_grid_object.insert("waypoint_row_z_m", doubleVectorToJson(config.slot_grid.waypoint_row_z_m));
    slot_grid_object.insert("waypoint_front_x_m", doubleVectorToJson(config.slot_grid.waypoint_front_x_m));
    slot_grid_object.insert("waypoint_back_x_m", doubleVectorToJson(config.slot_grid.waypoint_back_x_m));
    slot_grid_object.insert("front_yaw_rad", config.slot_grid.front_yaw_rad);
    slot_grid_object.insert("back_yaw_rad", config.slot_grid.back_yaw_rad);
    slot_grid_object.insert("pose_y_min", config.slot_grid.pose_y_min);
    slot_grid_object.insert("pose_y_max", config.slot_grid.pose_y_max);
    slot_grid_object.insert("pose_z_min", config.slot_grid.pose_z_min);
    slot_grid_object.insert("pose_z_max", config.slot_grid.pose_z_max);

    QJsonObject root;
    root.insert("version", 2);
    root.insert("shelves", shelves);
    root.insert("slots", slot_grid_object);
    root.insert("ros", rosConfigToJson(config.ros));
    root.insert("bridge_ros", rosConfigToJson(config.bridge_ros));
    root.insert("connection", connectionConfigToJson(config.connection));
    root.insert("mission", missionConfigToJson(config.mission));
    return root;
}

bool warehouseConfigFromJson(const QJsonObject &root,
                             WarehouseConfig &config,
                             QString *error_message)
{
    int version = 0;
    if (!readInt(root, "version", version, error_message))
    {
        return false;
    }
    if (version != 1 && version != 2)
    {
        return jsonError(
            error_message,
            QString("不支持 warehouse_config.json 版本 %1").arg(version));
    }

    QJsonArray shelves_array;
    QJsonObject slots_object;
    QJsonObject ros_object;
    QJsonObject bridge_ros_object;
    QJsonObject mission_object;
    if (!readArray(root, "shelves", shelves_array, error_message) ||
        !readObject(root, "slots", slots_object, error_message) ||
        !readObject(root, "ros", ros_object, error_message) ||
        !readObject(root, "bridge_ros", bridge_ros_object, error_message) ||
        !readObject(root, "mission", mission_object, error_message))
    {
        return false;
    }

    // 版本 1 只有 bridge_serial，读取时自动迁移为默认 WiFi 模式。
    ConnectionConfig connection;
    if (version == 1)
    {
        QJsonObject legacy_serial_object;
        if (!readObject(
                root,
                "bridge_serial",
                legacy_serial_object,
                error_message) ||
            !serialConfigFromJson(
                legacy_serial_object,
                connection.telemetry_serial,
                error_message))
        {
            return false;
        }
        connection.mode = ConnectionMode::Wifi;
    }
    else
    {
        QJsonObject connection_object;
        if (!readObject(
                root, "connection", connection_object, error_message) ||
            !connectionConfigFromJson(
                connection_object, connection, error_message))
        {
            return false;
        }
    }

    QVector<ShelfConfig> shelves;
    shelves.reserve(shelves_array.size());
    for (int shelf_index = 0; shelf_index < shelves_array.size(); ++shelf_index)
    {
        if (!shelves_array.at(shelf_index).isObject())
        {
            return jsonError(
                error_message,
                QString("shelves[%1] 必须是 JSON 对象").arg(shelf_index));
        }

        const QJsonObject shelf_object = shelves_array.at(shelf_index).toObject();
        QJsonObject rect_object;
        QJsonArray pose_regions_array;
        ShelfConfig shelf;
        QString scene_color;
        if (!readString(shelf_object, "code", shelf.code, error_message) ||
            !readString(shelf_object, "display_name", shelf.display_name, error_message) ||
            !readString(shelf_object, "front_slot_prefix", shelf.front_slot_prefix, error_message) ||
            !readString(shelf_object, "back_slot_prefix", shelf.back_slot_prefix, error_message) ||
            !readObject(shelf_object, "base_rect", rect_object, error_message) ||
            !readDouble(shelf_object, "height", shelf.height, error_message) ||
            !readString(shelf_object, "scene_color", scene_color, error_message) ||
            !readString(shelf_object, "button_status_color", shelf.button_status_color, error_message) ||
            !readDouble(shelf_object, "front_waypoint_y_m", shelf.front_waypoint_y_m, error_message) ||
            !readDouble(shelf_object, "back_waypoint_y_m", shelf.back_waypoint_y_m, error_message) ||
            !readArray(shelf_object, "pose_regions", pose_regions_array, error_message))
        {
            return false;
        }

        double rect_x = 0.0;
        double rect_y = 0.0;
        double rect_width = 0.0;
        double rect_length = 0.0;
        if (!readDouble(rect_object, "x", rect_x, error_message) ||
            !readDouble(rect_object, "y", rect_y, error_message) ||
            !readDouble(rect_object, "width", rect_width, error_message) ||
            !readDouble(rect_object, "length", rect_length, error_message))
        {
            return false;
        }
        shelf.base_rect = QRectF(rect_x, rect_y, rect_width, rect_length);
        shelf.scene_color = QColor(scene_color);

        for (int region_index = 0;
             region_index < pose_regions_array.size();
             ++region_index)
        {
            if (!pose_regions_array.at(region_index).isObject())
            {
                return jsonError(
                    error_message,
                    QString("pose_regions[%1] 必须是 JSON 对象").arg(region_index));
            }

            const QJsonObject region_object =
                pose_regions_array.at(region_index).toObject();
            ShelfPoseRegionConfig region;
            if (!readString(region_object, "side", region.side, error_message) ||
                !readDouble(region_object, "x_min", region.x_min, error_message) ||
                !readDouble(region_object, "x_max", region.x_max, error_message) ||
                !readDouble(region_object, "yaw_min", region.yaw_min, error_message) ||
                !readDouble(region_object, "yaw_max", region.yaw_max, error_message))
            {
                return false;
            }
            shelf.pose_regions.push_back(region);
        }
        shelves.push_back(shelf);
    }

    SlotGridConfig slot_grid;
    if (!readInt(slots_object, "rows", slot_grid.rows, error_message) ||
        !readInt(slots_object, "columns", slot_grid.columns, error_message) ||
        !doubleVectorFromJson(slots_object, "waypoint_row_z_m",
                             slot_grid.waypoint_row_z_m, error_message) ||
        !doubleVectorFromJson(slots_object, "waypoint_front_x_m",
                             slot_grid.waypoint_front_x_m, error_message) ||
        !doubleVectorFromJson(slots_object, "waypoint_back_x_m",
                             slot_grid.waypoint_back_x_m, error_message) ||
        !readDouble(slots_object, "front_yaw_rad", slot_grid.front_yaw_rad, error_message) ||
        !readDouble(slots_object, "back_yaw_rad", slot_grid.back_yaw_rad, error_message) ||
        !readDouble(slots_object, "pose_y_min", slot_grid.pose_y_min, error_message) ||
        !readDouble(slots_object, "pose_y_max", slot_grid.pose_y_max, error_message) ||
        !readDouble(slots_object, "pose_z_min", slot_grid.pose_z_min, error_message) ||
        !readDouble(slots_object, "pose_z_max", slot_grid.pose_z_max, error_message))
    {
        return false;
    }

    WarehouseConfig loaded_config;
    loaded_config.shelves = shelves;
    loaded_config.slot_grid = slot_grid;
    loaded_config.connection = connection;
    if (!rosConfigFromJson(ros_object, loaded_config.ros, error_message) ||
        !rosConfigFromJson(bridge_ros_object, loaded_config.bridge_ros, error_message) ||
        !missionConfigFromJson(mission_object, loaded_config.mission, error_message))
    {
        return false;
    }

    config = loaded_config;
    return true;
}
}

void applyConnectionModeToRosConfig(
    WarehouseConfig &config,
    ConnectionMode mode)
{
    config.connection.mode = mode;

    if (mode == ConnectionMode::Telemetry)
    {
        // 数传时，地面站改为订阅/调用 bridge_ros 中的 /serial 接口。
        // 节点名仍属于地面站自身，不能替换成 ground_link_bridge。
        const QString ground_station_node_name = config.ros.node_name;
        config.ros = config.bridge_ros;
        config.ros.node_name = ground_station_node_name;
        return;
    }

    // WiFi 时恢复直连接口。只移除开头的 /serial，其他名称保持原样。
    auto remove_serial_prefix = [](const QString &name) {
        const QString prefix = "/serial";
        if (name == prefix)
        {
            return QString("/");
        }
        if (name.startsWith(prefix + "/"))
        {
            return name.mid(prefix.size());
        }
        return name;
    };

    config.ros.drone_status =
        remove_serial_prefix(config.ros.drone_status);
    config.ros.task_status =
        remove_serial_prefix(config.ros.task_status);
    config.ros.path_ready =
        remove_serial_prefix(config.ros.path_ready);
    config.ros.return_world_group =
        remove_serial_prefix(config.ros.return_world_group);
    config.ros.barcode_capture =
        remove_serial_prefix(config.ros.barcode_capture);
    config.ros.vision_barcode =
        remove_serial_prefix(config.ros.vision_barcode);
    config.ros.local_position =
        remove_serial_prefix(config.ros.local_position);
    config.ros.pose_delta =
        remove_serial_prefix(config.ros.pose_delta);
    config.ros.start_task_service =
        remove_serial_prefix(config.ros.start_task_service);
    config.ros.stop_push_service =
        remove_serial_prefix(config.ros.stop_push_service);
    config.ros.start_offboard_service =
        remove_serial_prefix(config.ros.start_offboard_service);
    config.ros.upload_mission_service =
        remove_serial_prefix(config.ros.upload_mission_service);
}

QString warehouseDataDirectory()
{
    const QString configured_directory =
        qEnvironmentVariable("WAREHOUSE_GCS_DATA_DIR").trimmed();
    if (!configured_directory.isEmpty())
    {
        return QDir(configured_directory).absolutePath();
    }
    return QDir(QDir::homePath()).filePath("warehouse_gcs_data");
}

QString warehouseConfigFilePath()
{
    return QDir(warehouseDataDirectory()).filePath("warehouse_config.json");
}

QString shelfPanelDataFilePath()
{
    return QDir(warehouseDataDirectory()).filePath("shelf_panel_data.json");
}

bool saveWarehouseConfig(const WarehouseConfig &config, QString *error_message)
{
    const QString validation_error = validateWarehouseConfig(config);
    if (!validation_error.isEmpty())
    {
        return jsonError(
            error_message,
            "仓库配置校验失败：" + validation_error);
    }

    QDir data_directory(warehouseDataDirectory());
    if (!data_directory.exists() && !data_directory.mkpath("."))
    {
        return jsonError(
            error_message,
            "无法创建仓库数据目录：" + data_directory.absolutePath());
    }

    QSaveFile file(warehouseConfigFilePath());
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate))
    {
        return jsonError(
            error_message,
            QString("无法打开配置文件 %1：%2")
                .arg(file.fileName(), file.errorString()));
    }

    const QByteArray json_bytes =
        QJsonDocument(warehouseConfigToJson(config))
            .toJson(QJsonDocument::Indented);
    if (file.write(json_bytes) != json_bytes.size())
    {
        const QString write_error = file.errorString();
        file.cancelWriting();
        return jsonError(
            error_message,
            QString("配置文件写入不完整 %1：%2")
                .arg(file.fileName(), write_error));
    }

    if (!file.commit())
    {
        return jsonError(
            error_message,
            QString("配置文件提交失败 %1：%2")
                .arg(file.fileName(), file.errorString()));
    }
    return true;
}

bool loadWarehouseConfig(WarehouseConfig &config, QString *error_message)
{
    config = createDefaultWarehouseConfig();

    QFile file(warehouseConfigFilePath());
    if (!file.exists())
    {
        return saveWarehouseConfig(config, error_message);
    }
    if (!file.open(QIODevice::ReadOnly))
    {
        return jsonError(
            error_message,
            QString("无法读取配置文件 %1：%2")
                .arg(file.fileName(), file.errorString()));
    }

    QJsonParseError parse_error;
    const QJsonDocument document =
        QJsonDocument::fromJson(file.readAll(), &parse_error);
    if (parse_error.error != QJsonParseError::NoError ||
        !document.isObject())
    {
        return jsonError(
            error_message,
            QString("配置 JSON 解析错误 %1：%2")
                .arg(file.fileName(), parse_error.errorString()));
    }

    WarehouseConfig loaded_config;
    if (!warehouseConfigFromJson(
            document.object(), loaded_config, error_message))
    {
        if (error_message)
        {
            *error_message =
                QString("%1：%2").arg(file.fileName(), *error_message);
        }
        return false;
    }

    const QString validation_error =
        validateWarehouseConfig(loaded_config);
    if (!validation_error.isEmpty())
    {
        return jsonError(
            error_message,
            QString("%1：配置校验失败：%2")
                .arg(file.fileName(), validation_error));
    }

    config = loaded_config;
    return true;
}