#pragma once

#include <QByteArray>
#include <QMainWindow>
#include <QVector>
#include <memory>

#include "drone_warehouse/models.hpp"
#include "drone_warehouse/warehouse_config.hpp"
#include "drone_msgs/msg/mission_summary.hpp"

class QLabel;
class QWidget;
class CargoInspectionPage;
class AnimalInspectionPage;
class ShelfInfoDialog;
class TopStatusBar;
class RosManager;
class QTimer;

class QDialog;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);

    bool updateStatus_connected() const;

protected:
    void resizeEvent(QResizeEvent *event) override;//窗口大小改变时调整悬浮控件的位置和大小

private:
    void setupUi();//搭建基本UI框架
    void setupFloatingWidgets();//搭建悬浮的状态面板和视图切换控件
    void setupConnections();//连接信号槽
    void setupDemoData();//设置一些演示用的假数据
    void updateOverlayGeometry();//调整悬浮控件的位置和大小
    void applyWindowStyle();//设置整体的窗口和控件样式
    void applyInspectionProject(InspectionProject project);//切换 Cargo/Animal 主画板
    void appendRunLog(const QString &text);//把运行日志写入当前项目页面
    void clearRunLogs();//清除两个项目页面的运行日志

    SlotLocation resolveSlotFromPose(const Pose3D &pose) const;//槽位判断函数
    SlotLocation resolveSlotFromCode(const QString &slot_code) const;//根据位置码直接解析槽位
    ShelfSlotItem *findShelfSlot(int shelf_index, const QString &side, int row, int col);//根据槽位索引找到对应的货物信息
    void showShelfSlotImage(int shelf_index, const QString &side, int row, int col);
    void applyManualStockIn(int shelf_index, const QString &side, int row, int col,
                            const QString &category_id, const QString &package_id);
    void applyManualStockOut(int shelf_index, const QString &side, int row, int col,
                            const QString &category_id, const QString &package_id);

    void refreshWaypointLog();
    void clearWaypointRequest();
    void setWaypointRequest(int shelf_index, const QString &side, int row, int col);

    /***********************AI相关*************************/

    QVector<SlotAnalysisInput> collectSlotAnalysisInputs() const;//收集所有槽位为分析输入
    QVector<SlotRuleAnalysis> buildRuleAnalysisResults() const;//生成规则分析结果
    QString buildRuleAnalysisReport(const QVector<SlotRuleAnalysis> &results) const;//纯规则报告
    void runAiDiffAnalysis();//规则触发入口

    QString buildAiPrompt(const SlotRuleAnalysis &result) const;//Prompt 构造函数
    QVector<SlotAiAnalysis> parseAiResult(const QString &text) const;
    void runClaudeApiDiffAnalysis();//API 调用函数

    /******************************************************/

    /*********************ros移植部分***********************/

    // 负责接收无人机基础状态更新，并同步到当前仓储界面的顶部状态栏和姿态信息区。
    void updateStatus(
        bool connected,
        float battery_percent,
        int flight_mode,
        bool armed,
        const QString &task_name);

    // 负责接收控制程序 action 状态更新，先把信息汇总到顶部任务文本里。
    void action_updateStatus(
        bool task_running,
        int action_step,
        int action_num,
        const QString &action_name);

    void appendBarcodeRecord(
        const QString &barcode,
        const QByteArray &image_data,
        const QString &image_format,
        const QString &time_text);

    void appendVisionBarcodeCount(
        const QString &barcode,
        const QString &time_text);

    // 当前仓储界面里还没有单独的 delta 可视化控件，先保留这个入口，后面如有需要再扩展显示位置。
    void updateDelta(double dx, double dy, double dyaw, bool valid);
    void handleVisionServoStatus(
        bool active,
        const QString &state,
        const QString &requested_target_id,
        const QString &tracked_target_id,
        const QString &detail,
        const QString &time_text);

    void updatePathReadyState(bool ready);

    // 把控制程序回传的路线按 x/y 写入运行日志；空路线返回 false。
    bool updateWorldGroupState(const QVector<WorldCoord> &points);

    // Animal 必须同时满足“Offboard 已成功”和“已收到有效路线”才允许启动任务。
    // 两个 ROS 回调的到达顺序不固定，因此统一在这里检查并只启动一次。
    void tryStartAnimalTask();

    // 上传路线统一入口。按钮触发和时间触发都先汇总到这里，避免后面维护两套上传逻辑。
    void triggerMissionUpload(const QString &trigger_source);

    void handleMissionUploadFinished(bool success, const QString &message, const QString &saved_path);//处理上传完成后的顶部文本反馈

    void updateCommandResult(bool success, const QString &message);

    /******************************************************/

private:
    WarehouseConfig config_;//当前仓库配置，包含货架、槽位、任务、串口和 ROS 接口

    QWidget *central_container_ = nullptr;//主容器，所有内容都放在这里面，方便统一管理布局和坐标
    CargoInspectionPage *cargo_page_ = nullptr;//Cargo 页面，管理仓库画板、日志和视角控件
    AnimalInspectionPage *animal_page_ = nullptr;//Animal 页面，管理二维栅格、运行日志和识别记录
    TopStatusBar *top_status_bar_ = nullptr;//顶部状态栏
    ShelfInfoDialog *shelf_info_dialog_ = nullptr;//货架信息弹窗模板窗口

    /*********************ros移植部分***********************/
    RosManager *ros_manager_ = nullptr;//ROS 管理器，负责订阅状态、调用服务、把 ROS 数据转成 Qt 信号
    WarehouseSceneData scene_data_;//当前主场景正在使用的数据，后面 ROS 状态和位置更新都会直接改这份数据再刷新 SceneView
    QString mission_trigger_time_text_;//预留给后续时间触发使用的目标时刻，本轮先不启用
    int mission_trigger_time_text_flag_ = 1;
    bool mission_time_trigger_enabled_ = false;//当前是否启用时间触发上传，本轮默认关闭
    bool mission_upload_in_progress_ = false;//当前是否有一条上传请求正在执行，避免重复触发

    // Animal 点击执行后保持为 true，直到成功调用一次 startTask() 或前置步骤失败。
    bool animal_route_start_pending_ = false;
    bool animal_offboard_ready_ = false;//本轮 Animal 执行的 Offboard 服务是否已成功
    QVector<WorldCoord> animal_returned_route_;//暂存本轮控制程序回传的路线

    bool vision_servo_active_seen_ = false;
    bool last_vision_servo_active_ = false;
    QString current_tracked_target_id_;

    QVector<WorldCoord> path_points_;
    QVector<QString> waypoint_labels_;
    /******************************************************/

    QTimer *clock_timer_ = nullptr;//用于每秒刷新一次顶部时间


    QWidget *attitude_panel_ = nullptr;//姿态面板
    QLabel *altitude_value_label_ = nullptr;//高度数值
    QLabel *speed_value_label_ = nullptr;//速度数值
    QLabel *yaw_value_label_ = nullptr;//航向数值
    QLabel *battery_value_label_ = nullptr;//电池电量数值
    QLabel *mode_value_label_ = nullptr;//模式显示


    QVector<ShelfPanelData> shelf_panel_data_;//主窗口持有的货架弹窗数据，后续图片和槽位更新都改这份
    QDialog *image_preview_dialog_{nullptr};//槽位图片预览弹窗
    QLabel *image_preview_label_ = nullptr;//图片预览标签

    bool unlock_flag_{false};//是否是从开锁到解锁的状态
    bool auto_stop_flag_{false};//判断是否自动执行过stop
    int disarm_stable_count_{0};    // 连续收到 armed == false 的次数

    bool connect_status{false};//无人机连接状态
};
