#include "drone_qt/mainwindow.hpp"
#include "drone_qt/ros_manager.hpp"
#include "drone_qt/image_preview_dialog.hpp"
#include "drone_qt/position_view_widget.hpp"
#include "drone_qt/mission_yaml_builder.hpp"

#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>
#include <QGroupBox>
#include <QListWidget>
#include <QListWidgetItem>
#include <QScrollArea>
#include <QDialog>
#include <QPixmap>
#include <QImage>
#include <QPainter>
#include <QPen>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QByteArray>
#include <QPlainTextEdit>
#include <QMetaType>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ros_manager_(new RosManager(this))
{
    qRegisterMetaType<WorldCoord>("WorldCoord");
    qRegisterMetaType<QVector<WorldCoord>>("QVector<WorldCoord>");

    setupUi();
    setupConnections();
    setupSerialPort();
    ros_manager_->start();
    //开始按钮还没上传路线时不应该可用
    start_button_->setEnabled(false);
}

void MainWindow::setupUi()
{
    //最外层：上下布局
    auto *central = new QWidget(this);
    auto *main_layout  = new QVBoxLayout(central);

    //上方状态区1：左右布局
    auto *up_panel = new QGroupBox("无人机状态", central);
    auto *up_layout = new QHBoxLayout(up_panel);

    //上方状态区1：左右布局
    // auto *upd_panel = new QGroupBox("运行状态", central);
    // auto *upd_layout = new QHBoxLayout(upd_panel);

    //下方总区域：左右布局
    auto *down_panel = new QGroupBox("结果区", central);
    auto *down_layout = new QHBoxLayout(down_panel);

    //下左侧显示区：显示二维位置
    auto *display_panel = new QGroupBox("二维位置", down_panel);
    auto *display_layout = new QVBoxLayout(display_panel);

    //右侧总区：（按钮+视觉结果）网格布局
    auto *right_panel = new QGroupBox("控制与识别", down_panel);
    auto *right_layout = new QGridLayout(right_panel);

    //右上侧控制区：垂直布局
    // auto *button_panel = new QGroupBox("控制", right_panel);
    // auto *button_layout = new QGridLayout(button_panel);

    //右侧方向控制区：网格布局
    auto *direction_label = new QGroupBox("控制", right_panel);
    auto *direction_layout = new QGridLayout(direction_label);

    //右右侧识别结果区：垂直布局
    auto *capture_panel = new QGroupBox("识别结果", right_panel);
    auto *capture_layout = new QVBoxLayout(capture_panel);

    //右下侧日志查看区：垂直布局
    auto *log_panel = new QGroupBox("日志", right_panel);
    auto *log_layout = new QVBoxLayout(log_panel);

    /*================= 状态区控件 =================*/
    //创建上方状态区的状态显示元素
    connection_label_ = new QLabel("disconnected", up_panel);
    battery_label_ = new QLabel("N/A", up_panel);
    mode_label_ = new QLabel("MODE_UNKNOWN", up_panel);
    armed_label_ = new QLabel("Lock", up_panel);
    //status_label_ = new QLabel("空闲", up_panel);
    action_label_ = new QLabel("无", up_panel);
    progress_label_ = new QLabel("0/0", up_panel);
    progress_percent_label_ = new QLabel("0%", up_panel);
    dx_indicator_label_ = new QLabel(up_panel);
    dy_indicator_label_ = new QLabel(up_panel);
    dyaw_indicator_label_ = new QLabel(up_panel);
    dx_value_label_ = new QLabel("dx:", up_panel);
    dy_value_label_ = new QLabel("dy:", up_panel);
    dyaw_value_label_ = new QLabel("dyaw:", up_panel);

    dx_indicator_label_->setFixedSize(16, 16);
    dx_indicator_label_->setStyleSheet(
        "background-color: #9e9e9e;"
        "border-radius: 6px;"
        "border: 1px solid #666;"
    );
    dy_indicator_label_->setFixedSize(16, 16);
    dy_indicator_label_->setStyleSheet(
        "background-color: #9e9e9e;"
        "border-radius: 6px;"
        "border: 1px solid #666;"
    );
    dyaw_indicator_label_->setFixedSize(16, 16);
    dyaw_indicator_label_->setStyleSheet(
        "background-color: #9e9e9e;"
        "border-radius: 6px;"
        "border: 1px solid #666;"
    );

    //up_layout->addStretch();//添加一个伸缩项，靠右显示
    //up_layout->addWidget(new QLabel("连接状态：", up_panel));
    up_layout->addWidget(connection_label_);
    up_layout->addSpacing(20);
    //up_layout->addWidget(new QLabel("模式：", up_panel));//初始模式文本
    up_layout->addWidget(mode_label_);
    up_layout->addSpacing(20);
    //up_layout->addWidget(new QLabel("电量：", up_panel));//初始电量文本
    up_layout->addWidget(battery_label_);
    up_layout->addSpacing(30);
    //up_layout->addWidget(new QLabel("解锁状态：", up_panel));//初始解锁状态文本
    up_layout->addWidget(armed_label_);
    up_layout->addStretch();//添加一个伸缩项，靠左显示

    up_layout->addWidget(dx_value_label_);
    up_layout->addWidget(dx_indicator_label_);
    up_layout->addSpacing(20);
    up_layout->addWidget(dy_value_label_);
    up_layout->addWidget(dy_indicator_label_);
    up_layout->addSpacing(20);
    up_layout->addWidget(dyaw_value_label_);
    up_layout->addWidget(dyaw_indicator_label_);
    up_layout->addSpacing(20);

    up_layout->addStretch();//添加一个伸缩项
    up_layout->addSpacing(20);//添加一个水平间距，分隔连接状态和电量状态
    // up_layout->addWidget(new QLabel("当前执行任务：", up_panel));
    // up_layout->addWidget(status_label_);
    // up_layout->addSpacing(20);
    up_layout->addWidget(new QLabel("action：", up_panel));
    up_layout->addWidget(action_label_);
    up_layout->addSpacing(20);
    up_layout->addWidget(new QLabel("progress：", up_panel));
    up_layout->addWidget(progress_label_);
    up_layout->addSpacing(20);
    up_layout->addWidget(progress_percent_label_);
    up_layout->addSpacing(20);
    //upd_layout->addStretch();//添加一个伸缩项，靠左显示
    /*=============================================*/

    /*=============== 二维位置区控件 ===============*/
    //创建左下方显示区的二维位置显示元素
    position_view_ = new PositionViewWidget(display_panel);
    display_layout->addWidget(position_view_);
    /*=============================================*/

    /*================= 按钮区控件 =================*/
    //创建右上侧控制元素
    start_button_ = new QPushButton("起飞", right_panel);
    stop_button_ = new QPushButton("停止", right_panel);
    push_button_ = new QPushButton("上传", right_panel);
    refresh_button_ = new QPushButton("刷新", right_panel);
    true_button_ = new QPushButton("确定", right_panel);
    del_button_ = new QPushButton("删除", right_panel);
    display_button_ = new QPushButton("显示", right_panel);
    clear_button_ = new QPushButton("清空", right_panel);

    //将按钮元素添加到右上方按钮区的布局中
    // button_layout->addWidget(start_button_);
    // button_layout->addWidget(stop_button_);
    // button_layout->addWidget(push_button_);
    // button_layout->addWidget(refresh_button_);

    up_button_ = new QPushButton("↑", right_panel);
    left_button_ = new QPushButton("←", right_panel);
    down_button_ = new QPushButton("↓", right_panel);
    right_button_ = new QPushButton("→", right_panel);

    //将方向控制按钮添加到右侧方向控制区的布局中，形成一个简单的网格布局
    direction_layout->addWidget(start_button_,    0, 0);
    direction_layout->addWidget(stop_button_,  0, 1);
    direction_layout->addWidget(clear_button_,  0, 2);
    direction_layout->addWidget(push_button_,  1, 0);
    direction_layout->addWidget(refresh_button_, 1, 1);
    direction_layout->addWidget(display_button_, 1, 2);
    direction_layout->addWidget(true_button_,  2, 0);
    direction_layout->addWidget(del_button_, 2, 2);
    direction_layout->addWidget(up_button_,    2, 1);
    direction_layout->addWidget(left_button_,  3, 0);
    direction_layout->addWidget(down_button_,  3, 1);
    direction_layout->addWidget(right_button_, 3, 2);
    /*=============================================*/

    /*=============== 识别结果区控件 ===============*/
    //barcode_list_->addItem("A | test_1");
    //barcode_list_->addItem("B | test_2");
    //创建右下方视觉结果显示元素
    barcode_list_ = new QListWidget(capture_panel);
    //设置列表控件的滚动条策略、选择模式和编辑触发方式等属性
    barcode_list_->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    barcode_list_->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    barcode_list_->setSelectionMode(QAbstractItemView::SingleSelection);
    barcode_list_->setEditTriggers(QAbstractItemView::NoEditTriggers);

    //将视觉结果显示元素添加到右下识别结果区的布局中
    capture_layout->addWidget(barcode_list_);
    /*=============================================*/

    /*=============== 日志查看区控件 ===============*/
    run_log_view_ = new QPlainTextEdit(log_panel);
    run_log_view_->setReadOnly(true);
    run_log_view_->setMaximumBlockCount(1000);

    log_layout->addWidget(run_log_view_);

    run_log_view_->appendPlainText("日志区域初始化成功");
    run_log_view_->appendPlainText("等待新日志...");
    /*=============================================*/

    /*================ 布局配置控制 ================*/
    //主布局分配：上方状态区占1份，下方总区占10份
    up_panel->setFixedHeight(60);
    //upd_panel->setFixedHeight(70);
    main_layout->addWidget(up_panel,0);
    //main_layout->addWidget(upd_panel,0);
    main_layout->addWidget(down_panel,1);

    //下方总区布局分配：左侧显示区占5份，右侧总区占2份
    down_layout->addWidget(display_panel, 1);
    down_layout->addWidget(right_panel, 1);

    //右侧总区布局分配：按钮区占1份，方向控制区占2份，识别结果区占2份
    //right_layout->addWidget(button_panel,1);
    right_layout->addWidget(direction_label,0,0);
    right_layout->addWidget(capture_panel,0,1);
    right_layout->addWidget(log_panel,1,0,1,2);
    direction_label->setMaximumWidth(220);
    //direction_label->setMaximumHeight(100);
    right_layout->setRowStretch(0, 1);
    right_layout->setRowStretch(1, 1);

    //设置主布局的边距和间距等属性，确保界面元素之间有适当的空隙和分隔
    main_layout->setContentsMargins(2, 2, 2, 2);
    main_layout->setSpacing(1);

    // 先给一个基础高度，确保界面上能看见
    //barcode_list_->setFixedHeight(120);
    //capture_panel->setMaximumHeight(120);

    //将中心部件设置为主窗口的中心部件
    setCentralWidget(central);

    //设置窗口标题和初始大小和窗口标题
    setFixedSize(1024, 540);
    setWindowTitle("Ground Station");
    /*=============================================*/
}

void MainWindow::setupConnections()
{
    // 这里与二维界面直接相关的有三条链路：
    // 1. refresh_button_ -> PositionViewWidget::refreshPlannedPath()：确认灰格并重规划路线
    // 2. 方向键按钮 -> moveSelectionXxx()：只移动绿色选择框，不改背景路线
    // 3. positionUpdated -> setPosition()：只刷新无人机红点与坐标文字
    //连接按钮点击信号到ROS管理器的任务启动槽
    connect(start_button_, &QPushButton::clicked,
            this, &MainWindow::handleStartButtonClicked);

    //连接按钮点击信号到ROS管理器的任务启动槽
    connect(stop_button_, &QPushButton::clicked,
            ros_manager_, &RosManager::stopTask);

    //连接按钮点击信号到ROS管理器的发布路径
    // connect(push_button_, &QPushButton::clicked,
    //     this,
    //     [this]()
    //     {
    //         if (!position_view_ || !ros_manager_) {
    //             return;
    //         }

    //         if(waiting_push_result_){
    //             return;
    //         }

    //         waiting_push_result_ = true;
    //         //从界面获取当前坐标
    //         const QVector<WorldCoord> path_points = position_view_->plannedWorldPoints();
    //         if (path_points.isEmpty()) {
    //             run_log_view_->appendPlainText("路径为空，不能上传 mission YAML");
    //             waiting_push_result_ = false;
    //             return;
    //         }
            
    //         //构建mission yaml字符串，初始化参数
    //         MissionYamlBuilder::Options options;
    //         options.takeoff_altitude = 1.2;//起飞高度
    //         options.move_altitude = 1.2;//移动高度
    //         options.start_altitude = 0.0;//解锁高度
    //         options.yaw = 0.0;//偏航角
    //         options.tolerance = 0.12;//误差容忍
    //         options.takeoff_hover_duration = 4.0;//起飞悬停时长
    //         options.landing_hover_duration = 4.0;//降落悬停时长
    //         options.move_hover_duration = 2.0;//移动悬停时长
    //         options.add_hover_between_takeoff = true;//是否在起飞后添加悬停
    //         options.add_hover_between_landing = true;//是否在降落前添加悬停
    //         options.add_hover_between_moves = true;//是否在移动之间添加悬停
    //         options.use_camera_aim = false;//是否开启相机
    //         options.auto_start_mission = false;//是否自动启动任务
    //         options.compress_straight_segments = true;//是否压缩直线段

    //         //构建mission yaml字符串
    //         const QString mission_yaml = MissionYamlBuilder::buildMissionYaml(path_points, options);
    //         run_log_view_->appendPlainText("正在上传 mission YAML 到机载端...");
    //         start_button_->setEnabled(false);
    //         //发布坐标
    //         ros_manager_->uploadMissionYaml(mission_yaml);
    //     });

    //连接按钮点击信号到ROS管理器的发布路径与参数摘要
    connect(push_button_, &QPushButton::clicked,
    this,
    [this]()
    {
        if (!position_view_ || !ros_manager_) {
            return;
        }

        if (waiting_push_result_) {
            return;
        }

        waiting_push_result_ = true;
        const QVector<WorldCoord> path_points = position_view_->plannedWorldPoints();
        if (path_points.isEmpty()) {
            run_log_view_->appendPlainText("路径为空，不能上传 mission summary");
            waiting_push_result_ = false;
            return;
        }

        drone_msgs::msg::MissionSummary summary;
        summary.takeoff_altitude = 1.2;//起飞高度
        summary.move_altitude = 1.2;//移动高度
        summary.start_altitude = 0.0;//解锁高度
        summary.yaw = 0.0;//偏航角
        summary.tolerance = 0.12;//误差容忍
        summary.takeoff_hover_duration = 4.0;//起飞悬停时长
        summary.landing_hover_duration = 4.0;//降落悬停时长
        summary.move_hover_duration = 2.0;//移动悬停时长
        summary.add_hover_between_takeoff = true;//是否在起飞后添加悬停
        summary.add_hover_between_landing = true;//是否在降落前添加悬停
        summary.add_hover_between_moves = true;//是否在移动之间添加悬停
        summary.use_camera_aim = false;//是否开启相机
        summary.auto_start_mission = false;//是否自动启动任务
        summary.compress_straight_segments = true;//是否压缩直线段
        summary.frame = "world_body";

        run_log_view_->appendPlainText("正在上传路线和 mission 参数摘要到机载端...");
        start_button_->setEnabled(false);
        //发布坐标和参数摘要
        ros_manager_->uploadMissionSummary(path_points, summary);
    });

    //链接是否上传
    connect(ros_manager_, &RosManager::pushFlagChanged,
        this,
        [this](bool value)
        {
            if (position_view_) {
                position_view_->pushFlagresult(value);
            }
        });

    //链接判断控制程序返回内容
    connect(ros_manager_, &RosManager::pathReadyChanged,
        this,
        [this](bool ready)
        {
            updatePathReadyState(ready);
        },
        Qt::QueuedConnection);

    //链接返回预规划路线坐标点的信号
    connect(ros_manager_, &RosManager::returnWorldGroupUpdated,
        this,
        [this](const QVector<WorldCoord> &points)
        {
            updateWorldGroupState(points);
        },
        Qt::QueuedConnection);

    //查看起飞启动服务返回的内容
    connect(ros_manager_, &RosManager::commandResult,
            this,
            [this](bool success, const QString &message)
            {
                //根据命令执行结果的成功与否，更新界面上的结果标签文本，显示相关消息
                updateCommandResult(success, message);
            },
            Qt::QueuedConnection);

//查看停止服务返回的内容
    connect(ros_manager_, &RosManager::stopcommandResult,
            this,
            [this](bool success, const QString &message)
            {
                if(success){
                    path_ready_ = false;
                    waiting_push_result_ = false;
                    start_button_->setEnabled(false);
                    delta_result_ = true;
                    //push_button_->setEnabled(true);
                }
                run_log_view_->appendPlainText(QString("%1").arg(message));
            },
            Qt::QueuedConnection);

    //查看offboard启动服务返回的内容
    connect(ros_manager_, &RosManager::offboardCommandResult,
        this,
        [this](bool success, const QString &message)
        {
            if(success){
                //push_button_->setEnabled(false);
                waiting_task_result_ = false;
            }
            run_log_view_->appendPlainText(success
                ? QString("offboard 启动成功，等待ready信号确认")
                : QString("%1").arg(message));
        },
        Qt::QueuedConnection);

    //查看任务yaml上传服务返回的内容
    // connect(ros_manager_, &RosManager::missionUploadFinished,
    // this,
    // [this](bool success, const QString &message, const QString &saved_path)
    // {
    //     if (success) {
    //         start_button_->setEnabled(true);
    //         push_flag_ = false;//上传成功后重置上传标志,打印一次就好，后续不再打印，直到下一次点击上传按钮
    //         run_log_view_->appendPlainText(
    //             QString("mission YAML 上传成功，机载保存路径: %1").arg(saved_path));
    //             //上传成功后直接请求启动offboard
    //             ros_manager_->requestStartOffboard();
    //             run_log_view_->appendPlainText(QString("路线已上传给控制程序"));
    //     } else {
    //         start_button_->setEnabled(false);
    //         run_log_view_->appendPlainText(
    //             QString("mission YAML 上传失败: %1").arg(message));
    //             waiting_push_result_ = false;
    //     }
    // },
    // Qt::QueuedConnection);

    //查看任务yaml上传服务返回的内容
    connect(ros_manager_, &RosManager::missionUploadFinished,
    this,
    [this](bool success, const QString &message, const QString &saved_path)
    {
        if (success) {
            start_button_->setEnabled(true);
            push_flag_ = false;
            run_log_view_->appendPlainText(
                QString("mission 摘要上传成功，机载已生成 YAML，保存路径: %1").arg(saved_path));
            ros_manager_->requestStartOffboard();
            run_log_view_->appendPlainText(QString("路线已上传给控制程序"));
        } else {
            start_button_->setEnabled(false);
            run_log_view_->appendPlainText(
                QString("mission 摘要上传失败: %1").arg(message));
            waiting_push_result_ = false;
        }
    },
    Qt::QueuedConnection);

    //链接是否选择显示路线
    connect(display_button_, &QPushButton::clicked,
        this,
        [this]()
        {
            if (position_view_) {
                position_view_->display_select();
            }
        });

    //清空日志的链接
    connect(refresh_button_, &QPushButton::clicked,
        this,
        [this]()
        {
            if (run_log_view_) {
                run_log_view_->clear();
                run_log_view_->appendPlainText("等待新日志...");
            }
        });

    //清空视觉列表的链接
    connect(clear_button_, &QPushButton::clicked,
        this,
        [this]()
        {
            barcode_records_.clear();
            if (barcode_list_) {
                barcode_list_->clear();
            }
        });
    
    //连接按钮点击信号到位置视图控件的路线刷新槽
    connect(true_button_, &QPushButton::clicked,
            position_view_, &PositionViewWidget::refreshPlannedPath);

    connect(del_button_, &QPushButton::clicked,
            position_view_, &PositionViewWidget::delPlannedPath);


    //连接方向控制按钮的点击信号到位置视图控件的相应槽函数，用于更新当前选中的格子位置
    connect(up_button_, &QPushButton::clicked,
            position_view_,&PositionViewWidget::moveSelectionUp);

    connect(down_button_, &QPushButton::clicked,
            position_view_,&PositionViewWidget::moveSelectionDown);

    connect(left_button_, &QPushButton::clicked,
            position_view_,&PositionViewWidget::moveSelectionLeft);

    connect(right_button_, &QPushButton::clicked,
            position_view_,&PositionViewWidget::moveSelectionRight);

    //连接串口接收数据到读取数据函数
    // connect(serial_port_, &QSerialPort::readyRead,
    //         this, &MainWindow::handleSerialReadyRead);

    //连接ROS管理器的状态更新信号到一个lambda槽，用于解析状态消息并更新界面上的状态标签
    connect(ros_manager_, &RosManager::statusUpdated,
            this,
            [this](
                bool connected,
                float battery_percent,
                int flight_mode,
                bool armed,
                const QString &task_name)
            {
                updateStatus(
                    connected, 
                    battery_percent, 
                    flight_mode, 
                    armed,
                    task_name);
            },
            Qt::QueuedConnection);

    connect(ros_manager_, &RosManager::action_statusUpdated,
            this,
            [this](
                bool task_running,
                int action_step,
                int action_num,
                const QString &action_name)
            {
                action_updateStatus(
                    task_running,
                    action_step, 
                    action_num, 
                    action_name);
            },
            Qt::QueuedConnection);

    //连接ROS管理器的状态更新信号到一个lambda槽，用于解析状态消息并更新界面上的状态标签
    connect(ros_manager_, &RosManager::barcodeCaptured,
        this,
        [this](const QString &barcode,
               const QByteArray &image_data,
               const QString &image_format,
               const QString &time_text)
        {
            appendBarcodeRecord(barcode, image_data, image_format, time_text);
        },
        Qt::QueuedConnection);

    //连接ROS管理器的条形码捕获信号到一个lambda槽，用于将接收到的条形码捕获消息中的数据添加到界面上的列表控件中
    connect(barcode_list_, &QListWidget::itemDoubleClicked,
        this, &MainWindow::showBarcodeImage);

    connect(ros_manager_, &RosManager::positionUpdated,
        this,
        [this](double x, double y, double z)
        {
            if (position_view_) {
                
                position_view_->setPosition(x, y, z);
            }
        },
        Qt::QueuedConnection);

    connect(ros_manager_, &RosManager::deltaUpdated,
        this,
        [this](double dx, double dy, double dyaw, bool valid)
        {
            updateDelta(dx, dy, dyaw, valid);
        },
        Qt::QueuedConnection);
}

void MainWindow::updateCommandResult(bool success, const QString &message)
{
    if (!result_label_) {
        return;
    }

    run_log_view_->appendPlainText(success
        ? QString("执行成功")
        : QString("执行失败: %1").arg(message));
}

void MainWindow::updateStatus(
    bool connected,
    float battery_percent,
    int flight_mode,
    bool armed,
    const QString &task_name)
{
    if (!connection_label_ || !battery_label_ || !mode_label_ || !armed_label_ || !action_label_) {
        return;
    }

    switch (flight_mode)
    {
        case drone_msgs::msg::DroneStatus::MODE_MANUAL:
            current_flight_mode = "MANUAL";
            break;
        case drone_msgs::msg::DroneStatus::MODE_OFFBOARD:
            current_flight_mode = "OFFBOARD";
            break;
        case drone_msgs::msg::DroneStatus::MODE_STABILIZE:
            current_flight_mode = "STABILIZE";
            break;
        case drone_msgs::msg::DroneStatus::MODE_AUTO:
            current_flight_mode = "AUTO";
            break;
        case drone_msgs::msg::DroneStatus::MODE_LOITER:
            current_flight_mode = "LOITER";
            break;
        case drone_msgs::msg::DroneStatus::MODE_RTL:
            current_flight_mode = "RTL";
            break;
        default:
            current_flight_mode = "UNKNOWN";
            break;
    }
    //根据状态消息的内容，更新界面上的状态标签
    connection_label_->setText(connected ? "connected" : "disconnected");
    battery_label_->setText(QString::number(battery_percent * 100.0f, 'f', 1) + "%");
    mode_label_->setText(current_flight_mode);
    armed_label_->setText(armed ? "Unlock" : "Lock");
    //const QString action_text = action_name.trimmed().isEmpty() ? "无" : action_name.trimmed();
    //action_label_->setText(action_text);
    // if (task_running_) {
    //     status_label_->setText(QString("%1").arg(task_name));
    // } else {
    //     status_label_->setText("空闲");
    // }

    if (armed == true) {
        if (unlock_flag_ == false) {
            unlock_flag_ = true;       // 标记：本轮已经开锁过
            auto_stop_flag_ = false;    // 新一轮允许自动 stop
        }

        disarm_stable_count_ = 0;       // 只要又变回开锁，就清零去抖计数
    }

    if(unlock_flag_ == true && armed == false){
        disarm_stable_count_++;//只有在曾经开锁过的状态才加
    }
    else if(unlock_flag_ == false && armed == false){
        disarm_stable_count_ = 0;       // 一直没开锁过，不做自动 stop
    }

    //判断是否为从开锁到关索的状态并且判断是否是第一次运行
    if(unlock_flag_ == true && auto_stop_flag_ == false && disarm_stable_count_ >= 15){
        auto_stop_flag_ = true;
        unlock_flag_ = false;
        waiting_push_result_ = false;//重置等待上传结果的标志，允许下一次上传
        start_button_->setEnabled(false);
        if (ros_manager_) {
            ros_manager_->stopTask();
            run_log_view_->appendPlainText(QString("请求offboard"));
        }
    }
}

void MainWindow::action_updateStatus(
    bool task_running,
    int action_step,
    int action_num,
    const QString &action_name)
{
    task_running_ = task_running;
    progress_ = ((float)action_step / (float)action_num) * 100;

    if (task_running_) {
        action_label_->setText(QString("%1").arg(action_name));
        progress_label_->setText(QString("%1/%2").arg(action_step).arg(action_num));
        progress_percent_label_->setText(QString("%1%").arg(progress_));
    } else {
        action_label_->setText("无");
        progress_label_->setText("0/0");
        progress_percent_label_->setText("0%");
    }
}

void MainWindow::updateDelta(double dx, double dy, double dyaw, bool valid)
{
    if (!run_log_view_) {
        return;
    }

    if(!delta_result_){
        return;
    }

    // run_log_view_->appendPlainText(
    //     QString("dx = %1      dy = %2      dyaw = %3")
    //         .arg(dx, 0, 'f', 3)
    //         .arg(dy, 0, 'f', 3)
    //         .arg(dyaw, 0, 'f', 3));

    const double abs_dx = std::abs(dx);
    const double abs_dy = std::abs(dy);
    const double abs_dyaw = std::abs(dyaw);

    auto setIndicatorColor = [](QLabel *label, const QString &color) {
        if (!label) {
            return;
        }

        label->setStyleSheet(QString(
            "background-color: %1;"
            "border-radius: 6px;"
            "border: 1px solid #666;"
        ).arg(color));
    };

    if (!valid) {
        setIndicatorColor(dx_indicator_label_, "#9e9e9e");
        setIndicatorColor(dy_indicator_label_, "#9e9e9e");
        setIndicatorColor(dyaw_indicator_label_, "#9e9e9e");
        return;
    }

    auto updateIndicator = [&](QLabel *label, double value, double green_limit, double yellow_limit) {
        if (value <= green_limit) {
            setIndicatorColor(label, "#00c853");
        } else if (value <= yellow_limit) {
            setIndicatorColor(label, "#ffd600");
        } else {
            setIndicatorColor(label, "#d50000");
        }
    };

    updateIndicator(dx_indicator_label_, abs_dx, 0.3, 1.0);
    updateIndicator(dy_indicator_label_, abs_dy, 0.3, 1.0);
    updateIndicator(dyaw_indicator_label_, abs_dyaw, 15.0, 30.0);
}

void MainWindow::appendBarcodeRecord(
    const QString &barcode,
    const QByteArray &image_data,
    const QString &image_format,
    const QString &time_text)
{
    if (!barcode_list_) {
        return;
    }

    //创建一个条形码捕获记录结构体实例，并将接收到的条形码数据、图像数据、图像格式和时间文本等信息存储在其中
    BarcodeRecord record;
    record.barcode = barcode;
    record.image_data = image_data;
    record.image_format = image_format;
    record.time_text = time_text;

    //将记录添加到条形码捕获记录列表中，以便后续使用
    barcode_records_.append(record);

    //在界面上的列表控件中添加一个新的列表项，显示条形码数据和时间文本等信息
    const int index = barcode_records_.size() - 1;
    const QString text = QString("%1 | %2").arg(barcode, time_text);

    //创建一个新的列表项，并将其数据设置为记录在列表中的索引，以便在点击列表项时能够找到对应的记录
    auto *item = new QListWidgetItem(text, barcode_list_);
    item->setData(Qt::UserRole, index);

    //将列表项添加到列表控件中，并更新列表控件的高度，以适应内容的变化
    barcode_list_->addItem(item);
    updateBarcodeListHeight();
}

void MainWindow::updateBarcodeListHeight()
{
    if (!barcode_list_) {
        return;
    }
    //根据列表控件中的项数和每行的高度，计算并设置列表控件的最小高度，以适应内容的变化
    const int row_height = barcode_list_->count() > 0 ? barcode_list_->sizeHintForRow(0) : 32;

    //设置一个可见行数的上限，避免列表过高，同时考虑到列表的边框和间距等因素，计算出合适的最小高度
    const int visible_rows = 5;
    const int frame = barcode_list_->frameWidth() *2;
    const int height = row_height * visible_rows + frame + 4;
    barcode_list_->setMinimumHeight(height);
}

//当用户双击列表项时，显示对应的图像预览对话框
void MainWindow::showBarcodeImage(QListWidgetItem *item)
{
    if (!item) {
        return;
    }

    //从点击的列表项中获取对应的索引，并根据索引从条形码捕获记录列表中找到对应的记录
    const int index = item->data(Qt::UserRole).toInt();
    if (index < 0 || index >= barcode_records_.size()) {
        return;
    }

    //从记录中获取图像数据，并尝试将其加载为QImage对象，如果加载失败则直接返回
    const BarcodeRecord &record = barcode_records_[index];

    //将记录中的图像数据转换为QImage对象，以便在图像预览对话框中显示
    QImage image;
    if (!image.loadFromData(record.image_data)) {
        return;
    }

    //如果图像加载成功，则创建一个图像预览对话框，并将图像和标题文本设置到对话框中，然后显示对话框
    if (!image_preview_dialog_) {
        image_preview_dialog_ = new ImagePreviewDialog(this);
    }

    //根据记录中的条形码数据构建一个标题文本，并将图像和标题文本设置到图像预览对话框中，然后显示对话框
    const QString title = QString("条形码: %1").arg(record.barcode);
    image_preview_dialog_->setImage(image, title);
    image_preview_dialog_->show();
    image_preview_dialog_->raise();
    image_preview_dialog_->activateWindow();
}

void MainWindow::updatePathReadyState(bool ready)
{
    path_ready_ = ready;
}

void MainWindow::updateWorldGroupState(const QVector<WorldCoord> &points)
{
    QString text = QString("收到返回路径，共 %1 个点").arg(points.size());

    for (int i = 0; i < points.size(); ++i) {
        const auto &point = points[i];
        text += QString(" -> (%1,%2)")
                    .arg(point.x, 0, 'f', 1)
                    .arg(point.y, 0, 'f', 1);
    }

    //只打印一次
    if(push_flag_){
        return;
    }

    run_log_view_->appendPlainText(text);
    push_flag_ = true;
}

void MainWindow::handleStartButtonClicked()
{
    if (!ros_manager_) {
        return;
    }

    if (!path_ready_) {
        run_log_view_->appendPlainText("路线未确认，不能执行开始按钮");
        return;
    }

    if (waiting_task_result_) {
        run_log_view_->appendPlainText("任务请求处理中，请勿重复点击");
        return;
    }

    waiting_task_result_ = true;
    delta_result_ = true;
    run_log_view_->appendPlainText("开始任务");
    ros_manager_->startTask();
    start_button_->setEnabled(false);
}

void MainWindow::setupSerialPort()
{
    serial_port_ = new QSerialPort(this);

    serial_port_->setPortName("/dev/ttyUSB0");
    serial_port_->setBaudRate(QSerialPort::Baud9600);
    serial_port_->setDataBits(QSerialPort::Data8);
    serial_port_->setParity(QSerialPort::NoParity);
    serial_port_->setStopBits(QSerialPort::OneStop);
    serial_port_->setFlowControl(QSerialPort::NoFlowControl);

    connect(serial_port_, &QSerialPort::readyRead,
            this, &MainWindow::handleSerialReadyRead);

    //只读打开
    if (!serial_port_->open(QIODevice::ReadOnly)) {
        serial_port_->deleteLater();
        serial_port_ = nullptr;
        return;
    }
}

void MainWindow::handleSerialReadyRead()
{
    if (!serial_port_) {
        return;
    }

    //将所有字节存进serial_buffer_
    serial_buffer_.append(serial_port_->readAll());

    while (true) {
        //只要内容出现\n就把前面的数据保存进line_data，没数据就返回
        const int newline_index = serial_buffer_.indexOf('\n');
        if (newline_index < 0) {
            break;
        }

        const QByteArray line_data = serial_buffer_.left(newline_index);
        serial_buffer_.remove(0, newline_index + 1);

        const QString line = QString::fromUtf8(line_data).trimmed();
        if (!line.isEmpty()) {
            processSerialLine(line);
        }
    }
}

void MainWindow::processSerialLine(const QString &line)
{
    const QString command = line.trimmed().toUpper();

    if (command == "UP" ||
        command == "DOWN" ||
        command == "LEFT" ||
        command == "RIGHT") {
        triggerDirectionCommand(command);
    }
}

void MainWindow::triggerDirectionCommand(const QString &command)
{
    if (!position_view_) {
        return;
    }

    if (command == "UP") {
        position_view_->moveSelectionUp();
        return;
    }

    if (command == "DOWN") {
        position_view_->moveSelectionDown();
        return;
    }

    if (command == "LEFT") {
        position_view_->moveSelectionLeft();
        return;
    }

    if (command == "RIGHT") {
        position_view_->moveSelectionRight();
        return;
    }
}