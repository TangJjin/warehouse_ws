#include "drone_warehouse/mainwindow.hpp"

#include "drone_warehouse/models.hpp"
#include "drone_warehouse/scene_view.hpp"
#include "drone_warehouse/shelf_info_dialog.hpp"
#include "drone_warehouse/top_status_bar.hpp"
#include "drone_warehouse/color_palette.hpp"
#include "drone_warehouse/ros_manager.hpp"

#include <cmath>
#include <QDialog>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPlainTextEdit>
#include <QResizeEvent>
#include <QScrollArea>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>
#include <QImage>
#include <QPixmap>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    /*********************ros移植部分***********************/
    // 注册跨线程 Qt 信号里会用到的 WorldCoord 元类型。
    // 原 drone_qt 工程里 returnWorldGroupUpdated 会跨线程发 QVector<WorldCoord>，
    // 这里先把类型注册好，避免后面一旦启用这条信号链时 Qt 不认识这个自定义类型。
    qRegisterMetaType<WorldCoord>("WorldCoord");
    qRegisterMetaType<QVector<WorldCoord>>("QVector<WorldCoord>");
    /******************************************************/
    //setWindowFlags(Qt::Window | Qt::FramelessWindowHint);//去掉主窗口系统标题栏，不再显示上方 warehouse_gcs 那一层
    //resize(1024, 600);
    setFixedSize(1024, 600);//设置初始窗口大小

    setupUi();
    setupFloatingWidgets();
    setupConnections();
    applyWindowStyle();
    setupDemoData();

    /*********************ros移植部分***********************/
    // 本轮先把时间触发接口和状态变量接进来，但按用户要求默认关闭。
    // 这样后面如果你要补一个“设置触发时刻”的入口，只需要把这两个值改掉即可。
    mission_trigger_time_text_ = "";
    mission_time_trigger_enabled_ = false;
    top_status_bar_->setTriggerTime(mission_trigger_time_text_);//传入想要定的时间
    top_status_bar_->setTimeTriggerEnabled(mission_time_trigger_enabled_);//传入是否开启时间定时
    /******************************************************/

    updateOverlayGeometry();

    /*********************ros移植部分***********************/
    // 界面和信号槽都准备好之后，再启动 RosManager 的 spin 线程。
    // 这样一来，后面 ROS 回调一到，就能立刻把状态发到已经存在的 Qt 控件上。
    if (ros_manager_)
    {
        ros_manager_->start();
    }
    /******************************************************/
}

void MainWindow::setupUi()
{
    central_container_ = new QWidget(this);
    //设置主容器
    setCentralWidget(central_container_);

    //创建主场景视图和顶部状态栏，并把它们放在主容器里，方便统一管理布局和坐标
    scene_view_ = new SceneView(central_container_);
    //主场景视图占满整个主容器，悬浮控件会在上面调整位置
    scene_view_->setGeometry(central_container_->rect());
    
    top_status_bar_ = new TopStatusBar(central_container_);
    //创建货架信息弹窗模板，先由主窗口持有，点击顶部按钮时再弹出显示
    shelf_info_dialog_ = new ShelfInfoDialog(this);

    /*********************ros移植部分***********************/
    // RosManager 是本次从 drone_qt 移植过来的 ROS 入口。
    // 这里先在主窗口创建它，后面统一在 setupConnections() 里连信号，在构造末尾启动 spin 线程。
    ros_manager_ = new RosManager(this);
    /******************************************************/
}

void MainWindow::setupFloatingWidgets()
{
    /***********************日志控件*************************/

    log_panel_ = new QWidget(central_container_);
    auto *log_layout = new QVBoxLayout(log_panel_);
    log_panel_->setObjectName("logSwitchPanel");
    log_panel_->setContentsMargins(5, 5, 5, 5);//姿态面板内部边框留白

    run_log_view_ = new QPlainTextEdit(log_panel_);
    run_log_view_->setReadOnly(true);
    run_log_view_->setMaximumBlockCount(1000);

    log_layout->addWidget(run_log_view_);

    clock_timer_ = new QTimer(this);//新建定时器
    run_log_view_->appendPlainText("日志初始化成功");
    clock_timer_->start(5000);

    /*******************************************************/

    /*********************悬浮姿态控件***********************/

    attitude_panel_ = new QWidget(central_container_);
    auto *attitude_layout = new QGridLayout(attitude_panel_);
    attitude_panel_->setObjectName("attitudeSwitchPanel");
    attitude_layout->setContentsMargins(12, 12, 12, 12);//姿态面板内部边框留白

    attitude_layout->addWidget(new QLabel("高度", attitude_panel_), 0, 0);
    altitude_value_label_ = new QLabel("0.0 m", attitude_panel_);
    attitude_layout->addWidget(altitude_value_label_, 0, 1);

    attitude_layout->addWidget(new QLabel("模式", attitude_panel_), 1, 0);
    speed_value_label_ = new QLabel("MODE_UNKNOWN", attitude_panel_);
    attitude_layout->addWidget(speed_value_label_, 1, 1);

    attitude_layout->addWidget(new QLabel("航向", attitude_panel_), 2, 0);
    yaw_value_label_ = new QLabel("0.0°", attitude_panel_);
    attitude_layout->addWidget(yaw_value_label_, 2, 1);

    /*******************************************************/

    /*********************模式切换滑块***********************/

    view_mode_widget_ = new QWidget(central_container_);
    auto *slider_layout = new QHBoxLayout(view_mode_widget_);
    slider_layout->setContentsMargins(1, 1, 1, 1);//滑块模块内部留白
    slider_layout->setSpacing(1);//滑块中各元素间距

    view_mode_left_label_ = new QLabel("2D", view_mode_widget_);
    view_mode_right_label_ = new QLabel("3D", view_mode_widget_);
    view_mode_slider_ = new QSlider(Qt::Horizontal, view_mode_widget_);//水平滑动条
    view_mode_slider_->setRange(0, 1);//滑块取值范围0-1，0表示2D模式，1表示3D模式
    view_mode_slider_->setValue(1);//默认3D模式
    view_mode_slider_->setFixedHeight(35);

    slider_layout->addWidget(view_mode_left_label_);
    slider_layout->addWidget(view_mode_slider_);
    slider_layout->addWidget(view_mode_right_label_);

    /*******************************************************/

    /*********************视角切换滑块***********************/

    view_Perspective_widget_ = new QWidget(central_container_);
    auto *slider_Perspective_layout = new QHBoxLayout(view_Perspective_widget_);

    view_2D_widget_ = new QWidget(central_container_);
    auto *slider_2D_layout = new QHBoxLayout(view_2D_widget_);

    slider_Perspective_layout->setContentsMargins(1, 1, 1, 1);//滑块模块内部留白
    slider_Perspective_layout->setSpacing(1);//滑块中各元素间距

    slider_2D_layout->setContentsMargins(1, 1, 1, 1);//滑块模块内部留白
    slider_2D_layout->setSpacing(1);//滑块中各元素间距

    // view_Perspective_left_label_ = new QLabel("左视图", view_Perspective_widget_);
    // view_Perspective_center_label_ = new QLabel("中视图", view_Perspective_widget_);
    // view_Perspective_right_label_ = new QLabel("右视图", view_Perspective_widget_);
    view_Perspective_slider_ = new QSlider(Qt::Horizontal, view_Perspective_widget_);//水平滑动条
    view_Perspective_slider_->setRange(0, 3);//滑块取值范围0-3
    view_Perspective_slider_->setValue(0);
    view_Perspective_slider_->setFixedHeight(35);

    view_2D_slider_ = new QSlider(Qt::Horizontal, view_2D_widget_);//水平滑动条
    view_2D_slider_->setRange(0, 2);//滑块取值范围0-2，0表示上视图，1表示左视图，2表示右视图
    view_2D_slider_->setValue(0);
    view_2D_slider_->setFixedHeight(35);

    //slider_Perspective_layout->addWidget(view_Perspective_left_label_);
    slider_Perspective_layout->addWidget(view_Perspective_slider_);
    //slider_Perspective_layout->addWidget(view_Perspective_right_label_);

    slider_2D_layout->addWidget(view_2D_slider_);

    /*******************************************************/

    top_status_bar_->raise();//确保悬浮控件在主场景视图上面
    log_panel_->raise();//确保日志区主场景视图上面
    attitude_panel_->raise();//确保姿态面板在主场景视图上面
    view_mode_widget_->raise();//确保视图模式控件在主场景视图上面
    view_Perspective_widget_->raise();//确保视角切换控件在主场景视图上面
    view_2D_widget_->raise();//确保2D视角切换控件在主场景视图上面

    view_Perspective_widget_->show();
    view_2D_widget_->hide();
}

void MainWindow::setupConnections()
{
    connect(clock_timer_, &QTimer::timeout, this, [this]() {//每秒触发刷新一次日志文本
        run_log_view_->hide();
        clock_timer_->stop();
    });

    //连接视图模式滑动条的值改变信号，根据值切换视图模式
    connect(view_mode_slider_, &QSlider::valueChanged, this, [this](int value) {
        if (value == 0)
        {
            scene_view_->setViewMode(ViewMode::Top2D);
            view_Perspective_widget_->hide();
            view_2D_widget_->show();
        }
        else
        {
            scene_view_->setViewMode(ViewMode::Pseudo3D);
            view_Perspective_widget_->show();
            view_2D_widget_->hide();
        }
    });

    //连接3D切换视角滑动条的值改变信号，根据值切换视图模式
    connect(view_Perspective_slider_, &QSlider::valueChanged, this, [this](int value) {
        if (value == 0)
        {
            scene_view_->setViewPerspectiveMode(ViewPerspectiveMode::Perspective225);
        }
        else if(value == 1)
        {
            scene_view_->setViewPerspectiveMode(ViewPerspectiveMode::Perspective315);
        }
        else if(value == 2)
        {
            scene_view_->setViewPerspectiveMode(ViewPerspectiveMode::Perspective45);
        }
        else if(value == 3)
        {
            scene_view_->setViewPerspectiveMode(ViewPerspectiveMode::Perspective135);
        }
    });

    //连接2D切换视角滑动条的值改变信号，根据值切换视图模式
    connect(view_2D_slider_, &QSlider::valueChanged, this, [this](int value) {
        if (value == 0)
        {
            scene_view_->setView2DMode(View2DMode::Perspectivetop);
        }
        else if(value == 1)
        {
            scene_view_->setView2DMode(View2DMode::Perspective90);
        }
        else if(value == 2)
        {
            scene_view_->setView2DMode(View2DMode::Perspective0);
        }
    });

    //连接顶部状态栏货架信息点击信号，打开货架信息窗口
    connect(top_status_bar_, &TopStatusBar::shelfButtonClicked, this, [this]() {
        //取出货物信息左下角的x，y坐标
        const QPoint button_bottom_left = top_status_bar_->shelfButtonBottomLeftGlobal();

        const int margin = 8;//让弹窗和顶部状态栏之间留一点空隙

        shelf_info_dialog_->adjustSize();
        shelf_info_dialog_->move(button_bottom_left.x(), button_bottom_left.y() + margin);//先把弹窗移动到货物信息下方
        shelf_info_dialog_->show();//显示弹窗
        shelf_info_dialog_->raise();//把弹窗提升到最前面
        shelf_info_dialog_->activateWindow();//让弹窗获取焦点，方便后续继续操作
    });

    connect(shelf_info_dialog_, &ShelfInfoDialog::slotDoubleClicked,
        this,
        [this](int shelf_index, const QString &side, int row, int col)
        {
            showShelfSlotImage(shelf_index, side, row, col);
        });

    /*********************ros通信相关***********************/
    
    if (ros_manager_)
    {
        connect(top_status_bar_, &TopStatusBar::executeButtonClicked, this, [this]() {
            triggerMissionUpload("button");
        });

        connect(top_status_bar_, &TopStatusBar::triggerTimeReached, this, [this](const QString &) {
            triggerMissionUpload("time");
        });

        //查看任务yaml上传服务返回的内容
        connect(ros_manager_, &RosManager::missionUploadFinished,
            this,
            [this](bool success, const QString &message, const QString &saved_path)
            {
                handleMissionUploadFinished(success, message, saved_path);
            },
            Qt::QueuedConnection);

        //查看起飞启动服务返回的内容
        connect(ros_manager_, &RosManager::commandResult,
            this,
            [this](bool success, const QString &message)
            {
                //根据命令执行结果的成功与否，更新界面上的结果标签文本，显示相关消息
                updateCommandResult(success, message);
                run_log_view_->appendPlainText(QString("%1").arg(message));
                //clock_timer_->start(5000);
            },
            Qt::QueuedConnection);

        //查看停止服务返回的内容
        connect(ros_manager_, &RosManager::stopcommandResult,
            this,
            [this](bool success, const QString &message)
            {
                if(success){
                    // path_ready_ = false;
                    // waiting_push_result_ = false;
                    // start_button_->setEnabled(false);
                    // delta_result_ = true;
                    //push_button_->setEnabled(true);
                    run_log_view_->appendPlainText(QString("%1").arg(message));
                    clock_timer_->start(5000);
                }
            },
            Qt::QueuedConnection);

        //查看offboard启动服务返回的内容
        connect(ros_manager_, &RosManager::offboardCommandResult,
            this,
            [this](bool success, const QString &message)
            {
                if(success){
                    //push_button_->setEnabled(false);
                    //waiting_task_result_ = false;
                    run_log_view_->appendPlainText(QString("%1").arg(message));
                    //clock_timer_->start(5000);
                }
            },
            Qt::QueuedConnection);












        //连接rosmanager发出的无人机状态信号
        connect(ros_manager_, &RosManager::statusUpdated,
            this,
            [this](bool connected, float battery_percent, int flight_mode, bool armed, const QString &task_name)
            {
                updateStatus(connected, battery_percent, flight_mode, armed, task_name);
            },
            Qt::QueuedConnection);

        //连接rosmanager发出的动作状态信号
        connect(ros_manager_, &RosManager::action_statusUpdated,
            this,
            [this](bool task_running, int action_step, int action_num, const QString &action_name)
            {
                action_updateStatus(task_running, action_step, action_num, action_name);
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

        connect(ros_manager_, &RosManager::visionBarcodeCaptured,
            this,
            [this](const QString &barcode, const QString &time_text)
            {
                appendVisionBarcodeCount(barcode, time_text);
            },
            Qt::QueuedConnection);

        //连接rosmanager发出的无人机位置信号
        connect(ros_manager_, &RosManager::positionUpdated,
            this,
            [this](double x, double y, double z, double qx, double qy, double qz, double qw)
            {
                // 当前仓储项目没有移植 position_view_，所以这里改成直接更新当前场景里的无人机位置。
                scene_data_.drone_state.pose.x = -1 * (y * 100 -150);
                scene_data_.drone_state.pose.y = -1 * (x * 100 -100);
                scene_data_.drone_state.pose.z = z;

                const double siny_cosp = 2.0 * (qw * qz + qx * qy);
                const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
                const double yaw = std::atan2(siny_cosp, cosy_cosp);          // 弧度
                const double yaw_deg = yaw * 180.0 / M_PI;                    // 角度

                scene_data_.drone_state.pose.yaw = yaw_deg;

                altitude_value_label_->setText(QString::number(z, 'f', 1) + " m");
                scene_view_->setSceneData(scene_data_);
            },
            Qt::QueuedConnection);

        //连接rosmanager发出的x,y偏移信号
        connect(ros_manager_, &RosManager::deltaUpdated,
            this,
            [this](double dx, double dy, double dyaw, bool valid)
            {
                updateDelta(dx, dy, dyaw, valid);
            },
            Qt::QueuedConnection);

        //链接判断控制程序返回内容
        connect(ros_manager_, &RosManager::pathReadyChanged,
            this,
            [this](bool ready)
            {
                updatePathReadyState(ready);
            },
            Qt::QueuedConnection);

        //连接rosmanager发出的会传路线数据信号
        connect(ros_manager_, &RosManager::returnWorldGroupUpdated,
            this,
            [this](const QVector<WorldCoord> &points)
            {
                updateWorldGroupState(points);
                ros_manager_->startTask();
                //run_log_view_->appendPlainText("初始化成功，准备巡检");
                //clock_timer_->start(5000);
            },
            Qt::QueuedConnection);
    }
}

/*********************ros移植部分***********************/

void MainWindow::triggerMissionUpload(const QString &trigger_source)
{
    Q_UNUSED(trigger_source);

    if (!ros_manager_)
    {
        run_log_view_->appendPlainText("初始化失败,rosmanager未就绪");
        //clock_timer_->start(5000);
        return;
    }

    if (mission_upload_in_progress_)
    {
        run_log_view_->appendPlainText("无法初始化");
        //clock_timer_->start(5000);
        return;
    }

    drone_msgs::msg::MissionSummary summary;
    summary.takeoff_altitude = 1.2;//起飞高度
    summary.move_altitude = 1.2;//移动高度
    summary.start_altitude = 0.0;//解锁高度
    summary.yaw = 0.0;//偏航角
    summary.tolerance = 0.12;//误差容忍
    summary.takeoff_hover_duration = 0.0;//起飞悬停时长
    summary.landing_hover_duration = 3.0;//降落悬停时长
    summary.move_hover_duration = 0.5;//移动悬停时长
    summary.add_hover_between_takeoff = true;//是否在起飞后添加悬停
    summary.add_hover_between_landing = true;//是否在降落前添加悬停
    summary.add_hover_between_moves = true;//是否在移动之间添加悬停
    summary.use_camera_aim = false;//是否开启相机
    summary.auto_start_mission = false;//是否自动启动任务
    summary.compress_straight_segments = false;//是否压缩直线段
    summary.frame = "world_body";

    summary.cam_tolerance = 10.0;
    summary.camera_aim_pid_p = 0.01;
    summary.camera_aim_pid_i = 0.00;
    summary.camera_aim_pid_d = 0.01;
    summary.camera_aim_target_timeout_s = 1.0;
    summary.camera_aim_stable_cycles = 15;
    summary.camera_aim_max_step = 0.05;
    summary.camera_aim_wait_first_targets_timeout_s = 8.0;
    summary.camera_aim_no_target_confirm_s = 3.0;
    summary.camera_aim_record_result_timeout_s = 10.0;
    summary.camera_aim_scan_point_timeout_s = 30.0;

    QVector<WorldCoord> path_points = {
        {140.0, 125.0},
        {140.0, -125.0},
        {0.0, -125.0},
        {0.0, 125.0}
    };

    mission_upload_in_progress_ = true;
    ros_manager_->uploadMissionSummary(path_points, summary);
}

void MainWindow::handleMissionUploadFinished(bool success, const QString &message, const QString &saved_path)
{
    Q_UNUSED(saved_path);

    mission_upload_in_progress_ = false;

    if (success)
    {
        run_log_view_->appendPlainText(QString("%1").arg(message));
        //clock_timer_->start(5000);
        ros_manager_->requestStartOffboard();
    }
    else if (!message.isEmpty())
    {
        run_log_view_->appendPlainText(QString("初始化失败：%1").arg(message));
        //clock_timer_->start(5000);
    }
    else
    {
        run_log_view_->appendPlainText("初始化失败");
        //clock_timer_->start(5000);
    }
}

void MainWindow::updateCommandResult(bool success, const QString &message)
{
    
}











bool MainWindow::updateStatus_connected()  const
{
    return connect_status;
}

void MainWindow::updateStatus(
    bool connected,
    float battery_percent,
    int flight_mode,
    bool armed,
    const QString &task_name)
{
    Q_UNUSED(armed);

    connect_status = connected;

    // 先把 ROS 返回的基础飞行状态同步到当前场景数据里。
    scene_data_.drone_state.connected = connected;
    scene_data_.drone_state.battery = battery_percent;
    scene_data_.drone_state.flight_mode = task_name;

    // 当前项目的 TopStatusBar 只直接展示“连接状态”和“任务文本”，
    // 所以这里先把最关键的信息映射过去。
    top_status_bar_->setConnected(connected);
    //top_status_bar_->setTaskText(task_name);

    // flight_mode 目前在当前仓储界面里没有单独的枚举显示控件，
    // 先把它拼进速度/航向旁的任务文本体系里，不额外造新控件。
    Q_UNUSED(flight_mode);

    scene_view_->setSceneData(scene_data_);



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
        //waiting_push_result_ = false;//重置等待上传结果的标志，允许下一次上传
        if (ros_manager_) {
            ros_manager_->stopTask();
            //run_log_view_->appendPlainText("巡检任务结束");
            top_status_bar_->setTaskText("任务待命");
        }
    }
}

void MainWindow::action_updateStatus(
    bool task_running,
    int action_step,
    int action_num,
    const QString &action_name)
{
    // 当前仓储界面没有原 drone_qt 那套 action 进度控件，
    // 这里先把 action 状态收敛成一条顶部任务文本，便于看到“当前执行到哪一步”。
    const QString task_text = task_running
        ? QString("%1 (%2/%3)").arg(action_name).arg(action_step).arg(action_num)
        : QString("任务待命");
    top_status_bar_->setTaskText(task_text);
}

void MainWindow::appendBarcodeRecord(
    const QString &barcode,
    const QByteArray &image_data,
    const QString &image_format,
    const QString &time_text)
{
    if (image_data.isEmpty())//图片数据为空就返回
    {
        return;
    }

    //根据当前无人机姿态推断槽位
    const SlotLocation location = resolveSlotFromPose(scene_data_.drone_state.pose);

    if (!location.valid)
    {
        run_log_view_->appendPlainText("收到图片，无法映射");
        //clock_timer_->start(5000);
        return;
    }

    //根据槽位索引找到对应的货物信息
    ShelfSlotItem *slot = findShelfSlot(location.shelf_index, location.side, location.row, location.col);
    if (!slot)
    {
        run_log_view_->appendPlainText("收到图片，目标货架槽位无效");
        //clock_timer_->start(5000);
        return;
    }

    //消息拆包
    const QStringList parts = barcode.split('|', Qt::KeepEmptyParts);
    if (parts.size() == 2)
    {
        slot->observed_category_id = parts[0].trimmed();
        slot->observed_package_id = parts[1].trimmed();
    }
    else
    {
        slot->observed_category_id.clear();
        slot->observed_package_id = barcode.trimmed();
    }

    slot->observed_time_text = time_text;

    slot->has_image = true;//标记这个槽位已经有图
    slot->latest_image.image_data = image_data;
    slot->latest_image.image_format = image_format;
    slot->latest_image.barcode = barcode;
    slot->latest_image.time_text = time_text;

    shelf_info_dialog_->setShelfPanelData(shelf_panel_data_);
    // run_log_view_->appendPlainText(
    //     QString("图片已绑定到货架%1 %2 R%3C%4")
    //         .arg(location.shelf_index + 1)
    //         .arg(location.side)
    //         .arg(location.row + 1)
    //         .arg(location.col + 1));
    // clock_timer_->start(5000);
}

void MainWindow::appendVisionBarcodeCount(
    const QString &barcode,
    const QString &time_text)
{
    Q_UNUSED(barcode);
    Q_UNUSED(time_text);

    // 当前轮次先不使用这条统计信号，只保留接口避免后续接线时再次改动。
}

ShelfSlotItem *MainWindow::findShelfSlot(int shelf_index, const QString &side, int row, int col)
{
    //根据货架索引、前后面、行列号，找到真正的 `ShelfSlotItem*`
    if (shelf_index < 0 || shelf_index >= shelf_panel_data_.size())
    {
        return nullptr;
    }

    if (row < 0 || row >= 4 || col < 0 || col >= 4)
    {
        return nullptr;
    }

    // 决定当前到底是在访问 front 还是 back
    QVector<ShelfSlotItem> *slot_list = nullptr;
    if (side == "front")
    {
        slot_list = &shelf_panel_data_[shelf_index].front_slots;
    }
    else
    {
        slot_list = &shelf_panel_data_[shelf_index].back_slots;
    }

    const int index = row * 4 + col;//行列转一维下标
    if (!slot_list || index < 0 || index >= slot_list->size())
    {
        return nullptr;
    }

    return &(*slot_list)[index];
}

SlotLocation MainWindow::resolveSlotFromPose(const Pose3D &pose) const
{
    SlotLocation location;//准备一个默认无效结果

    if (shelf_panel_data_.size() < 2)
    {
        return location;
    }

    if (((pose.x >= 110.0) && (pose.x <= 140.0) && (pose.yaw >= 80.0) && (pose.yaw <= 100.0)) ||
        ((pose.x >= -10.0) && (pose.x <= 10.0) && (pose.yaw >= -100.0) && (pose.yaw <= -80.0)))//第 1 个货架判定标准
    {
        location.shelf_index = 0;
    }
    else if (((pose.x >= -10.0) && (pose.x <= 10.0) && (pose.yaw >= 80.0) && (pose.yaw <= 100.0)) ||
        ((pose.x >= -140.0) && (pose.x <= -110.0) && (pose.yaw >= -100.0) && (pose.yaw <= -80.0)))//第 2 个货架判定标准
    {
        location.shelf_index = 1;
    }
    else
    {
        return location;
    }

    const double normalized_yaw = std::fmod(std::fmod(pose.yaw, 360.0) + 360.0, 360.0);//归一化 yaw
    location.side = (std::abs(normalized_yaw - 90.0) <= std::abs(normalized_yaw - 180.0)) ? "front" : "back";//比较离 90° 和 180° 哪个更近

    const double clamped_y = qBound(-100.0, pose.y, 50.0);//先把 `y` 限制到 `[-100, 50]`
    const double clamped_z = qBound(0.0, pose.z, 160.0);//先把 `z` 限制到 `[0, 160]`

    const double normalized_col = (clamped_y + 100.0) / 150.0;//把 `y` 从 `[-100, 50]` 映射到 `[0, 1]`
    const double normalized_row = clamped_z / 160.0;//把 `z` 从 `[0, 160]` 映射到 `[0, 1]`

    //判断行列
    location.col = qBound(0, static_cast<int>(normalized_col * 3.0), 2);
    location.row = qBound(0, static_cast<int>(normalized_row * 4.0), 3);
    location.valid = true;//标志成功定位
    return location;
}

void MainWindow::showShelfSlotImage(int shelf_index, const QString &side, int row, int col)
{
    ShelfSlotItem *slot = findShelfSlot(shelf_index, side, row, col);//找到对应的槽位
    //检查这个槽位是否真的有图
    if (!slot || !slot->has_image || slot->latest_image.image_data.isEmpty())
    {
        // run_log_view_->appendPlainText("当前槽位暂无图片");
        // clock_timer_->start(5000);
        return;
    }

    //处理照片
    QImage image;
    const QByteArray format_bytes = slot->latest_image.image_format.toUtf8();
    const char *format_ptr = format_bytes.isEmpty() ? nullptr : format_bytes.constData();
    if (!image.loadFromData(slot->latest_image.image_data, format_ptr))
    {
        // run_log_view_->appendPlainText("当前槽位图片解析失败");
        // clock_timer_->start(5000);
        return;
    }

    if (!image_preview_dialog_)
    {
        image_preview_dialog_ = new QDialog(this);//图片预览窗口本体
        image_preview_dialog_->resize(600, 400);

        auto *layout = new QVBoxLayout(image_preview_dialog_);//垂直布局，把里面的滚动区域铺进去
        auto *scroll_area = new QScrollArea(image_preview_dialog_);//滚动区域，避免大图把窗口撑爆
        image_preview_label_ = new QLabel(scroll_area);//真正承载图片的控件
        image_preview_label_->setAlignment(Qt::AlignCenter);
        scroll_area->setWidget(image_preview_label_);
        scroll_area->setWidgetResizable(true);
        layout->addWidget(scroll_area);
    }

    //写弹窗标题
    image_preview_dialog_->setWindowTitle(
        QString("货架%1 %2 R%3C%4 | %5 | %6")
            .arg(shelf_index + 1)
            .arg(side)
            .arg(row + 1)
            .arg(col + 1)
            .arg(slot->latest_image.barcode)
            .arg(slot->latest_image.time_text));

    //显示图片
    image_preview_label_->setPixmap(QPixmap::fromImage(image));
    image_preview_label_->adjustSize();
    image_preview_dialog_->show();
    image_preview_dialog_->raise();
    image_preview_dialog_->activateWindow();
}

void MainWindow::updateDelta(double dx, double dy, double dyaw, bool valid)
{
    Q_UNUSED(dx);
    Q_UNUSED(dy);
    Q_UNUSED(dyaw);
    Q_UNUSED(valid);

    // 当前仓储项目里还没有专门显示 dx / dy / dyaw 的面板。
    // 这里先保留空实现，表示 ROS delta 信号链已经接进来了，
    // 但由于缺少对应界面控件，暂时只做到“可编译、可继续扩展”，不瞎造显示位置。
}

void MainWindow::updatePathReadyState(bool ready)
{
    
}

void MainWindow::updateWorldGroupState(const QVector<WorldCoord> &points)
{
    
}

/******************************************************/

void MainWindow::applyWindowStyle()
{
    setStyleSheet(
        "QMainWindow, QWidget {"
        "background: #0c1018;"
        "color: #d7e3f4;"
        "}"
        "QSlider::groove:horizontal {"//滑块槽道样式
        "height: 30px;"//槽道高度
        "background: rgba(70, 90, 120, 120);"//槽道背景颜色
        "border-radius: 8px;"//槽道圆角
        "}"
        "QSlider::handle:horizontal {"//滑块样式
        "width: 40px;"//滑块宽度
        "height: 30px;"//滑块高度
        "background: #00c8ff;" //滑块颜色
        "margin: -6px 0;"//滑块垂直居中
        "border-radius: 8px;"//滑块圆角
        "}"
    );

    log_panel_->setStyleSheet(
        "background: rgba(18, 24, 34, 0);"//透明深色背景
        "border: none;"//标签无边框
        "border-radius: 10px;"

        "border: none;"//无边框
        "padding: 6px 10px;"//内边距
        "}"
    );

    attitude_panel_->setStyleSheet(
        "background: rgba(18, 24, 34, 100);"//半透明深色背景
        "border: 1px solid rgba(90, 130, 180, 100);"//边框颜色和透明度
        "border-radius: 10px;"

        "border: none;"//无边框
        "padding: 6px 10px;"//内边距
        "}"
    );

    view_mode_widget_->setStyleSheet(
        "background: rgba(18, 24, 34, 170);"
        //"border: 1px solid rgba(90, 130, 180, 120);"
        "border: none;"
        "border-radius: 10px;"
    );

    view_mode_left_label_->setStyleSheet(
        "border: none;"//标签无边框
        "font-size: 18px;"
    );

    view_mode_right_label_->setStyleSheet(
        "border: none;"
        "font-size: 18px;"
    );
}

void MainWindow::setupDemoData()
{

    /*********************无人机位置修改***********************/

    WarehouseSceneData data;

    data.drone_state.flight_mode = "OFFBOARD";

    data.drone_state.pose.z = 0.0;
    // ===== 临时测试坐标，联调完成后删掉 =====
    // data.drone_state.pose.x = 0.0;
    // data.drone_state.pose.y = -80.0;
    // data.drone_state.pose.z = 140;
    // data.drone_state.pose.yaw = 90.0;
    // =====================================
    data.drone_state.speed = 4.2;
    data.drone_state.battery = 87.0;

    scene_data_ = data;//先把演示数据复制到成员里，后面 ROS 实时更新时会直接改这份成员数据

    /*********************************************************/

    /**********************货架尺寸更改************************/

    //QRectF(-200, -120, 80, 160)：左上角x，左上角y，底面宽，底面长
    //height是货架的高度，name是货架的名字
    ShelfBlock shelf1;
    shelf1.base_rect = QRectF(-90, -100, 30, 150);
    shelf1.height = 160;
    shelf1.name = "A01";
    shelf1.color = ColorPalette::withAlpha(ColorPalette::BlueGrayDark, 180);
    data.shelves.push_back(shelf1);

    ShelfBlock shelf2;
    shelf2.base_rect = QRectF(60, -100, 30, 150);
    shelf2.height = 160;
    shelf2.name = "A02";
    shelf2.color = ColorPalette::withAlpha(ColorPalette::BlueGrayDark, 180);
    data.shelves.push_back(shelf2);

    /*********************************************************/

    /**********************轨迹位置更改************************/

    // data.trajectory.push_back({140, 125, 0});
    // data.trajectory.push_back({140, 125, 120});
    // data.trajectory.push_back({140, -125, 120});
    // data.trajectory.push_back({0, -125, 120});
    // data.trajectory.push_back({0, 125, 120});
    // data.trajectory.push_back({-140, 125, 120});
    // data.trajectory.push_back({-140, -125, 120});
    // data.trajectory.push_back({-140, -125, 0});

    /*********************************************************/

    scene_view_->setSceneData(data);
    scene_data_ = data;//把最终演示场景保存成成员，后面 ROS 位置/状态更新时直接在这份数据上改

    altitude_value_label_->setText(QString::number(data.drone_state.pose.z, 'f', 1) + " m");
    speed_value_label_->setText(QString::number(data.drone_state.speed, 'f', 1) + " m/s");
    yaw_value_label_->setText(QString::number(data.drone_state.pose.yaw, 'f', 1) + "°");

    top_status_bar_->setConnected(data.drone_state.connected);
    top_status_bar_->setTaskText("任务待命");


    /**********************货架信息更改************************/

    // 这里开始统一准备“货架弹窗”需要的全部演示数据。
    // 现在不再像之前那样拆成很多 QStringList，而是一个货架对应一个 ShelfPanelData。
    // 这样做的好处是：
    // 1. 所有假数据都集中在 MainWindow 里，后面查起来不容易乱。
    // 2. 一个货架的名称、前面点位、后面点位都装在一起，结构更清楚。
    // 3. 后面如果你要把假数据换成 ROS / 数据库 / 接口数据，也更容易整体替换。

    // -------------------- 货架1数据 --------------------
    shelf_panel_data_.clear();

    ShelfPanelData shelf1_panel;
    shelf1_panel.display_name = "货架1";//顶部按钮和弹窗标题里显示的名称
    shelf1_panel.button_status_color = "#00d48a";//顶部按钮前面的状态灯颜色，这里交给主窗口统一决定
    shelf1_panel.front_slots.resize(12);//前面固定16个点位，对应4x4网格
    shelf1_panel.back_slots.resize(12);//后面固定16个点位，对应4x4网格

    // 下面这些就是货架1的演示数据。
    // 下标规则是：index = row * 4 + col。
    // 例如：R1C1 -> 0，R2C3 -> 6，R3C2 -> 9。
    shelf1_panel.front_slots[0].category_id = "FOOD-001";
    shelf1_panel.front_slots[0].package_id = "PKG-0001";

    shelf1_panel.front_slots[6].category_id = "MED-002";
    shelf1_panel.front_slots[6].package_id = "PKG-0008";

    shelf1_panel.back_slots[9].category_id = "ELEC-003";
    shelf1_panel.back_slots[9].package_id = "PKG-0016";

    // -------------------- 货架2数据 --------------------
    ShelfPanelData shelf2_panel;
    shelf2_panel.display_name = "货架2";//第二个顶部按钮和标题显示名称
    shelf2_panel.button_status_color = "#f0b400";//第二个顶部按钮前面的状态灯颜色
    shelf2_panel.front_slots.resize(12);
    shelf2_panel.back_slots.resize(12);

    // 这里也给货架2先放一组演示数据，方便切换到第二个货架时能看出内容确实发生了变化。
    shelf2_panel.front_slots[3].category_id = "BOOK-005";
    shelf2_panel.front_slots[3].package_id = "PKG-0021";

    // 把两个货架的数据一起压进总列表。
    shelf_panel_data_.push_back(shelf1_panel);
    shelf_panel_data_.push_back(shelf2_panel);

    // 最后把整包货架弹窗数据一次性传给 ShelfInfoDialog。
    // 从这里开始，弹窗只负责显示，不再自己写任何假数据。
    shelf_info_dialog_->setShelfPanelData(shelf_panel_data_);

    /*********************************************************/

}

void MainWindow::resizeEvent(QResizeEvent *event)
{
    QMainWindow::resizeEvent(event);
    updateOverlayGeometry();//窗口大小改变时调整悬浮控件的位置和大小
}

void MainWindow::updateOverlayGeometry()
{
    if (!central_container_)
    {
        return;
    }

    const QRect area = central_container_->rect();

    const QPoint top_left = top_status_bar_->shelfButtonBottomLeftGlobal();

    scene_view_->setGeometry(area);//主场景视图占满整个主容器
    //左边距；上边距；宽度；高度
    top_status_bar_->setGeometry(20, 16, area.width() - 40, 52);
    log_panel_->setGeometry(5, top_left.y()+10, 310, 200);
    attitude_panel_->setGeometry(area.width() - 220, 84, 180, 120);
    view_mode_widget_->setGeometry(20, area.height() - 70, 160, 40);
    view_Perspective_widget_->setGeometry(area.width() - 220, area.height() - 70, 160, 40);
    view_2D_widget_->setGeometry(area.width() - 220, area.height() - 70, 160, 40);
}