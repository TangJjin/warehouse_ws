#include "drone_warehouse/parameter_config_dialog.hpp"

#include <QButtonGroup>
#include <QComboBox>
#include <QDoubleValidator>
#include <QFrame>
#include <QHBoxLayout>
#include <QHash>
#include <QIntValidator>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QStackedWidget>
#include <QVBoxLayout>
#include <QAbstractButton>
#include <QListWidgetItem>
#include <QMessageBox>

#include <cmath>

namespace
{
struct ProjectDefinition
{
    InspectionProject type;
    QString name;
};

const QVector<ProjectDefinition> projects = {
    //按钮为动态创建，添加列表即可
    {InspectionProject::Cargo, "货物巡检"},
    {InspectionProject::Animal, "动物巡检"}
};

QString boolDisplayText(bool value)
{
    return value ? "启用" : "关闭";
}

bool isBooleanParameter(const QString &id)
{
    static const QStringList ids = {
        "add_hover_between_takeoff", "add_hover_between_landing",
        "add_hover_between_moves", "auto_start_mission",
        "compress_waypoint_segments", "compress_non_waypoint_segments",
        "visual_servo.require_confirmed", "visual_servo.continue_on_timeout",
        "industrial_camera.auto_exposure",
        "industrial_camera.auto_exposure_priority",
        "industrial_camera.auto_white_balance",
        "industrial_camera.auto_focus"
    };
    return ids.contains(id);
}

bool isIntegerParameter(const QString &id)
{
    return id.startsWith("industrial_camera.") &&
           !isBooleanParameter(id);
}

bool isChoiceParameter(const QString &id)
{
    return isBooleanParameter(id) || id == "frame" ||
           id == "visual_servo.image_x_axis" ||
           id == "visual_servo.image_y_axis" ||
           id == "visual_servo.image_x_sign" ||
           id == "visual_servo.image_y_sign" ||
           id == "industrial_camera.power_line_frequency";
}

QString parameterDescription(const QString &id, const QString &fallback_name)
{
    static const QHash<QString, QString> descriptions = {
        {"takeoff_altitude", "任务文件中的起飞高度，单位为米。"},
        {"move_altitude", "生成货架巡检航点时使用的默认飞行高度，单位为米。"},
        {"start_altitude", "takeoff 动作的目标高度，单位为米。"},
        {"yaw", "生成任务时使用的默认航向角，单位为弧度。"},
        {"tolerance", "move 动作判定位置到达时允许的距离误差，单位为米。"},
        {"yaw_tolerance_deg", "move 动作判定航向到达时允许的角度误差。"},
        {"max_xy_speed_mps", "move 动作在水平 X/Y 方向的最大速度。"},
        {"max_z_speed_mps", "move 动作在竖直 Z 方向的最大速度。"},
        {"max_yaw_rate_deg_s", "move 动作允许的最大航向角速度。"},
        {"takeoff_hover_duration", "起飞后插入悬停动作时的持续时间。"},
        {"landing_hover_duration", "降落前插入悬停动作时的持续时间。"},
        {"move_hover_duration", "每个移动动作后插入悬停时的持续时间。"},
        {"add_hover_between_takeoff", "决定是否在起飞动作后加入悬停。"},
        {"add_hover_between_landing", "决定是否在降落动作前加入悬停。"},
        {"add_hover_between_moves", "决定是否在移动动作之间加入悬停。"},
        {"auto_start_mission", "任务上传成功后是否自动开始执行。"},
        {"compress_waypoint_segments", "是否压缩货架航点中的连续共线段。"},
        {"compress_non_waypoint_segments", "是否压缩非货架航点中的连续共线段。"},
        {"frame", "任务航点使用的参考坐标系。"},
        {"visual_servo.target_id", "指定需要跟踪的视觉目标 ID；留空时锁定首个符合条件的目标。"},
        {"visual_servo.require_confirmed", "启用后只接受视觉端已稳定确认的目标。"},
        {"visual_servo.image_x_axis", "图像水平误差映射到无人机机体系的哪个轴。"},
        {"visual_servo.image_y_axis", "图像垂直误差映射到无人机机体系的哪个轴，不能与图像 X 映射轴相同。"},
        {"visual_servo.image_x_sign", "图像水平误差映射到机体运动方向时使用的正负号。"},
        {"visual_servo.image_y_sign", "图像垂直误差映射到机体运动方向时使用的正负号。"},
        {"visual_servo.kp_x", "图像 X 误差的 PID 比例增益。"},
        {"visual_servo.ki_x", "图像 X 误差的 PID 积分增益。"},
        {"visual_servo.kd_x", "图像 X 误差的 PID 微分增益。"},
        {"visual_servo.kp_y", "图像 Y 误差的 PID 比例增益。"},
        {"visual_servo.ki_y", "图像 Y 误差的 PID 积分增益。"},
        {"visual_servo.kd_y", "图像 Y 误差的 PID 微分增益。"},
        {"visual_servo.integral_limit", "两轴积分累计量的绝对值上限，用于防止积分饱和。"},
        {"visual_servo.filter_alpha", "误差低通滤波系数，范围 0 到 1；越大响应越快。"},
        {"visual_servo.enter_tolerance_x", "图像 X 误差进入对准状态的阈值。"},
        {"visual_servo.enter_tolerance_y", "图像 Y 误差进入对准状态的阈值。"},
        {"visual_servo.exit_tolerance_x", "已对准后图像 X 误差退出对准状态的阈值。"},
        {"visual_servo.exit_tolerance_y", "已对准后图像 Y 误差退出对准状态的阈值。"},
        {"visual_servo.settle_time_s", "误差持续位于对准范围内多久后判定成功。"},
        {"visual_servo.acquire_timeout_s", "动作开始后等待首个有效目标的最长时间。"},
        {"visual_servo.lost_timeout_s", "跟踪过程中允许目标连续丢失的最长时间。"},
        {"visual_servo.overall_timeout_s", "单次视觉伺服动作允许执行的总时长。"},
        {"visual_servo.max_body_speed_mps", "视觉 PID 输出的单轴机体系速度上限。"},
        {"visual_servo.continue_on_timeout", "视觉伺服超时后是否继续执行后续任务动作。"},
        {"industrial_camera.auto_exposure", "自动曝光开关；启用时手动曝光时间不会写入相机。"},
        {"industrial_camera.exposure_absolute", "手动曝光时间，范围 1 到 10000。"},
        {"industrial_camera.auto_exposure_priority", "自动曝光时是否允许降低帧率来提高画面亮度。"},
        {"industrial_camera.gain", "图像增益，范围 0 到 190；过高会增加噪点。"},
        {"industrial_camera.brightness", "图像亮度处理值，范围 0 到 255。"},
        {"industrial_camera.contrast", "图像对比度，范围 0 到 128。"},
        {"industrial_camera.saturation", "图像色彩饱和度，范围 0 到 128。"},
        {"industrial_camera.gamma", "图像中间亮度校正值，范围 0 到 255。"},
        {"industrial_camera.sharpness", "图像锐化强度，范围 0 到 255。"},
        {"industrial_camera.backlight_compensation", "逆光补偿值，范围 16 到 160。"},
        {"industrial_camera.auto_white_balance", "自动白平衡开关；启用时手动色温不会写入相机。"},
        {"industrial_camera.white_balance_temperature", "手动白平衡色温，范围 2800 到 6500 K。"},
        {"industrial_camera.power_line_frequency", "防止灯光引起画面闪烁：关闭、50 Hz 或 60 Hz。"},
        {"industrial_camera.auto_focus", "自动对焦开关；启用时手动焦点不会写入相机。"},
        {"industrial_camera.focus_absolute", "手动焦点位置，范围 0 到 1023。"},
        {"industrial_camera.zoom_absolute", "相机变焦值，范围 100 到 200。"}
    };
    return descriptions.value(id, fallback_name + " 的配置值。");
}
}

ParameterConfigDialog::ParameterConfigDialog(
    const WarehouseConfig &config,
    QWidget *parent)
    : QDialog(parent),
      original_config_(config),
      working_config_(config)
{
    setWindowTitle("参数设置");
    setWindowFlag(Qt::FramelessWindowHint, true);

    buildUi();

    // 进入页面默认显示参数设置。
    selectMainPage(1);
}

void ParameterConfigDialog::buildUi()
{
    auto *root_layout = new QVBoxLayout(this);

    auto *top_layout = new QHBoxLayout;
    auto *page_title = new QLabel("智航参数", this);
    page_title->setObjectName("parameterPageTitle");
    top_layout->addWidget(page_title);
    top_layout->addStretch();

    restore_button_ = new QPushButton("恢复默认值", this);
    discard_button_ = new QPushButton("丢弃", this);
    apply_button_ = new QPushButton("启用", this);
    for (QPushButton *button : {restore_button_, discard_button_, apply_button_})
    {
        button->setMinimumSize(112, 44);
    }

    top_layout->addWidget(restore_button_);
    top_layout->addWidget(discard_button_);
    top_layout->addWidget(apply_button_);

    auto *body_layout = new QHBoxLayout;

    auto *left_panel = new QWidget(this);
    auto *left_layout = new QVBoxLayout(left_panel);
    left_panel->setObjectName("leftPanel");

    project_page_button_ = new QPushButton("项目选择", left_panel);
    parameter_page_button_ = new QPushButton("参数设置", left_panel);
    close_page_button_ = new QPushButton("关闭", left_panel);
    for (QPushButton *button :
         {project_page_button_, parameter_page_button_, close_page_button_})
    {
        button->setMinimumSize(150, 52);
    }

    //设置按钮为可选中
    project_page_button_->setCheckable(true);
    parameter_page_button_->setCheckable(true);
    close_page_button_->setCheckable(true);

    main_page_button_group_ = new QButtonGroup(this);//主界面按钮组
    main_page_button_group_->setExclusive(true);//设置按钮为互斥模式
    main_page_button_group_->addButton(project_page_button_);
    main_page_button_group_->addButton(parameter_page_button_);
    main_page_button_group_->addButton(close_page_button_);

    left_layout->addWidget(project_page_button_);
    left_layout->addWidget(parameter_page_button_);
    left_layout->addWidget(close_page_button_);
    left_layout->addStretch();

    //主界面堆叠窗口，用于切换项目选择和参数编辑页面
    page_stack_ = new QStackedWidget(this);
    buildProjectPage();//构建项目选择界面
    buildParameterPage();//构建参数修改界面

    body_layout->addWidget(left_panel);
    body_layout->addWidget(page_stack_, 1);

    root_layout->addLayout(top_layout);
    root_layout->addLayout(body_layout, 1);

    connect(project_page_button_, &QPushButton::clicked,
            this, [this]() { selectMainPage(0); });

    connect(parameter_page_button_, &QPushButton::clicked,
            this, [this]() { selectMainPage(1); });

    connect(close_page_button_, &QPushButton::clicked,
            this, &QDialog::reject);

    // connect(parameter_page_button_, &QPushButton::clicked,
    //         this, [this]() {dialog.exec()});

    connect(restore_button_, &QPushButton::clicked,
        this, [this]() {
            const WarehouseConfig defaults =
                createDefaultWarehouseConfig();

            working_config_.mission = defaults.mission;
            working_config_.visual_servo = defaults.visual_servo;
            working_config_.industrial_camera = defaults.industrial_camera;
            parameter_editor_->hide();
            rebuildParameterList();
        });

    connect(discard_button_, &QPushButton::clicked,
            this, [this]() {
                working_config_ = original_config_;
                reject();
            });

    connect(apply_button_, &QPushButton::clicked,
            this, &ParameterConfigDialog::handleApply);

    setStyleSheet(
        "QDialog { background: #101722; color: #d7e3f4; }"
        "#parameterPageTitle { font-size: 24px; font-weight: 600; color: #f1f6fb; }"
        "#leftPanel { background: #1b2736; border: 1px solid #34485e; border-radius: 6px; }"
        "QPushButton { min-height: 40px; padding: 0 16px; font-size: 17px;"
        "  color: #dce8f4; background: #26384b; border: 1px solid #47627d; border-radius: 6px; }"
        "QPushButton:hover { background: #31485f; }"
        "QPushButton:checked, QPushButton:pressed { color: #071018; background: #5bc0be; border-color: #76d4d1; }"
        "QListWidget { background: #111b27; border: 1px solid #30465c; font-size: 18px; }"
        "QListWidget::item:selected { background: #24485b; }"
        "QLabel { font-size: 17px; }"
        "#parameterEditor { background: #172332; border-left: 1px solid #40566d; }"
        "#parameterEditorTitle { font-size: 21px; font-weight: 600; color: #f1f6fb; }"
        "#parameterDescription { color: #b9c7d5; line-height: 1.4; }"
        "#parameterDefaultValue { color: #8fa3b7; }"
        "#parameterEditorClose { padding: 0; font-size: 24px; }"
        "QLineEdit, QComboBox { min-height: 42px; padding: 0 10px; font-size: 17px;"
        "  color: #eef5fb; background: #0d1620; border: 1px solid #526d87; border-radius: 4px; }"
        "QLineEdit:disabled, QComboBox:disabled { color: #6f8294; background: #151d26; }"
    );
}

const WarehouseConfig &
ParameterConfigDialog::savedConfig() const
{
    return working_config_;
}

void ParameterConfigDialog::selectMainPage(int page_index)
{
    project_page_button_->setChecked(page_index == 0);//设置按钮选中状态
    parameter_page_button_->setChecked(page_index == 1);
    page_stack_->setCurrentIndex(page_index);//切换堆叠窗口页面
}

void ParameterConfigDialog::buildProjectPage()
{
    project_page_ = new QWidget(page_stack_);

    auto *layout = new QVBoxLayout(project_page_);
    layout->setContentsMargins(24, 24, 24, 24);
    layout->setSpacing(12);

    auto *title_label = new QLabel("巡检项目", project_page_);
    layout->addWidget(title_label);

    project_button_group_ = new QButtonGroup(this);
    project_button_group_->setExclusive(true);//设置按钮为互斥模式

    for (const ProjectDefinition &project : projects)
    {
        //循环创建按钮，创建按钮时写入属性
        auto *button =
            new QPushButton(project.name, project_page_);

        button->setCheckable(true);
        button->setMinimumHeight(48);

        //按钮组保证巡检项目始终只有一个处于高亮状态
        project_button_group_->addButton(
            button,
            static_cast<int>(project.type));

        connect(button, &QPushButton::clicked,
                this,
                [this, type = project.type]() {
                    selectProject(type);
                });

        layout->addWidget(button);
    }

    layout->addStretch();
    page_stack_->addWidget(project_page_);

    // 首次打开时默认高亮货物巡检。
    selectProject(InspectionProject::Cargo);
}

void ParameterConfigDialog::selectProject(
    InspectionProject project)
{
    selected_project_ = project;

    //按照当前选择的巡检项目高亮对应按钮
    QAbstractButton *button =
        project_button_group_->button(
            static_cast<int>(project));

    if (button)
    {
        button->setChecked(true);
    }

    // 当前货物和动物暂时显示相同参数，后续再拆分。
    rebuildParameterList();
}

void ParameterConfigDialog::buildParameterPage()
{
    //参数编辑页面
    parameter_page_ = new QWidget(page_stack_);

    auto *layout = new QHBoxLayout(parameter_page_);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);

    parameter_list_ = new QListWidget(parameter_page_);//参数列表
    parameter_list_->setFrameShape(QFrame::NoFrame);//边框去除
    parameter_list_->setSpacing(0);//行间距
    parameter_list_->setHorizontalScrollBarPolicy(//只允许纵向滑动
        Qt::ScrollBarAlwaysOff);

    parameter_editor_ = new QWidget(parameter_page_);//参数编辑器
    parameter_editor_->setObjectName("parameterEditor");
    parameter_editor_->setFixedWidth(420);

    auto *editor_layout = new QVBoxLayout(parameter_editor_);
    editor_layout->setContentsMargins(24, 20, 24, 24);
    editor_layout->setSpacing(16);

    auto *editor_header = new QHBoxLayout;
    editor_title_label_ = new QLabel(parameter_editor_);
    editor_title_label_->setObjectName("parameterEditorTitle");
    auto *editor_close_button = new QPushButton("×", parameter_editor_);
    editor_close_button->setObjectName("parameterEditorClose");
    editor_close_button->setFixedSize(40, 40);
    editor_header->addWidget(editor_title_label_, 1);
    editor_header->addWidget(editor_close_button);

    editor_description_label_ = new QLabel(parameter_editor_);
    editor_description_label_->setWordWrap(true);
    editor_description_label_->setObjectName("parameterDescription");

    auto *value_title = new QLabel("修改值", parameter_editor_);
    editor_input_stack_ = new QStackedWidget(parameter_editor_);
    editor_line_edit_ = new QLineEdit(editor_input_stack_);
    editor_line_edit_->setMinimumHeight(44);
    editor_combo_box_ = new QComboBox(editor_input_stack_);
    editor_combo_box_->setMinimumHeight(44);
    editor_input_stack_->addWidget(editor_line_edit_);
    editor_input_stack_->addWidget(editor_combo_box_);

    editor_default_label_ = new QLabel(parameter_editor_);
    editor_default_label_->setObjectName("parameterDefaultValue");
    editor_default_label_->setWordWrap(true);

    editor_confirm_button_ = new QPushButton("确定", parameter_editor_);
    editor_confirm_button_->setMinimumHeight(44);

    editor_layout->addLayout(editor_header);
    editor_layout->addWidget(editor_description_label_);
    editor_layout->addSpacing(8);
    editor_layout->addWidget(value_title);
    editor_layout->addWidget(editor_input_stack_);
    editor_layout->addWidget(editor_default_label_);
    editor_layout->addStretch();
    editor_layout->addWidget(editor_confirm_button_);
    parameter_editor_->hide();//选择参数前不显示右侧编辑区域

    layout->addWidget(parameter_list_, 1);
    layout->addWidget(parameter_editor_);

    page_stack_->addWidget(parameter_page_);

    connect(parameter_list_, &QListWidget::itemClicked,
            this, &ParameterConfigDialog::showParameterEditor);
    connect(editor_close_button, &QPushButton::clicked,
            this, [this]() {
                editing_parameter_id_.clear();
                parameter_list_->clearSelection();
                parameter_editor_->hide();
            });
    connect(editor_confirm_button_, &QPushButton::clicked,
            this, &ParameterConfigDialog::commitParameterEdit);
    connect(editor_line_edit_, &QLineEdit::returnPressed,
            this, &ParameterConfigDialog::commitParameterEdit);

    rebuildParameterList();
}

void ParameterConfigDialog::rebuildParameterList()
{
    if (!parameter_list_)
    {
        return;
    }

    parameter_list_->clear();

    const MissionConfig &mission =
        working_config_.mission;

    addParameterRow(
        "takeoff_altitude",
        "起飞高度",
        QString("%1 m").arg(
            mission.takeoff_altitude, 0, 'f', 2));

    addParameterRow(
        "move_altitude",
        "移动高度",
        QString("%1 m").arg(
            mission.move_altitude, 0, 'f', 2));

    addParameterRow(
        "start_altitude",
        "任务起始高度",
        QString("%1 m").arg(
            mission.start_altitude, 0, 'f', 2));

    addParameterRow(
        "yaw",
        "默认任务航向",
        QString("%1 rad").arg(
            mission.yaw, 0, 'f', 2));

    addParameterRow(
        "tolerance",
        "航点容差",
        QString("%1 m").arg(
            mission.tolerance, 0, 'f', 2));
    addParameterRow(
        "yaw_tolerance_deg",
        "航向到达容差",
        QString("%1 deg").arg(mission.yaw_tolerance_deg, 0, 'f', 2));

    addParameterRow(
        "max_xy_speed_mps",
        "最大水平速度",
        QString("%1 m/s").arg(mission.max_xy_speed_mps, 0, 'f', 2));

    addParameterRow(
        "max_z_speed_mps",
        "最大垂直速度",
        QString("%1 m/s").arg(mission.max_z_speed_mps, 0, 'f', 2));

    addParameterRow(
        "max_yaw_rate_deg_s",
        "最大航向角速度",
        QString("%1 deg/s").arg(mission.max_yaw_rate_deg_s, 0, 'f', 2));

    addParameterRow("takeoff_hover_duration", "起飞后悬停时间",
                    QString("%1 s").arg(mission.takeoff_hover_duration, 0, 'f', 2));
    addParameterRow("landing_hover_duration", "降落前悬停时间",
                    QString("%1 s").arg(mission.landing_hover_duration, 0, 'f', 2));
    addParameterRow("move_hover_duration", "移动后悬停时间",
                    QString("%1 s").arg(mission.move_hover_duration, 0, 'f', 2));
    addParameterRow("add_hover_between_takeoff", "起飞后加入悬停",
                    boolDisplayText(mission.add_hover_between_takeoff));
    addParameterRow("add_hover_between_landing", "降落前加入悬停",
                    boolDisplayText(mission.add_hover_between_landing));
    addParameterRow("add_hover_between_moves", "移动之间加入悬停",
                    boolDisplayText(mission.add_hover_between_moves));
    addParameterRow("auto_start_mission", "上传后自动启动",
                    boolDisplayText(mission.auto_start_mission));
    addParameterRow("compress_waypoint_segments", "压缩货架航点直线段",
                    boolDisplayText(mission.compress_waypoint_segments));
    addParameterRow("compress_non_waypoint_segments", "压缩其他航点直线段",
                    boolDisplayText(mission.compress_non_waypoint_segments));
    addParameterRow("frame", "任务坐标系", mission.frame);

    // Visual-servo defaults used only when a visual_servo action is present.
    const VisualServoConfig &visual = working_config_.visual_servo;
    auto addNumber = [this](const QString &id, const QString &name,
                            double value, const QString &unit = QString()) {
        const QString suffix = unit.isEmpty() ? QString() : " " + unit;
        addParameterRow(id, name, QString::number(value, 'f', 3) + suffix);
    };
    addParameterRow("visual_servo.target_id", "视觉目标 ID",
                    visual.target_id.isEmpty() ? "自动锁定" : visual.target_id);
    addParameterRow("visual_servo.require_confirmed", "要求稳定确认",
                    boolDisplayText(visual.require_confirmed));
    addParameterRow("visual_servo.image_x_axis", "图像 X 映射轴", visual.image_x_axis);
    addParameterRow("visual_servo.image_y_axis", "图像 Y 映射轴", visual.image_y_axis);
    addNumber("visual_servo.image_x_sign", "图像 X 方向", visual.image_x_sign);
    addNumber("visual_servo.image_y_sign", "图像 Y 方向", visual.image_y_sign);
    addNumber("visual_servo.kp_x", "图像 X 比例增益", visual.kp_x);
    addNumber("visual_servo.ki_x", "图像 X 积分增益", visual.ki_x);
    addNumber("visual_servo.kd_x", "图像 X 微分增益", visual.kd_x);
    addNumber("visual_servo.kp_y", "图像 Y 比例增益", visual.kp_y);
    addNumber("visual_servo.ki_y", "图像 Y 积分增益", visual.ki_y);
    addNumber("visual_servo.kd_y", "图像 Y 微分增益", visual.kd_y);
    addNumber("visual_servo.integral_limit", "积分上限", visual.integral_limit);
    addNumber("visual_servo.filter_alpha", "低通滤波系数", visual.filter_alpha);
    addNumber("visual_servo.enter_tolerance_x", "X 进入容差", visual.enter_tolerance_x);
    addNumber("visual_servo.enter_tolerance_y", "Y 进入容差", visual.enter_tolerance_y);
    addNumber("visual_servo.exit_tolerance_x", "X 退出容差", visual.exit_tolerance_x);
    addNumber("visual_servo.exit_tolerance_y", "Y 退出容差", visual.exit_tolerance_y);
    addNumber("visual_servo.settle_time_s", "稳定持续时间", visual.settle_time_s, "s");
    addNumber("visual_servo.acquire_timeout_s", "目标获取超时", visual.acquire_timeout_s, "s");
    addNumber("visual_servo.lost_timeout_s", "目标丢失超时", visual.lost_timeout_s, "s");
    addNumber("visual_servo.overall_timeout_s", "动作总超时", visual.overall_timeout_s, "s");
    addNumber("visual_servo.max_body_speed_mps", "机体系最大速度",
              visual.max_body_speed_mps, "m/s");
    addParameterRow("visual_servo.continue_on_timeout", "超时后继续任务",
                    boolDisplayText(visual.continue_on_timeout));

    // Industrial-camera values are always published as one complete message.
    const IndustrialCameraConfig &camera = working_config_.industrial_camera;
    addParameterRow("industrial_camera.auto_exposure", "自动曝光",
                    boolDisplayText(camera.auto_exposure));
    addParameterRow("industrial_camera.exposure_absolute", "手动曝光时间",
                    QString::number(camera.exposure_absolute));
    addParameterRow("industrial_camera.auto_exposure_priority", "自动曝光帧率优先",
                    boolDisplayText(camera.auto_exposure_priority));
    addParameterRow("industrial_camera.gain", "相机增益", QString::number(camera.gain));
    addParameterRow("industrial_camera.brightness", "亮度", QString::number(camera.brightness));
    addParameterRow("industrial_camera.contrast", "对比度", QString::number(camera.contrast));
    addParameterRow("industrial_camera.saturation", "饱和度", QString::number(camera.saturation));
    addParameterRow("industrial_camera.gamma", "Gamma", QString::number(camera.gamma));
    addParameterRow("industrial_camera.sharpness", "锐度", QString::number(camera.sharpness));
    addParameterRow("industrial_camera.backlight_compensation", "逆光补偿",
                    QString::number(camera.backlight_compensation));
    addParameterRow("industrial_camera.auto_white_balance", "自动白平衡",
                    boolDisplayText(camera.auto_white_balance));
    addParameterRow("industrial_camera.white_balance_temperature", "白平衡色温",
                    QString("%1 K").arg(camera.white_balance_temperature));
    addParameterRow("industrial_camera.power_line_frequency", "防闪烁频率",
                    camera.power_line_frequency == 0 ? "关闭" :
                    QString("%1 Hz").arg(camera.power_line_frequency == 1 ? 50 : 60));
    addParameterRow("industrial_camera.auto_focus", "自动对焦",
                    boolDisplayText(camera.auto_focus));
    addParameterRow("industrial_camera.focus_absolute", "手动焦点",
                    QString::number(camera.focus_absolute));
    addParameterRow("industrial_camera.zoom_absolute", "变焦值",
                    QString::number(camera.zoom_absolute));
}

void ParameterConfigDialog::addParameterRow(
    const QString &parameter_id,
    const QString &display_name,
    const QString &display_value)
{
    auto *item = new QListWidgetItem(parameter_list_);
    item->setData(Qt::UserRole, parameter_id);
    item->setData(Qt::UserRole + 1, display_name);
    item->setFlags(
        Qt::ItemIsEnabled |
        Qt::ItemIsSelectable);
    item->setSizeHint(QSize(0, 58));

    auto *row_widget = new QWidget(parameter_list_);
    row_widget->setAttribute(
        Qt::WA_TransparentForMouseEvents);

    auto *row_layout = new QVBoxLayout(row_widget);
    row_layout->setContentsMargins(16, 0, 16, 0);
    row_layout->setSpacing(0);

    auto *value_layout = new QHBoxLayout;
    value_layout->setContentsMargins(0, 0, 0, 0);

    auto *name_label =
        new QLabel(display_name, row_widget);
    auto *value_label =
        new QLabel(display_value, row_widget);

    value_label->setAlignment(
        Qt::AlignRight | Qt::AlignVCenter);

    value_layout->addWidget(name_label);
    value_layout->addStretch();
    value_layout->addWidget(value_label);

    auto *separator = new QFrame(row_widget);
    separator->setFrameShape(QFrame::HLine);
    separator->setFrameShadow(QFrame::Plain);

    row_layout->addLayout(value_layout, 1);
    row_layout->addWidget(separator);

    parameter_list_->setItemWidget(item, row_widget);
}

void ParameterConfigDialog::showParameterEditor(QListWidgetItem *item)
{
    if (!item)
    {
        return;
    }

    editing_parameter_id_ = item->data(Qt::UserRole).toString();
    const QString display_name = item->data(Qt::UserRole + 1).toString();
    const QString current_value =
        parameterRawValue(working_config_, editing_parameter_id_);
    const QString default_value =
        parameterRawValue(createDefaultWarehouseConfig(), editing_parameter_id_);

    editor_title_label_->setText(display_name);
    editor_description_label_->setText(
        parameterDescription(editing_parameter_id_, display_name));
    editor_default_label_->setText("默认值：" +
                                   (default_value.isEmpty() ? "空" : default_value));
    editor_confirm_button_->setEnabled(true);
    editor_line_edit_->setEnabled(true);
    editor_combo_box_->setEnabled(true);

    if (isChoiceParameter(editing_parameter_id_))
    {
        editor_combo_box_->clear();
        if (isBooleanParameter(editing_parameter_id_))
        {
            editor_combo_box_->addItem("启用", "true");
            editor_combo_box_->addItem("关闭", "false");
        }
        else if (editing_parameter_id_ == "frame")
        {
            editor_combo_box_->addItem("world_body", "world_body");
            editor_combo_box_->addItem("body", "body");
            editor_combo_box_->addItem("world_enu", "world_enu");
        }
        else if (editing_parameter_id_.endsWith("_axis"))
        {
            editor_combo_box_->addItem("X 轴", "x");
            editor_combo_box_->addItem("Y 轴", "y");
            editor_combo_box_->addItem("Z 轴", "z");
        }
        else if (editing_parameter_id_.endsWith("_sign"))
        {
            editor_combo_box_->addItem("正向 (+1)", "1");
            editor_combo_box_->addItem("反向 (-1)", "-1");
        }
        else
        {
            editor_combo_box_->addItem("关闭", "0");
            editor_combo_box_->addItem("50 Hz", "1");
            editor_combo_box_->addItem("60 Hz", "2");
        }

        const int current_index = editor_combo_box_->findData(current_value);
        editor_combo_box_->setCurrentIndex(current_index >= 0 ? current_index : 0);
        editor_input_stack_->setCurrentWidget(editor_combo_box_);
    }
    else
    {
        editor_line_edit_->clear();
        editor_line_edit_->setText(current_value);
        editor_line_edit_->setPlaceholderText(
            editing_parameter_id_ == "visual_servo.target_id"
                ? "留空时自动锁定首个目标"
                : QString());
        editor_input_stack_->setCurrentWidget(editor_line_edit_);
        editor_line_edit_->selectAll();
        editor_line_edit_->setFocus();
    }

    // 自动模式开启时保留手动值，但不允许从界面改动它。
    const bool manual_value_disabled =
        (editing_parameter_id_ == "industrial_camera.exposure_absolute" &&
         working_config_.industrial_camera.auto_exposure) ||
        (editing_parameter_id_ == "industrial_camera.white_balance_temperature" &&
         working_config_.industrial_camera.auto_white_balance) ||
        (editing_parameter_id_ == "industrial_camera.focus_absolute" &&
         working_config_.industrial_camera.auto_focus);
    if (manual_value_disabled)
    {
        editor_line_edit_->setEnabled(false);
        editor_confirm_button_->setEnabled(false);
        editor_description_label_->setText(
            editor_description_label_->text() +
            " 当前对应的自动模式已启用，请先关闭自动模式。" );
    }

    parameter_editor_->show();
}
QString ParameterConfigDialog::parameterRawValue(
    const WarehouseConfig &config,
    const QString &id) const
{
    const MissionConfig &mission = config.mission;
    if (id == "takeoff_altitude") return QString::number(mission.takeoff_altitude, 'g', 15);
    if (id == "move_altitude") return QString::number(mission.move_altitude, 'g', 15);
    if (id == "start_altitude") return QString::number(mission.start_altitude, 'g', 15);
    if (id == "yaw") return QString::number(mission.yaw, 'g', 15);
    if (id == "tolerance") return QString::number(mission.tolerance, 'g', 15);
    if (id == "yaw_tolerance_deg") return QString::number(mission.yaw_tolerance_deg, 'g', 15);
    if (id == "max_xy_speed_mps") return QString::number(mission.max_xy_speed_mps, 'g', 15);
    if (id == "max_z_speed_mps") return QString::number(mission.max_z_speed_mps, 'g', 15);
    if (id == "max_yaw_rate_deg_s") return QString::number(mission.max_yaw_rate_deg_s, 'g', 15);
    if (id == "takeoff_hover_duration") return QString::number(mission.takeoff_hover_duration, 'g', 15);
    if (id == "landing_hover_duration") return QString::number(mission.landing_hover_duration, 'g', 15);
    if (id == "move_hover_duration") return QString::number(mission.move_hover_duration, 'g', 15);
    if (id == "add_hover_between_takeoff") return mission.add_hover_between_takeoff ? "true" : "false";
    if (id == "add_hover_between_landing") return mission.add_hover_between_landing ? "true" : "false";
    if (id == "add_hover_between_moves") return mission.add_hover_between_moves ? "true" : "false";
    if (id == "auto_start_mission") return mission.auto_start_mission ? "true" : "false";
    if (id == "compress_waypoint_segments") return mission.compress_waypoint_segments ? "true" : "false";
    if (id == "compress_non_waypoint_segments") return mission.compress_non_waypoint_segments ? "true" : "false";
    if (id == "frame") return mission.frame;

    const VisualServoConfig &visual = config.visual_servo;
    if (id == "visual_servo.target_id") return visual.target_id;
    if (id == "visual_servo.require_confirmed") return visual.require_confirmed ? "true" : "false";
    if (id == "visual_servo.image_x_axis") return visual.image_x_axis;
    if (id == "visual_servo.image_y_axis") return visual.image_y_axis;
    if (id == "visual_servo.image_x_sign") return QString::number(visual.image_x_sign, 'g', 15);
    if (id == "visual_servo.image_y_sign") return QString::number(visual.image_y_sign, 'g', 15);
    if (id == "visual_servo.kp_x") return QString::number(visual.kp_x, 'g', 15);
    if (id == "visual_servo.ki_x") return QString::number(visual.ki_x, 'g', 15);
    if (id == "visual_servo.kd_x") return QString::number(visual.kd_x, 'g', 15);
    if (id == "visual_servo.kp_y") return QString::number(visual.kp_y, 'g', 15);
    if (id == "visual_servo.ki_y") return QString::number(visual.ki_y, 'g', 15);
    if (id == "visual_servo.kd_y") return QString::number(visual.kd_y, 'g', 15);
    if (id == "visual_servo.integral_limit") return QString::number(visual.integral_limit, 'g', 15);
    if (id == "visual_servo.filter_alpha") return QString::number(visual.filter_alpha, 'g', 15);
    if (id == "visual_servo.enter_tolerance_x") return QString::number(visual.enter_tolerance_x, 'g', 15);
    if (id == "visual_servo.enter_tolerance_y") return QString::number(visual.enter_tolerance_y, 'g', 15);
    if (id == "visual_servo.exit_tolerance_x") return QString::number(visual.exit_tolerance_x, 'g', 15);
    if (id == "visual_servo.exit_tolerance_y") return QString::number(visual.exit_tolerance_y, 'g', 15);
    if (id == "visual_servo.settle_time_s") return QString::number(visual.settle_time_s, 'g', 15);
    if (id == "visual_servo.acquire_timeout_s") return QString::number(visual.acquire_timeout_s, 'g', 15);
    if (id == "visual_servo.lost_timeout_s") return QString::number(visual.lost_timeout_s, 'g', 15);
    if (id == "visual_servo.overall_timeout_s") return QString::number(visual.overall_timeout_s, 'g', 15);
    if (id == "visual_servo.max_body_speed_mps") return QString::number(visual.max_body_speed_mps, 'g', 15);
    if (id == "visual_servo.continue_on_timeout") return visual.continue_on_timeout ? "true" : "false";

    const IndustrialCameraConfig &camera = config.industrial_camera;
    if (id == "industrial_camera.auto_exposure") return camera.auto_exposure ? "true" : "false";
    if (id == "industrial_camera.exposure_absolute") return QString::number(camera.exposure_absolute);
    if (id == "industrial_camera.auto_exposure_priority") return camera.auto_exposure_priority ? "true" : "false";
    if (id == "industrial_camera.gain") return QString::number(camera.gain);
    if (id == "industrial_camera.brightness") return QString::number(camera.brightness);
    if (id == "industrial_camera.contrast") return QString::number(camera.contrast);
    if (id == "industrial_camera.saturation") return QString::number(camera.saturation);
    if (id == "industrial_camera.gamma") return QString::number(camera.gamma);
    if (id == "industrial_camera.sharpness") return QString::number(camera.sharpness);
    if (id == "industrial_camera.backlight_compensation") return QString::number(camera.backlight_compensation);
    if (id == "industrial_camera.auto_white_balance") return camera.auto_white_balance ? "true" : "false";
    if (id == "industrial_camera.white_balance_temperature") return QString::number(camera.white_balance_temperature);
    if (id == "industrial_camera.power_line_frequency") return QString::number(camera.power_line_frequency);
    if (id == "industrial_camera.auto_focus") return camera.auto_focus ? "true" : "false";
    if (id == "industrial_camera.focus_absolute") return QString::number(camera.focus_absolute);
    if (id == "industrial_camera.zoom_absolute") return QString::number(camera.zoom_absolute);
    return {};
}
bool ParameterConfigDialog::updateParameterFromEditor(QString *error_message)
{
    if (editing_parameter_id_.isEmpty())
    {
        if (error_message) *error_message = "没有选中参数";
        return false;
    }

    const QString id = editing_parameter_id_;
    const QString raw_value = isChoiceParameter(id)
        ? editor_combo_box_->currentData().toString()
        : editor_line_edit_->text().trimmed();
    WarehouseConfig candidate = working_config_;

    if (isBooleanParameter(id))
    {
        const bool value = raw_value == "true";
        if (id == "add_hover_between_takeoff") candidate.mission.add_hover_between_takeoff = value;
        else if (id == "add_hover_between_landing") candidate.mission.add_hover_between_landing = value;
        else if (id == "add_hover_between_moves") candidate.mission.add_hover_between_moves = value;
        else if (id == "auto_start_mission") candidate.mission.auto_start_mission = value;
        else if (id == "compress_waypoint_segments") candidate.mission.compress_waypoint_segments = value;
        else if (id == "compress_non_waypoint_segments") candidate.mission.compress_non_waypoint_segments = value;
        else if (id == "visual_servo.require_confirmed") candidate.visual_servo.require_confirmed = value;
        else if (id == "visual_servo.continue_on_timeout") candidate.visual_servo.continue_on_timeout = value;
        else if (id == "industrial_camera.auto_exposure") candidate.industrial_camera.auto_exposure = value;
        else if (id == "industrial_camera.auto_exposure_priority") candidate.industrial_camera.auto_exposure_priority = value;
        else if (id == "industrial_camera.auto_white_balance") candidate.industrial_camera.auto_white_balance = value;
        else if (id == "industrial_camera.auto_focus") candidate.industrial_camera.auto_focus = value;
    }
    else if (id == "frame") candidate.mission.frame = raw_value;
    else if (id == "visual_servo.target_id") candidate.visual_servo.target_id = raw_value;
    else if (id == "visual_servo.image_x_axis") candidate.visual_servo.image_x_axis = raw_value;
    else if (id == "visual_servo.image_y_axis") candidate.visual_servo.image_y_axis = raw_value;
    else if (id == "industrial_camera.power_line_frequency")
    {
        bool ok = false;
        const int value = raw_value.toInt(&ok);
        if (!ok || value < 0 || value > 2)
        {
            if (error_message) *error_message = "防闪烁频率无效";
            return false;
        }
        candidate.industrial_camera.power_line_frequency = static_cast<quint8>(value);
    }
    else if (isIntegerParameter(id))
    {
        bool ok = false;
        const int value = raw_value.toInt(&ok);
        if (!ok)
        {
            if (error_message) *error_message = "请输入有效整数";
            return false;
        }
        if (id == "industrial_camera.exposure_absolute") candidate.industrial_camera.exposure_absolute = value;
        else if (id == "industrial_camera.gain") candidate.industrial_camera.gain = value;
        else if (id == "industrial_camera.brightness") candidate.industrial_camera.brightness = value;
        else if (id == "industrial_camera.contrast") candidate.industrial_camera.contrast = value;
        else if (id == "industrial_camera.saturation") candidate.industrial_camera.saturation = value;
        else if (id == "industrial_camera.gamma") candidate.industrial_camera.gamma = value;
        else if (id == "industrial_camera.sharpness") candidate.industrial_camera.sharpness = value;
        else if (id == "industrial_camera.backlight_compensation") candidate.industrial_camera.backlight_compensation = value;
        else if (id == "industrial_camera.white_balance_temperature") candidate.industrial_camera.white_balance_temperature = value;
        else if (id == "industrial_camera.focus_absolute") candidate.industrial_camera.focus_absolute = value;
        else if (id == "industrial_camera.zoom_absolute") candidate.industrial_camera.zoom_absolute = value;
    }
    else
    {
        bool ok = false;
        const double value = raw_value.toDouble(&ok);
        if (!ok || !std::isfinite(value))
        {
            if (error_message) *error_message = "请输入有效有限数值";
            return false;
        }

        if (id == "takeoff_altitude") candidate.mission.takeoff_altitude = value;
        else if (id == "move_altitude") candidate.mission.move_altitude = value;
        else if (id == "start_altitude") candidate.mission.start_altitude = value;
        else if (id == "yaw") candidate.mission.yaw = value;
        else if (id == "tolerance") candidate.mission.tolerance = value;
        else if (id == "yaw_tolerance_deg") candidate.mission.yaw_tolerance_deg = value;
        else if (id == "max_xy_speed_mps") candidate.mission.max_xy_speed_mps = value;
        else if (id == "max_z_speed_mps") candidate.mission.max_z_speed_mps = value;
        else if (id == "max_yaw_rate_deg_s") candidate.mission.max_yaw_rate_deg_s = value;
        else if (id == "takeoff_hover_duration") candidate.mission.takeoff_hover_duration = value;
        else if (id == "landing_hover_duration") candidate.mission.landing_hover_duration = value;
        else if (id == "move_hover_duration") candidate.mission.move_hover_duration = value;
        else if (id == "visual_servo.image_x_sign") candidate.visual_servo.image_x_sign = value;
        else if (id == "visual_servo.image_y_sign") candidate.visual_servo.image_y_sign = value;
        else if (id == "visual_servo.kp_x") candidate.visual_servo.kp_x = value;
        else if (id == "visual_servo.ki_x") candidate.visual_servo.ki_x = value;
        else if (id == "visual_servo.kd_x") candidate.visual_servo.kd_x = value;
        else if (id == "visual_servo.kp_y") candidate.visual_servo.kp_y = value;
        else if (id == "visual_servo.ki_y") candidate.visual_servo.ki_y = value;
        else if (id == "visual_servo.kd_y") candidate.visual_servo.kd_y = value;
        else if (id == "visual_servo.integral_limit") candidate.visual_servo.integral_limit = value;
        else if (id == "visual_servo.filter_alpha") candidate.visual_servo.filter_alpha = value;
        else if (id == "visual_servo.enter_tolerance_x") candidate.visual_servo.enter_tolerance_x = value;
        else if (id == "visual_servo.enter_tolerance_y") candidate.visual_servo.enter_tolerance_y = value;
        else if (id == "visual_servo.exit_tolerance_x") candidate.visual_servo.exit_tolerance_x = value;
        else if (id == "visual_servo.exit_tolerance_y") candidate.visual_servo.exit_tolerance_y = value;
        else if (id == "visual_servo.settle_time_s") candidate.visual_servo.settle_time_s = value;
        else if (id == "visual_servo.acquire_timeout_s") candidate.visual_servo.acquire_timeout_s = value;
        else if (id == "visual_servo.lost_timeout_s") candidate.visual_servo.lost_timeout_s = value;
        else if (id == "visual_servo.overall_timeout_s") candidate.visual_servo.overall_timeout_s = value;
        else if (id == "visual_servo.max_body_speed_mps") candidate.visual_servo.max_body_speed_mps = value;
        else
        {
            if (error_message) *error_message = "未知参数：" + id;
            return false;
        }
    }

    const QString validation_error = validateWarehouseConfig(candidate);
    if (!validation_error.isEmpty())
    {
        if (error_message) *error_message = validation_error;
        return false;
    }

    working_config_ = candidate;
    return true;
}

void ParameterConfigDialog::commitParameterEdit()
{
    QString error_message;
    if (!updateParameterFromEditor(&error_message))
    {
        QMessageBox::warning(this, "参数错误", error_message);
        return;
    }

    editing_parameter_id_.clear();
    parameter_editor_->hide();
    rebuildParameterList();
}
void ParameterConfigDialog::handleApply()
{
    //检查参数合法性并保存
    const QString validation_error =
        validateWarehouseConfig(working_config_);

    if (!validation_error.isEmpty())
    {
        QMessageBox::warning(
            this, "参数错误", validation_error);
        return;
    }

    QString error_message;
    if (!saveWarehouseConfig(
            working_config_, &error_message))
    {
        QMessageBox::warning(
            this, "保存失败", error_message);
        return;
    }

    original_config_ = working_config_;
    accept();
}

