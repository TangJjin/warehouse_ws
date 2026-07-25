#include "drone_warehouse/parameter_config_dialog.hpp"

#include <QButtonGroup>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QStackedWidget>
#include <QVBoxLayout>
#include <QAbstractButton>
#include <QListWidgetItem>
#include <QMessageBox>

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
}

ParameterConfigDialog::ParameterConfigDialog(
    const WarehouseConfig &config,
    QWidget *parent)
    : QDialog(parent),
      original_config_(config),
      working_config_(config)
{
    setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);//去掉系统自带的白色标题栏，只保留自定义弹窗内容
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
    top_layout->addWidget(new QLabel("智航参数", this));
    top_layout->addStretch();

    restore_button_ = new QPushButton("恢复默认值", this);
    discard_button_ = new QPushButton("丢弃", this);
    apply_button_ = new QPushButton("启用", this);

    top_layout->addWidget(restore_button_);
    top_layout->addWidget(discard_button_);
    top_layout->addWidget(apply_button_);

    auto *body_layout = new QHBoxLayout;

    auto *left_panel = new QWidget(this);
    auto *left_layout = new QVBoxLayout(left_panel);

    project_page_button_ = new QPushButton("项目选择", left_panel);
    parameter_page_button_ = new QPushButton("参数设置", left_panel);

    //设置按钮为可选中
    project_page_button_->setCheckable(true);
    parameter_page_button_->setCheckable(true);

    main_page_button_group_ = new QButtonGroup(this);//主界面按钮组
    main_page_button_group_->setExclusive(true);//设置按钮为互斥模式
    main_page_button_group_->addButton(project_page_button_);
    main_page_button_group_->addButton(parameter_page_button_);

    left_layout->addWidget(project_page_button_);
    left_layout->addWidget(parameter_page_button_);
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

    connect(restore_button_, &QPushButton::clicked,
        this, [this]() {
            const WarehouseConfig defaults =
                createDefaultWarehouseConfig();

            working_config_.mission = defaults.mission;
            rebuildParameterList();
        });

    connect(discard_button_, &QPushButton::clicked,
            this, [this]() {
                working_config_ = original_config_;
                reject();
            });

    connect(apply_button_, &QPushButton::clicked,
            this, &ParameterConfigDialog::handleApply);
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

    // 下一阶段点击参数后显示这个编辑区域。
    parameter_editor_ = new QWidget(parameter_page_);//参数编辑器
    parameter_editor_->setFixedWidth(360);
    parameter_editor_->hide();//默认隐藏编辑区域

    layout->addWidget(parameter_list_, 1);
    layout->addWidget(parameter_editor_);

    page_stack_->addWidget(parameter_page_);

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
        "use_camera_aim",
        "启用相机对准",
        boolDisplayText(mission.use_camera_aim));

    addParameterRow(
        "auto_start_mission",
        "上传后自动启动",
        boolDisplayText(mission.auto_start_mission));

    addParameterRow(
        "frame",
        "任务坐标系",
        mission.frame);
}

void ParameterConfigDialog::addParameterRow(
    const QString &parameter_id,
    const QString &display_name,
    const QString &display_value)
{
    auto *item = new QListWidgetItem(parameter_list_);
    item->setData(Qt::UserRole, parameter_id);
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

