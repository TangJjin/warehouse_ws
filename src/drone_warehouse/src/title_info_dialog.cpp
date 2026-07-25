#include "drone_warehouse/title_info_dialog.hpp"

#include <QButtonGroup>
#include <QComboBox>
#include <QFormLayout>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QList>
#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

TitleInfoDialog::TitleInfoDialog(
    const WarehouseConfig &config,
    QWidget *parent)
    : QDialog(parent),
      working_config_(config),
      pending_type_(InspectionType::Cargo)
{
    setObjectName("TitleInfoDialog");
    setWindowTitle("详细配置");
    setModal(true);//设置为模态对话框，阻塞父窗口
    setWindowFlag(Qt::WindowContextHelpButtonHint, false);

    buildUi();
    loadConfigIntoControls();
    setPendingType(pending_type_);
}

void TitleInfoDialog::buildUi()
{
    auto *root_layout = new QHBoxLayout(this);

    auto *left_panel = new QWidget(this);
    auto *left_layout = new QVBoxLayout(left_panel);

    inspection_info_label_ = new QLabel("当前巡检：货物巡检", left_panel);
    cargo_button_ = new QPushButton("货物巡检", left_panel);
    animal_button_ = new QPushButton("动物巡检", left_panel);
    save_button_ = new QPushButton("保存", left_panel);
    close_button_ = new QPushButton("关闭", left_panel);

    //设置按钮为可选中状态
    cargo_button_->setCheckable(true);
    animal_button_->setCheckable(true);

    //把两个按钮加成一组，可以保证同一时间只有一个按钮被选中
    type_button_group_ = new QButtonGroup(this);
    type_button_group_->setExclusive(true);//设置为互斥模式
    type_button_group_->addButton(cargo_button_);
    type_button_group_->addButton(animal_button_);

    left_layout->addWidget(inspection_info_label_);
    left_layout->addWidget(cargo_button_);
    left_layout->addWidget(animal_button_);
    left_layout->addStretch();
    left_layout->addWidget(save_button_);
    left_layout->addWidget(close_button_);

    //堆叠窗口的设置，可以避免在切换巡检类型时，重新创建和销毁配置页面，从而提高性能和用户体验。
    config_stack_ = new QStackedWidget(this);
    cargo_page_ = new QWidget(config_stack_);
    animal_page_ = new QWidget(config_stack_);

    config_stack_->addWidget(cargo_page_);
    config_stack_->addWidget(animal_page_);

    root_layout->addWidget(left_panel);
    root_layout->addWidget(config_stack_, 1);

    connect(cargo_button_, &QPushButton::clicked, this, [this]() {
        setPendingType(InspectionType::Cargo);
    });
    connect(animal_button_, &QPushButton::clicked, this, [this]() {
        setPendingType(InspectionType::Animal);
    });

    connect(save_button_, &QPushButton::clicked,
            this, &TitleInfoDialog::handleSave);
    connect(close_button_, &QPushButton::clicked,
            this, &TitleInfoDialog::reject);
}

void TitleInfoDialog::setPendingType(InspectionType type)
{
    pending_type_ = type;

    //保证同一时间只能按一个按钮
    const bool cargo = type == InspectionType::Cargo;
    cargo_button_->setChecked(cargo);
    animal_button_->setChecked(!cargo);
    //切换堆叠窗口的当前页面
    config_stack_->setCurrentWidget(cargo ? cargo_page_ : animal_page_);

    //根据按钮状态更新巡检类型标签的内容
    inspection_info_label_->setText(
            cargo ? "当前选择：货物巡检"
                : "当前选择：动物巡检");
}

const WarehouseConfig &TitleInfoDialog::savedConfig() const
{
    return working_config_;
}

void TitleInfoDialog::loadConfigIntoControls()
{

}

void TitleInfoDialog::handleSave()
{
    working_config_ = working_config_;
    accept();
}
