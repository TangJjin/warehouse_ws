#include "drone_warehouse/title_info_dialog.hpp"

#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

TitleInfoDialog::TitleInfoDialog(QWidget *parent)
    : QDialog(parent)
{
    setObjectName("titleInfoDialog");
    setWindowFlag(Qt::WindowContextHelpButtonHint, false);
    setWindowFlag(Qt::Popup, true);

    buildUi();
}

void TitleInfoDialog::buildUi()
{
    auto *layout = new QVBoxLayout(this);
    layout->setContentsMargins(8, 8, 8, 8);
    layout->setSpacing(8);

    parameter_button_ = new QPushButton("参数", this);
    close_button_ = new QPushButton("关闭", this);

    parameter_button_->setMinimumSize(140, 38);
    close_button_->setMinimumSize(140, 38);

    layout->addWidget(parameter_button_);
    layout->addWidget(close_button_);

    // Accepted 表示用户选择了“参数”，由 MainWindow 打开全屏页面。
    connect(parameter_button_, &QPushButton::clicked,
            this, &TitleInfoDialog::accept);

    connect(close_button_, &QPushButton::clicked,
            this, &TitleInfoDialog::reject);
}