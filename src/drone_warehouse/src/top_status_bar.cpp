#include "drone_warehouse/top_status_bar.hpp"
#include "drone_warehouse/color_palette.hpp"

#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>

#include <QDateTime>
#include <QTimer>

TopStatusBar::TopStatusBar(QWidget *parent)
    : QFrame(parent)
{
    setObjectName("topStatusBar");

    auto *layout = new QHBoxLayout(this);
    layout->setContentsMargins(12, 8, 12, 8);
    layout->setSpacing(12);

    title_button_ = new QPushButton("仓储智航", this);
    connection_button_ = new QPushButton("未连接", this);
    shelf_button_ = new QPushButton("货架信息", this);
    task_button_ = new QPushButton("任务待命", this);
    execute_button_ = new QPushButton("执行", this);
    scheduled_check_button_ = new QPushButton("定时巡检", this);
    time_label_ = new QLabel("00:00:00", this);

    clock_timer_ = new QTimer(this);//新建定时器
    time_label_->setText(QDateTime::currentDateTime().toString("HH:mm:ss"));//立即刷新一次

    layout->addWidget(connection_button_);
    layout->addWidget(title_button_);
    layout->addWidget(shelf_button_);
    layout->addWidget(task_button_);
    layout->addStretch();
    layout->addWidget(execute_button_);
    layout->addWidget(scheduled_check_button_);
    layout->addWidget(time_label_);

    title_button_->hide();
    shelf_button_->hide();
    task_button_->hide();
    execute_button_->hide();
    scheduled_check_button_->hide();

    connect(title_button_, &QPushButton::clicked, this, &TopStatusBar::titleClicked);
    connect(connection_button_, &QPushButton::clicked, this, &TopStatusBar::connectionClicked);
    connect(task_button_, &QPushButton::clicked, this, &TopStatusBar::taskClicked);
    connect(execute_button_, &QPushButton::clicked, this, &TopStatusBar::executeButtonClicked);
    connect(shelf_button_, &QPushButton::clicked, this, &TopStatusBar::shelfButtonClicked);

    connect(clock_timer_, &QTimer::timeout, this, [this]() {//每秒触发刷新一次时间文本
        const QString current_time_text = QDateTime::currentDateTime().toString("HH:mm:ss");
        time_label_->setText(current_time_text);

        /*********************ros移植部分***********************/
        // 当前先把“到点触发上传”的检测也挂在顶部状态栏已有的时钟定时器里。
        // 这样不需要额外再造一个新的 UI 定时器，主窗口只要接收信号并决定是否真正上传即可。
        if (time_trigger_enabled_ && !trigger_time_text_.isEmpty())
        {
            if (current_time_text == trigger_time_text_)
            {
                if (last_triggered_time_text_ != current_time_text)
                {
                    last_triggered_time_text_ = current_time_text;
                    emit triggerTimeReached(current_time_text);
                }
            }
            else
            {
                // 一旦离开目标时刻，就清掉“本秒已触发”的标记，方便后续再次到点时还能重新触发。
                last_triggered_time_text_.clear();
            }
        }
        /******************************************************/
    });

    clock_timer_->start(1000);

    setStyleSheet(
        "#topStatusBar {"
        "background: rgba(20, 28, 40, 180);"
        "border: 1px solid rgba(90, 130, 180, 120);"
        "border-radius: 10px;"
        "}"
        "#topStatusBar QPushButton {"
        "background: transparent;"//按钮背景透明
        "border: none;"//按钮无边框
        "color: #d7e3f4;"//按钮文字颜色
        "padding: 6px 10px;"//按钮内边距
        "}"
        "#topStatusBar QPushButton:hover {"//按钮悬停效果
        "background: rgba(70, 110, 160, 80);"//悬停时背景变亮
        "border-radius: 6px;"//悬停时圆角稍微变大
        "}"
        "#topStatusBar QLabel {"
        "color: #d7e3f4;"
        "}"
    );
}

void TopStatusBar::setConnected(bool connected)
{
    title_button_->setVisible(connected);
    shelf_button_->setVisible(connected);
    task_button_->setVisible(connected);
    execute_button_->setVisible(connected);
    scheduled_check_button_->setVisible(connected);
    connection_button_->setText(connected ? "已连接" : "未连接");
}

void TopStatusBar::setConnectionText(const QString &text)
{
    connection_button_->setText(text);
}

void TopStatusBar::setTaskText(const QString &text)
{
    task_button_->setText(text);
}

void TopStatusBar::setShelfText(const QString &text)
{
    shelf_button_->setText(text);
}

void TopStatusBar::setTimeText(const QString &text)
{
    time_label_->setText(text);
}

/*********************ros移植部分***********************/
void TopStatusBar::setTriggerTime(const QString &text)
{
    trigger_time_text_ = text;
    last_triggered_time_text_.clear();
}

void TopStatusBar::setTimeTriggerEnabled(bool enabled)
{
    time_trigger_enabled_ = enabled;
    if (!time_trigger_enabled_)
    {
        last_triggered_time_text_.clear();
    }
}
/******************************************************/

QPoint TopStatusBar::shelfButtonBottomLeftGlobal() const
{
    return shelf_button_->mapToGlobal(QPoint(0, shelf_button_->height()));
}

