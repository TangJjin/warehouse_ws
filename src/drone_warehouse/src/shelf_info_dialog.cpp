#include "drone_warehouse/shelf_info_dialog.hpp"

#include <QDebug>
#include <limits>
#include <QColor>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QIcon>
#include <QEvent>
#include <QLabel>
#include <QPainter>
#include <QPixmap>
#include <QPushButton>
#include <QSize>
#include <QVBoxLayout>
#include <QWidget>
#include <QVariant>

namespace
{
    QIcon makeStatusIcon(const QColor &color)
    {
        QPixmap pixmap(12, 12);
        pixmap.fill(Qt::transparent);

        QPainter painter(&pixmap);
        painter.setRenderHint(QPainter::Antialiasing, true);
        painter.setPen(Qt::NoPen);
        painter.setBrush(color);
        painter.drawEllipse(1, 1, 10, 10);

        return QIcon(pixmap);
    }

    QColor slotStatusColor(const ShelfSlotItem &slot)
    {
        const bool has_manual_data =
            !slot.category_id.isEmpty() && !slot.package_id.isEmpty();
        const bool has_observed_data =
            !slot.observed_category_id.isEmpty() && !slot.observed_package_id.isEmpty();

        // 1. 台账没有、巡检也没有：灰
        if (!has_manual_data && !has_observed_data)
        {
            return QColor("#7f8c9a");
        }

        // 2. 巡检有、台账没有：红
        if (!has_manual_data && has_observed_data)
        {
            return QColor("#ffae00");
        }

        // 3. 台账有、巡检没有：黄
        if (has_manual_data && !has_observed_data)
        {
            return QColor("#eed292");
        }

        // 4. 两边都有，完全一致：绿
        if (slot.category_id == slot.observed_category_id &&
            slot.package_id == slot.observed_package_id)
        {
            return QColor("#00d48a");
        }

        // 5. 两边都有，但不一致：红
        return QColor("#ff5c5c");
    }

    QColor shelfStatusColor(const ShelfPanelData &shelf)
    {
        bool has_red = false;
        bool has_yellow = false;
        bool all_gray = true;

        auto check_slots = [&](const QVector<ShelfSlotItem> &slot_items)
        {
            for (const ShelfSlotItem &slot : slot_items)
            {
                const QColor color = slotStatusColor(slot);

                if (color == QColor("#ff5c5c"))
                {
                    has_red = true;
                }
                else if (color == QColor("#f0b429"))
                {
                    has_yellow = true;
                }

                if (color != QColor("#7f8c9a"))
                {
                    all_gray = false;
                }
            }
        };

        check_slots(shelf.front_slots);
        check_slots(shelf.back_slots);

        if (all_gray)
        {
            return QColor("#7f8c9a");
        }

        if (has_red)
        {
            return QColor("#ff5c5c");
        }

        if (has_yellow)
        {
            return QColor("#f0b429");
        }

        return QColor("#00d48a");
    }

    uint16_t crc16_ccitt(const uint8_t *data, int length)
    {
        uint16_t crc = 0x0000;

        for (int i = 0; i < length; ++i) {
            crc ^= static_cast<uint16_t>(data[i]);
            for (int j = 0; j < 8; ++j) {
                if (crc & 0x0001) {
                    crc = (crc >> 1) ^ 0xA001;
                } else {
                    crc >>= 1;
                }
            }
        }

        return crc;
    }
}

ShelfInfoDialog::ShelfInfoDialog(QWidget *parent)
    : QDialog(parent)
{
    setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);//去掉系统自带的白色标题栏，只保留自定义弹窗内容
    resize(380, 470);//先给一个接近正方形的初始尺寸，便于后续继续扩展布局
    setMinimumSize(380, 470);//限制最小尺寸，避免窗口过小导致控件挤压
    setModal(false);//这里先用非模态窗口，点击后可与主界面同时操作

    setupSerial();//打开串口

    auto *main_layout = new QVBoxLayout(this);
    main_layout->setContentsMargins(5, 5, 5, 5);//窗口内部整体留白
    main_layout->setSpacing(12);//各模块之间的垂直间距

    title_label_ = new QLabel("货架1信息面板", this);
    title_label_->setObjectName("shelfDialogTitle");
    main_layout->addWidget(title_label_);

    /*********************顶部货架切换控件***********************/

    auto *switch_panel = new QWidget(this);
    switch_panel->setObjectName("shelfSwitchPanel");
    auto *switch_layout = new QHBoxLayout(switch_panel);
    switch_layout->setContentsMargins(5, 5, 5, 5);
    switch_layout->setSpacing(10);

    // 顶部仍然保留货架1/货架2切换，只是这两个按钮现在是切换当前货架，不再承载点位详情。
    // 这里先给一份默认文字和默认状态灯，真正显示什么会在 setShelfPanelData() 里由 MainWindow 传进来覆盖。
    shelf1_button_ = new QPushButton("货架1", switch_panel);
    shelf1_button_->setIcon(makeStatusIcon(QColor("#00d48a")));//默认状态灯，只作为数据未传入前的占位显示
    shelf1_button_->setIconSize(QSize(14, 14));

    shelf2_button_ = new QPushButton("货架2", switch_panel);
    shelf2_button_->setIcon(makeStatusIcon(QColor("#00d48a")));//默认状态灯，只作为数据未传入前的占位显示
    shelf2_button_->setIconSize(QSize(14, 14));

    switch_layout->addWidget(shelf1_button_);
    switch_layout->addWidget(shelf2_button_);
    switch_layout->addStretch();
    main_layout->addWidget(switch_panel);

    /***********************************************************/

    /*********************中间紧凑网格面板***********************/

    auto *content_panel = new QWidget(this);
    content_panel->setObjectName("slotContentPanel");
    auto *content_layout = new QHBoxLayout(content_panel);
    content_layout->setContentsMargins(5, 5, 5, 5);
    content_layout->setSpacing(10);

    // 左边竖向放“前面/后面”两个切换按钮，表示当前看的是货架哪一面。
    auto *side_button_panel = new QWidget(content_panel);
    auto *side_button_layout = new QVBoxLayout(side_button_panel);
    side_button_layout->setContentsMargins(0, 0, 0, 0);
    side_button_layout->setSpacing(8);

    front_button_ = new QPushButton("front", side_button_panel);
    back_button_ = new QPushButton("bark", side_button_panel);
    side_button_layout->addWidget(front_button_);
    side_button_layout->addWidget(back_button_);
    side_button_layout->addStretch();

    // 右边用4x4网格展示货位点位，每个按钮表示一个点位。
    auto *grid_panel = new QWidget(content_panel);
    slot_grid_layout_ = new QGridLayout(grid_panel);
    slot_grid_layout_->setContentsMargins(0, 0, 0, 0);
    slot_grid_layout_->setHorizontalSpacing(8);
    slot_grid_layout_->setVerticalSpacing(8);

    buildSlotGrid();//负责一次性创建16个货位按钮

    content_layout->addWidget(side_button_panel);
    content_layout->addWidget(grid_panel, 1);
    main_layout->addWidget(content_panel);

    /***********************************************************/

    /**********************简洁详情区***************************/

    // 这里不再显示一大堆货架整体信息，只保留当前点位、类别编号和包裹编号三行。
    auto *detail_panel = new QWidget(this);
    detail_panel->setObjectName("slotDetailPanel");
    auto *detail_layout = new QVBoxLayout(detail_panel);
    detail_layout->setContentsMargins(5, 5, 5, 5);
    detail_layout->setSpacing(6);

    slot_value_label_ = new QLabel("当前点位：前面 R1C1", detail_panel);
    package_value_label_ = new QLabel("包裹编号：————", detail_panel);
    category_value_label_ = new QLabel("类别编号：————", detail_panel);

    detail_layout->addWidget(slot_value_label_);
    detail_layout->addWidget(package_value_label_);
    detail_layout->addWidget(category_value_label_);
    main_layout->addWidget(detail_panel);

    /***********************************************************/

    /*********************底部按钮控件*************************/

    main_layout->addStretch();//把底部按钮区域推到窗口下方
    auto *button_layout = new QHBoxLayout();
    button_layout->setSpacing(10);//按钮之间的水平间距

    stock_in_button_ = new QPushButton("入库", this);
    outgoing_button_ = new QPushButton("出库", this);
    close_button_ = new QPushButton("关闭", this);

    button_layout->addWidget(stock_in_button_);
    button_layout->addWidget(outgoing_button_);
    button_layout->addStretch();
    button_layout->addWidget(close_button_);

    main_layout->addLayout(button_layout);

    /***********************************************************/

    // 顶部货架切换按钮负责切换当前货架。
    connect(shelf1_button_, &QPushButton::clicked, this, &ShelfInfoDialog::showShelf1Info);
    connect(shelf2_button_, &QPushButton::clicked, this, &ShelfInfoDialog::showShelf2Info);

    // 左侧前后按钮负责切换当前面。
    connect(front_button_, &QPushButton::clicked, this, &ShelfInfoDialog::switchToFront);
    connect(back_button_, &QPushButton::clicked, this, &ShelfInfoDialog::switchToBack);

    // 关闭按钮当前只做最基础行为，后续如果需要保存状态可再扩展。
    connect(close_button_, &QPushButton::clicked, this, &ShelfInfoDialog::close);
    connect(stock_in_button_, &QPushButton::clicked, this, &ShelfInfoDialog::startManualStockIn);
    connect(outgoing_button_, &QPushButton::clicked, this, &ShelfInfoDialog::handleManualStockOut);

    setStyleSheet(
        "QDialog {"
        "background: #101722;"
        "color: #d7e3f4;"
        "border: 1px solid rgba(90, 130, 180, 120);"
        "border-radius: 12px;"
        "}"
        "#shelfDialogTitle {"
        "font-size: 22px;"
        "font-weight: 600;"
        "color: #e7f3ff;"
        "}"
        "#shelfSwitchPanel {"
        "background: rgba(20, 28, 40, 160);"
        "border: 1px solid rgba(90, 130, 180, 120);"
        "border-radius: 10px;"
        "}"
        "#slotContentPanel {"
        "background: rgba(20, 28, 40, 180);"
        "border: 1px solid rgba(90, 130, 180, 120);"
        "border-radius: 10px;"
        "}"

        "#slotContentPanel QPushButton {"
        "background: transparent;"//按钮背景透明
        "border: none;"//按钮无边框
        "padding: 6px 10px;"//按钮内边距
        "}"

        "#slotContentPanel QPushButton:hover {"//按钮悬停效果
        "background: rgba(70, 110, 160, 80);"//悬停时背景变亮
        "border-radius: 6px;"//悬停时圆角稍微变大
        "}"

        "#slotDetailPanel {"
        "background: rgba(20, 28, 40, 130);"
        "border: 1px dashed rgba(90, 130, 180, 120);"
        "border-radius: 10px;"
        "}"
        "QLabel {"
        "border: none;"
        "color: #d7e3f4;"
        "font-size: 15px;"
        "}"
        "QPushButton {"
        "background: rgba(42, 58, 82, 180);"
        "border: 1px solid rgba(90, 130, 180, 140);"
        "border-radius: 8px;"
        "color: #d7e3f4;"
        "padding: 8px 12px;"
        "min-width: 72px;"
        "}"
        "QPushButton:hover {"
        "background: rgba(70, 110, 160, 120);"
        "}"
    );

    showShelf1Info();//默认先显示货架1
}

void ShelfInfoDialog::setShelfPanelData(const QVector<ShelfPanelData> &shelf_panel_data)
{
    // 这里接收主窗口一次性传进来的所有货架数据。
    // 每一个 ShelfPanelData 里，已经同时带了：
    // 1. 顶部显示名称
    // 2. 前面16个点位
    // 3. 后面16个点位
    // 弹窗这里不再自己拼假数据，只负责接收和显示。
    if (shelf_panel_data.isEmpty())
    {
        return;
    }

    // 每个货架都必须严格带有前后各16个点位。
    // 如果长度不对，后面按 4x4 网格刷新时就会越界，所以这里直接拦住。
    for (const ShelfPanelData &shelf : shelf_panel_data)
    {
        if (shelf.front_slots.size() != 12 || shelf.back_slots.size() != 12)
        {
            return;
        }
    }

    shelf_panel_data_ = shelf_panel_data;
    current_shelf_index_ = 0;//外部重新传数据后，默认回到第一个货架

    // if (shelf_panel_data_.size() > 0)
    // {
    //     shelf1_button_->setText(shelf_panel_data_[0].display_name);
    //     shelf1_button_->setIcon(makeStatusIcon(QColor(shelf_panel_data_[0].button_status_color)));
    // }
    // if (shelf_panel_data_.size() > 1)
    // {
    //     shelf2_button_->setText(shelf_panel_data_[1].display_name);
    //     shelf2_button_->setIcon(makeStatusIcon(QColor(shelf_panel_data_[1].button_status_color)));
    // }
    if (shelf_panel_data_.size() > 0)
    {
        shelf1_button_->setText(shelf_panel_data_[0].display_name);
        shelf1_button_->setIcon(makeStatusIcon(shelfStatusColor(shelf_panel_data_[0])));
    }

    if (shelf_panel_data_.size() > 1)
    {
        shelf2_button_->setText(shelf_panel_data_[1].display_name);
        shelf2_button_->setIcon(makeStatusIcon(shelfStatusColor(shelf_panel_data_[1])));
    }

    showShelfInfo(0);//数据设置完成后，默认显示第一个货架
}

void ShelfInfoDialog::showShelf1Info()
{
    showShelfInfo(0);//切换到第一个货架显示
}

void ShelfInfoDialog::showShelf2Info()
{
    showShelfInfo(1);//切换到第二个货架显示
}

void ShelfInfoDialog::showShelfInfo(int index)
{
    // index 表示当前要切到第几个货架。
    // 这里先做范围检查，防止顶部按钮和真实数据数量不一致时越界。
    if (index < 0 || index >= shelf_panel_data_.size())
    {
        return;
    }

    current_shelf_index_ = index;//记录当前正在显示哪一个货架

    // 标题只显示当前货架名称，具体点位内容交给下面的网格和详情区刷新。
    title_label_->setText(shelf_panel_data_[index].display_name + "信息面板");

    updateSlotGrid();//先刷新当前货架当前面的16个点位状态灯
    handleSlotClicked(current_slot_row_, current_slot_col_);//再刷新当前选中点位的详情文字
}

void ShelfInfoDialog::switchToFront()
{
    current_side_ = "front";//切到前面
    updateSlotGrid();//切换后刷新所有点位状态灯
    handleSlotClicked(current_slot_row_, current_slot_col_);//刷新当前点位详情
}

void ShelfInfoDialog::switchToBack()
{
    current_side_ = "back";//切到后面
    updateSlotGrid();//切换后刷新所有点位状态灯
    handleSlotClicked(current_slot_row_, current_slot_col_);//刷新当前点位详情
}

void ShelfInfoDialog::buildSlotGrid()
{
    // 这里一次性创建4x4共16个点位按钮。
    // 后面如果要改成更多行列，可以优先从这里调整循环范围。
    for (int row = 0; row < 4; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            auto *button = new QPushButton(QString("R%1C%2").arg(row + 1).arg(col + 1), this);
            button->setProperty("slotRow", QVariant(row));//创建按钮时写入属性，表示是几行几列
            button->setProperty("slotCol", QVariant(col));
            button->installEventFilter(this);
            button->setIcon(makeStatusIcon(QColor("#7f8c9a")));//默认先按空位灰色状态灯显示
            button->setIconSize(QSize(13, 13));
            button->setMinimumSize(78, 40);//让每个格子的尺寸比较统一，方便组成紧凑矩阵

            connect(button, &QPushButton::clicked, this, [this, row, col]() {
                handleSlotClicked(row, col);//点击后记录当前点位并刷新下方详情区
            });

            slot_buttons_.push_back(button);
            slot_grid_layout_->addWidget(button, row, col);
        }
    }
}

void ShelfInfoDialog::updateSlotGrid()
{
    // 如果外部还没有把货架数据传进来，这里就先不刷新，避免直接访问空数组。
    if (shelf_panel_data_.isEmpty() || current_shelf_index_ < 0 || current_shelf_index_ >= shelf_panel_data_.size())
    {
        return;
    }
    

    const ShelfPanelData &current_shelf = shelf_panel_data_[current_shelf_index_];//先取当前货架
    const QVector<ShelfSlotItem> &current_slots =
        (current_side_ == "front") ? current_shelf.front_slots : current_shelf.back_slots;//再按前后面取当前这一面的16个点位

    // 按当前正在查看的是“前面”还是“后面”，统一刷新16个点位按钮的状态灯和文字。
    for (int row = 0; row < 4; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            const int index = slotIndex(row, col);
            QPushButton *button = slot_buttons_[index];

            const QString button_text = QString("R%1C%2").arg(row + 1).arg(col + 1);
            button->setText(button_text);//文字始终显示点位编号，不跟状态混在一起

            const ShelfSlotItem &slot = current_slots[index];//取出当前格子对应的数据

            // 只要类别编号非空，就认为该点位当前有货；否则视为一个空位。
            // if (slot.category_id.isEmpty())
            // {
            //     button->setIcon(makeStatusIcon(QColor("#7f8c9a")));//空位用灰色状态灯
            // }
            // else
            // {
            //     button->setIcon(makeStatusIcon(QColor("#00d48a")));//有货用绿色状态灯
            // }
            button->setIcon(makeStatusIcon(slotStatusColor(slot)));

            const bool is_selected = (row == current_slot_row_ && col == current_slot_col_);
            if (is_selected) {
                button->setStyleSheet("background: rgba(70, 110, 160, 160); border: 1px solid rgba(120, 180, 255, 180); border-radius:8px; color: #e7f3ff;");
            } else {
                button->setStyleSheet("background: rgba(42, 58, 82, 120); border: 1px solid rgba(90, 130, 180, 80); border-radius:8px; color: #d7e3f4;");
            }

            button->setToolTip(
            QString("台账: %1 | %2\n巡检: %3 | %4")
                .arg(slot.category_id.isEmpty() ? "——" : slot.category_id)
                .arg(slot.package_id.isEmpty() ? "——" : slot.package_id)
                .arg(slot.observed_category_id.isEmpty() ? "——" : slot.observed_category_id)
                .arg(slot.observed_package_id.isEmpty() ? "——" : slot.observed_package_id));
        }
    }

    // 顺手把前后按钮的选中态也一起刷新，方便一眼看出当前看的哪一面。
    if (current_side_ == "front")
    {
        front_button_->setStyleSheet("background: rgba(70, 110, 160, 160); border: 1px solid rgba(120, 180, 255, 180); border-radius: 8px; color: #e7f3ff; padding: 8px 12px;");
        back_button_->setStyleSheet("background: rgba(42, 58, 82, 180); border: 1px solid rgba(90, 130, 180, 140); border-radius: 8px; color: #d7e3f4; padding: 8px 12px;");
    }
    else
    {
        back_button_->setStyleSheet("background: rgba(70, 110, 160, 160); border: 1px solid rgba(120, 180, 255, 180); border-radius: 8px; color: #e7f3ff; padding: 8px 12px;");
        front_button_->setStyleSheet("background: rgba(42, 58, 82, 180); border: 1px solid rgba(90, 130, 180, 140); border-radius: 8px; color: #d7e3f4; padding: 8px 12px;");
    }
}

void ShelfInfoDialog::handleSlotClicked(int row, int col)
{
    current_slot_row_ = row;//记录当前选中的行
    current_slot_col_ = col;//记录当前选中的列

    updateSlotGrid();

    // 如果当前还没有任何货架数据，就不要继续往下取点位了。
    if (shelf_panel_data_.isEmpty() || current_shelf_index_ < 0 || current_shelf_index_ >= shelf_panel_data_.size())
    {
        updateSlotDetail("未加载货架数据", "————", "————");
        return;
    }

    const int index = slotIndex(row, col);
    const QString slot_name = QString("%1 R%2C%3")
                                  .arg(current_side_ == "front" ? "front" : "bark")
                                  .arg(row + 1)
                                  .arg(col + 1);

    const ShelfPanelData &current_shelf = shelf_panel_data_[current_shelf_index_];//先取当前货架
    const QVector<ShelfSlotItem> &current_slots =
        (current_side_ == "front") ? current_shelf.front_slots : current_shelf.back_slots;//再取当前这一面的16个点位
    const ShelfSlotItem &slot = current_slots[index];//最后取当前点击的那个点位

    // 空位只显示横线占位，表示当前点位还没有放货。
    if (slot.category_id.isEmpty())
    {
        updateSlotDetail(slot_name, "————", "————");
    }
    else
    {
        updateSlotDetail(slot_name, slot.category_id, slot.package_id);
    }
}

void ShelfInfoDialog::updateSlotDetail(const QString &slot_name, const QString &category_id, const QString &package_id)
{
    slot_value_label_->setText("当前点位：" + slot_name);
    package_value_label_->setText("包裹编号：" + package_id);
    category_value_label_->setText("类别编号：" + category_id);
}

bool ShelfInfoDialog::eventFilter(QObject *watched, QEvent *event)
{
    //监听是否是4x4网格触发的双击
    auto *button = qobject_cast<QPushButton *>(watched);
    if (button && slot_buttons_.contains(button) && event->type() == QEvent::MouseButtonDblClick)
    {
        //从按钮属性中读出行列
        const int row = button->property("slotRow").toInt();
        const int col = button->property("slotCol").toInt();
        emit slotDoubleClicked(current_shelf_index_, current_side_, row, col);//发信号给mainwindow
        return true;
    }

    return QDialog::eventFilter(watched, event);
}

void ShelfInfoDialog::setupSerial()
{
    serial_.setPortName("/dev/ttyS6");
    serial_.setBaudRate(QSerialPort::Baud9600);
    serial_.setDataBits(QSerialPort::Data8);
    serial_.setParity(QSerialPort::NoParity);
    serial_.setStopBits(QSerialPort::OneStop);
    serial_.setFlowControl(QSerialPort::NoFlowControl);

    if (!serial_.open(QIODevice::ReadWrite)) {
        return;
    }

    connect(&serial_, &QSerialPort::readyRead, this, [this]() {
        uint8_t deviceId = 0;
        uint8_t status = 0;
        QByteArray payload;

        // 地面站手动入库的串口接收链路分两段：
        // 1. 先走 `uart_read(...)`，把带 `31 01` 包头的 ACK 帧从串口流里切出来，再交给 `handleSerialFrame(...)` 判断。
        // 2. ACK 之后，扫码枪会继续直接吐出纯文本 `CAT01|PKG88`，这部分不是协议帧，所以继续从串口剩余字节里攒文本。
        // 3. 一旦文本里已经形成完整的 `类别|包裹` 格式，就交给 `processManualScanText(...)` 往主窗口回传。
        if (waiting_manual_scan_result_ && manual_scan_ack_received_) {
            raw_serial_buffer_.append(serial_.readAll());
            const qsizetype newline_index = raw_serial_buffer_.indexOf('\n');
            if (newline_index >= 0) {
                const QByteArray line = raw_serial_buffer_.left(newline_index);
                raw_serial_buffer_.remove(0, newline_index + 1);
                processManualScanText(QString::fromLatin1(line).trimmed());
            } else if (!raw_serial_buffer_.isEmpty()) {
                const QString scan_text = QString::fromLatin1(raw_serial_buffer_).trimmed();
                if (raw_serial_buffer_.contains('\r')) {
                    QByteArray line = raw_serial_buffer_.left(raw_serial_buffer_.indexOf('\r'));
                    raw_serial_buffer_.clear();
                    processManualScanText(scan_text);
                }
            }
        }

        while (uart_read(deviceId, status, payload)) {
            handleSerialFrame(deviceId, status, payload);
        }
    });
}

void ShelfInfoDialog::startManualStockIn()
{
    if (!serial_.isOpen()) {
        updateSlotDetail("串口未打开", category_value_label_->text().mid(5), package_value_label_->text().mid(5));
        return;
    }

    // 地面站手动入库发起链路：
    // 1. 先把“当前选中的货架/前后面/行列”冻结到 pending_*，避免扫码期间用户切换选中格后写错位置。
    // 2. 清空本轮串口文本缓存，并进入 `waiting_manual_scan_result_` 等待态。
    // 3. 最后发固定开启扫码命令，等待设备先回 ACK、再回纯文本扫码结果。
    pending_shelf_index_ = current_shelf_index_;
    pending_side_ = current_side_;
    pending_row_ = current_slot_row_;
    pending_col_ = current_slot_col_;
    waiting_manual_scan_result_ = true;
    manual_scan_ack_received_ = false;
    raw_serial_buffer_.clear();

    stock_outgoing_ = 0;
    uart_write(0x00, 0x00, QByteArray::fromHex("0A020001"));
}

void ShelfInfoDialog::handleManualStockOut()
{
    if (!serial_.isOpen()) {
        updateSlotDetail("串口未打开", category_value_label_->text().mid(5), package_value_label_->text().mid(5));
        return;
    }
    // 地面站手动出库发起链路：
    // 1. Dialog 不直接改自己的展示副本。
    // 2. 这里只把“当前选中的格子坐标”发给 MainWindow。
    // 3. 真正的数据清空统一由 MainWindow 执行，避免主数据和弹窗副本分叉。

    pending_shelf_index_ = current_shelf_index_;
    pending_side_ = current_side_;
    pending_row_ = current_slot_row_;
    pending_col_ = current_slot_col_;
    waiting_manual_scan_result_ = true;
    manual_scan_ack_received_ = false;
    raw_serial_buffer_.clear();

    stock_outgoing_ = 1;
    uart_write(0x00, 0x00, QByteArray::fromHex("0A020001"));
}

void ShelfInfoDialog::handleSerialFrame(uint8_t deviceId, uint8_t status, const QByteArray &payload)
{
    Q_UNUSED(deviceId);
    Q_UNUSED(payload);

    // 这里专门处理“开启扫码命令”对应的 ACK 帧。
    // 当前协议里，设备会先返回一帧 `31 01 ...` 的确认帧，status 仍然是 0x00。
    // 这一步不落库，只表示设备已经收到命令，真正的扫码结果还要继续等后面的纯文本串口输出。
    if (status == 0x00 && waiting_manual_scan_result_) {
        manual_scan_ack_received_ = true;
        raw_serial_buffer_.clear();
        return;
    }
}

void ShelfInfoDialog::processManualScanText(const QString &scan_text)
{
    // 这里处理地面站手动入库真正拿到的扫码文本。
    // 当前约定格式只有两段：`CAT01|PKG88`。
    // 解析成功后，把结果和之前冻结下来的 pending 格子一起发给 MainWindow，
    // 由 MainWindow 统一写入主数据，再刷新弹窗显示。
    const QStringList parts = scan_text.split('|', Qt::KeepEmptyParts);
    if (parts.size() < 2) {
        return;
    }
    const QString package_id = parts[0].trimmed();
    const QString category_id = parts[1].trimmed();

    manual_scan_ack_received_ = false;
    waiting_manual_scan_result_ = false;
    if(stock_outgoing_ == 0)
    {
        emit manualStockInScanned(pending_shelf_index_, pending_side_, pending_row_, pending_col_,
                                category_id, package_id);
        stock_outgoing_ = -1;
    }
    else if(stock_outgoing_ == 1)
    {
        emit manualStockOutRequested(pending_shelf_index_, pending_side_, pending_row_, pending_col_,
                                    category_id, package_id);
        stock_outgoing_ = -1;
    }
}

int ShelfInfoDialog::slotIndex(int row, int col) const
{
    return row * 3 + col;//4列表格里，把二维行列下标换算成一维数组下标
}

void ShelfInfoDialog::uart_write(uint8_t deviceId, uint8_t status, const QByteArray &payload)
{

    if (!serial_.isOpen()) {
        return;
    }

    const QByteArray frame = encodeFrame(0x57, 0x01, deviceId, status, payload);
    serial_.write(frame);
    qDebug() << "TX:" << frame.toHex(' ').toUpper();
    serial_.flush();

    // QByteArray payload;
    // uint8_t status = (0x1 << 4) | 0x0;
    // uart_write(0x00, status, payload);
}

bool ShelfInfoDialog::uart_read(uint8_t &deviceId, uint8_t &status, QByteArray &payload)
{
    static QByteArray rxBuffer;

    deviceId = 0;
    status = 0;
    payload.clear();

    rxBuffer.append(serial_.readAll());

    constexpr uint8_t kRespSof1 = 0x31;
    constexpr uint8_t kRespSof2 = 0x01;
    constexpr int kMinFrameSize = 10;   // 2 + 1 + 1 + 4 + 2
    constexpr int kFixedHeaderSize = 8; // 2 + 1 + 1 + 4

    while (true) {
        if (rxBuffer.size() < 2) {
            return false;
        }

        int sofIndex = -1;
        for (int i = 0; i <= rxBuffer.size() - 2; ++i) {
            const uint8_t b0 = static_cast<uint8_t>(rxBuffer.at(i));
            const uint8_t b1 = static_cast<uint8_t>(rxBuffer.at(i + 1));
            if (b0 == kRespSof1 && b1 == kRespSof2) {
                sofIndex = i;
                break;
            }
        }

        if (sofIndex < 0) {
            if (rxBuffer.size() > 1) {
                rxBuffer.remove(0, rxBuffer.size() - 1);
            }
            return false;
        }

        if (sofIndex > 0) {
            rxBuffer.remove(0, sofIndex);
        }

        if (rxBuffer.size() < kFixedHeaderSize) {
            return false;
        }

        const auto *data = reinterpret_cast<const uint8_t *>(rxBuffer.constData());

        // 数据长度：4 byte，大端
        const uint32_t payloadLen =
            (static_cast<uint32_t>(data[4]) << 24) |
            (static_cast<uint32_t>(data[5]) << 16) |
            (static_cast<uint32_t>(data[6]) << 8)  |
            static_cast<uint32_t>(data[7]);

        const quint64 expectedSize64 =
            static_cast<quint64>(kFixedHeaderSize) +
            static_cast<quint64>(payloadLen) +
            2u;

        if (expectedSize64 > static_cast<quint64>(std::numeric_limits<int>::max())) {
            rxBuffer.remove(0, 1);
            continue;
        }

        const int expectedSize = static_cast<int>(expectedSize64);

        if (expectedSize < kMinFrameSize) {
            rxBuffer.remove(0, 1);
            continue;
        }

        if (rxBuffer.size() < expectedSize) {
            return false;
        }

        const QByteArray frame = rxBuffer.left(expectedSize);

        uint8_t parsedDeviceId = 0;
        uint8_t parsedStatus = 0;
        QByteArray parsedPayload;

        if (validateFrame(frame, kRespSof1, kRespSof2,
                        parsedDeviceId, parsedStatus, parsedPayload)) {
            rxBuffer.remove(0, expectedSize);

            deviceId = parsedDeviceId;
            status = parsedStatus;
            payload = parsedPayload;

            qDebug() << "RX:" << frame.toHex(' ').toUpper();
            return true;
        }

        rxBuffer.remove(0, 1);
    }
}

QByteArray ShelfInfoDialog::encodeFrame(uint8_t sof1, uint8_t sof2,
                                        uint8_t deviceId, uint8_t status,
                                        const QByteArray &payload) const
{
    QByteArray frame;
    frame.reserve(2 + 1 + 1 + 4 + payload.size() + 2);

    // 包头
    frame.append(static_cast<char>(sof1));
    frame.append(static_cast<char>(sof2));

    // 设备号
    frame.append(static_cast<char>(deviceId));

    // 指令状态
    frame.append(static_cast<char>(status));

    // 数据长度：4 byte，大端
    const uint32_t payloadLen = static_cast<uint32_t>(payload.size());
    frame.append(static_cast<char>((payloadLen >> 24) & 0xFF));
    frame.append(static_cast<char>((payloadLen >> 16) & 0xFF));
    frame.append(static_cast<char>((payloadLen >> 8) & 0xFF));
    frame.append(static_cast<char>(payloadLen & 0xFF));

    // 数据
    frame.append(payload);

    // CRC16：对除 CRC 自身以外的所有字节计算
    const uint8_t *data = reinterpret_cast<const uint8_t *>(frame.constData());
    const int crcLen = frame.size();
    const uint16_t crc = crc16_ccitt(data, crcLen);

    frame.append(static_cast<char>((crc >> 8) & 0xFF));
    frame.append(static_cast<char>(crc & 0xFF));

    return frame;
}

bool ShelfInfoDialog::validateFrame(const QByteArray &frame,
                                    uint8_t expectedSof1, uint8_t expectedSof2,
                                    uint8_t &deviceId, uint8_t &status,
                                    QByteArray &payload) const
{
    // 最小长度 = 包头2 + 设备号1 + 指令状态1 + 数据长度4 + CRC2
    if (frame.size() < 10) {
        return false;
    }

    const auto *data = reinterpret_cast<const uint8_t *>(frame.constData());

    // 包头校验
    if (data[0] != expectedSof1 || data[1] != expectedSof2) {
        return false;
    }

    // 数据长度：4 byte，大端
    const uint32_t payloadLen =
        (static_cast<uint32_t>(data[4]) << 24) |
        (static_cast<uint32_t>(data[5]) << 16) |
        (static_cast<uint32_t>(data[6]) << 8)  |
        static_cast<uint32_t>(data[7]);

    const int expectedSize = 2 + 1 + 1 + 4 + static_cast<int>(payloadLen) + 2;
    if (frame.size() != expectedSize) {
        return false;
    }

    // 提取 CRC
    const uint16_t recvCrc =
        (static_cast<uint16_t>(data[frame.size() - 2]) << 8) |
        static_cast<uint16_t>(data[frame.size() - 1]);

    // 计算 CRC（除 CRC 本身）
    const uint16_t calcCrc = crc16_ccitt(data, frame.size() - 2);

    if (recvCrc != calcCrc) {
        return false;
    }

    // 解析字段
    deviceId = data[2];
    status = data[3];
    payload = frame.mid(8, static_cast<int>(payloadLen));

    return true;
}
