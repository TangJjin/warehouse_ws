#pragma once

#include "drone_warehouse/warehouse_config.hpp"

#include <QDialog>

class QButtonGroup;
class QComboBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QWidget;
class QStackedWidget;

enum class InspectionType
{
    Cargo,//货物巡检
    Animal//动物巡检
};

class TitleInfoDialog : public QDialog
{
public:
    explicit TitleInfoDialog(
        const WarehouseConfig &config,
        QWidget *parent = nullptr);

    // 保存成功后，主窗口通过这个接口取得最新配置。
    const WarehouseConfig &savedConfig() const;

private:
    void buildUi();//构建UI界面
    void loadConfigIntoControls();//加载配置到控件
    void setPendingType(InspectionType type);//设置待处理的巡检类型
    void handleSave();//处理保存操作

    // 弹窗只修改副本，点击保存成功前不会影响主窗口当前配置。
    WarehouseConfig working_config_;
    InspectionType pending_type_ = InspectionType::Cargo;//默认巡检类型为货物巡检

    QLabel *inspection_info_label_ = nullptr;//巡检类型标签
    QPushButton *cargo_button_ = nullptr;//货物巡检按钮
    QPushButton *animal_button_ = nullptr;//动物巡检按钮
    QPushButton *save_button_ = nullptr;//保存按钮
    QPushButton *close_button_ = nullptr;//关闭按钮
    QButtonGroup *type_button_group_ = nullptr;//巡检类型按钮组

    QStackedWidget *config_stack_ = nullptr;//配置堆叠窗口
    QWidget *cargo_page_ = nullptr;//货物巡检配置页面
    QWidget *animal_page_ = nullptr;//动物巡检配置页面
};