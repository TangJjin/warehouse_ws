#pragma once

#include "drone_warehouse/warehouse_config.hpp"

#include <QDialog>

class QButtonGroup;
class QListWidget;
class QPushButton;
class QStackedWidget;
class QWidget;

enum class InspectionProject
{
    Cargo,
    Animal
};

class ParameterConfigDialog : public QDialog
{
public:
    explicit ParameterConfigDialog(
        const WarehouseConfig &config,
        QWidget *parent = nullptr);

    const WarehouseConfig &savedConfig() const;

private:
    void buildUi();
    void buildProjectPage();//构建项目选择页面
    void buildParameterPage();//构建参数编辑页面
    void selectMainPage(int page_index);//页面切换功能
    void selectProject(InspectionProject project);//选择巡检项目设置为高亮显示
    void rebuildParameterList();//根据当前项目重建参数列表

    void addParameterRow(//创建带数值和分割线的参数行
        const QString &parameter_id,
        const QString &display_name,
        const QString &display_value);

    void handleApply();//启用修改

    WarehouseConfig original_config_;// 打开页面时的配置，用于丢弃
    WarehouseConfig working_config_;// 页面内正在修改的配置

    InspectionProject selected_project_ =
        InspectionProject::Cargo;//当前选择的巡检项目，默认为仓储

    QPushButton *project_page_button_ = nullptr;//主界面项目选择按钮
    QPushButton *parameter_page_button_ = nullptr;//主界面参数编辑按钮
    QButtonGroup *main_page_button_group_ = nullptr;//主界面按钮组，用于切换项目选择和参数编辑页面

    QPushButton *restore_button_ = nullptr;//恢复默认参数按钮
    QPushButton *discard_button_ = nullptr;//丢弃修改按钮
    QPushButton *apply_button_ = nullptr;//保存修改按钮

    QStackedWidget *page_stack_ = nullptr;//主界面堆叠窗口，用于切换项目选择和参数编辑页面
    QWidget *project_page_ = nullptr;//主界面项目选择页面
    QWidget *parameter_page_ = nullptr;//主界面参数编辑页面

    QButtonGroup *project_button_group_ = nullptr;//项目选择按钮组，用于切换仓储和动物巡检项目
    QListWidget *parameter_list_ = nullptr;//参数列表，用于显示当前项目的参数项
    QWidget *parameter_editor_ = nullptr;//参数编辑器，用于编辑当前选中的参数项
};