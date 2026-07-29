#pragma once

#include <QPointF>
#include <QRectF>
#include <QWidget>

class QPaintEvent;
class QPainter;

// 空地协同专用固定二维画板。
// 左下圆形区域的圆心是实时位置坐标原点：画面上方为 x+，左方为 y+。
// ROS 传入坐标单位为米；场地图形尺寸统一使用厘米；Qt 最终绘制使用像素。
class CollaborationGridView : public QWidget
{
public:
    explicit CollaborationGridView(QWidget *parent = nullptr);
    ~CollaborationGridView() override = default;

    // 更新无人机实时位置，x/y/z 单位都是米。
    void setdronePosition(double x, double y, double z);

    // 更新无人车实时位置，使用与无人机相同的坐标原点和方向。
    void setcarPosition(double x, double y, double z);

protected:
    // QWidget 需要重绘时由 Qt 自动调用，所有固定图形和实时圆点都在这里绘制。
    void paintEvent(QPaintEvent *event) override;

private:
    // 实际场地宽 400 cm、高 500 cm。
    // 修改场地总尺寸时，需要同时确认 cpp 顶部的跑道和圆形区域尺寸。
    static constexpr qreal kMapWidthCm = 400.0;
    static constexpr qreal kMapHeightCm = 500.0;

    // 计算保持 400:500 比例后的场地像素矩形。
    QRectF mapRect() const;

    // 把场地尺寸坐标转换成 Qt 像素坐标：
    // horizontal_cm 从场地左边向右量，vertical_cm 从场地底边向上量。
    QPointF mapPoint(
        qreal horizontal_cm,
        qreal vertical_cm) const;

    // 把以左边、底边、宽度、高度描述的厘米矩形转换为 Qt QRectF。
    QRectF mapObjectRect(
        qreal left_cm,
        qreal bottom_cm,
        qreal width_cm,
        qreal height_cm) const;

    // 当前缩放比例：现场 1 cm 对应多少窗口像素。
    qreal mapScale() const;

    // 绘制固定的直圆跑道。
    void drawInspectionTrack(QPainter &painter) const;

    // 绘制左下角圆形区域和原点十字。
    void drawCircularArea(QPainter &painter) const;

    // 根据实时坐标绘制黄色小车圆点。
    void drawCarMarker(QPainter &painter) const;

    // 无人机默认坐标 (0, 0) m，因此红点位于左下圆心。
    double display_x_ = 0.0;
    double display_y_ = 0.0;
    double altitude_ = 0.0;

    // 小车中心与无人机中心是两个不同的位置：
    //
    // 无人机圆心（ROS 原点）：
    //   场地尺寸坐标 = (horizontal=112.5, vertical=112.5) cm。
    //
    // 小车中心（A 点）：
    //   场地尺寸坐标 = (horizontal=150, vertical=200) cm。
    //
    // 把 A 点换成以无人机圆心为原点的 ROS 坐标：
    //   x = (200 - 112.5) / 100 = 0.875 m，x+ 向上；
    //   y = (112.5 - 150) / 100 = -0.375 m，y+ 向左。
    double car_display_x_ = 0.875;
    double car_display_y_ = -0.375;
    double car_altitude_ = 0.0;
};