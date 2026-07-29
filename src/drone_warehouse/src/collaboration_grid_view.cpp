#include "drone_warehouse/collaboration_grid_view.hpp"

#include "drone_warehouse/color_palette.hpp"

#include <QFont>
#include <QPaintEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPen>

#include <algorithm>

namespace
{
// ======================== 场地图形参数 ========================
// 下面所有带 Cm 后缀的数值单位都是厘米。
// 如果现场尺寸或示意图发生变化，优先修改这里，不需要改后面的绘图公式。

constexpr qreal kCmPerMeter = 100.0;

// 直圆跑道外接矩形：左边距 150 cm、底边距 125 cm、宽 150 cm、高 300 cm。
// 圆角半径为 75 cm，所以跑道上下两端正好是半圆。
constexpr qreal kTrackLeftCm = 150.0;
constexpr qreal kTrackBottomCm = 125.0;
constexpr qreal kTrackWidthCm = 150.0;
constexpr qreal kTrackHeightCm = 300.0;
constexpr qreal kTrackRadiusCm = 75.0;

// 图片左下角圆形区域：
//   圆的左外沿距离场地左边 75 cm；
//   圆的下外沿距离场地底边 75 cm；
//   圆的半径是 37.5 cm。
//
// 所以圆心的场地尺寸坐标是：
//   horizontal = 75 + 37.5 = 112.5 cm；
//   vertical   = 75 + 37.5 = 112.5 cm。
//
// 这个圆心只作为无人机红点的初始中心，同时也是 ROS 坐标系的 (0, 0)。
// 小车黄色圆的初始中心是跑道 A 点，不是这个圆心。
constexpr qreal kCircleLeftClearanceCm = 75.0;
constexpr qreal kCircleBottomClearanceCm = 75.0;
constexpr qreal kOuterCircleRadiusCm = 37.5;
constexpr qreal kOriginHorizontalCm =
    kCircleLeftClearanceCm + kOuterCircleRadiusCm;
constexpr qreal kOriginVerticalCm =
    kCircleBottomClearanceCm + kOuterCircleRadiusCm;
constexpr qreal kInnerCircleRadiusCm = 25.0;

// 跑道上的四个命名点。
// Horizontal 表示从场地左边向右量，Vertical 表示从场地底边向上量。
// 注意：这只是绘制示意图使用的场地尺寸坐标，不是 ROS 的 x/y 坐标。
constexpr qreal kPointAHorizontalCm = 150.0;
constexpr qreal kPointAVerticalCm = 200.0;
constexpr qreal kPointBHorizontalCm = 150.0;
constexpr qreal kPointBVerticalCm = 350.0;
constexpr qreal kPointCHorizontalCm = 300.0;
constexpr qreal kPointCVerticalCm = 350.0;
constexpr qreal kPointDHorizontalCm = 300.0;
constexpr qreal kPointDVerticalCm = 200.0;

// 实时位置圆点使用固定像素半径，窗口缩放时仍然清晰可见。
// 如果希望圆点跟随场地一起缩放，可以改成“实际厘米尺寸 * mapScale()”。
constexpr qreal kPositionMarkerRadiusPx = 8.0;

// 圆心十字和命名点同样使用像素尺寸，只承担视觉提示作用。
constexpr qreal kOriginCrossHalfSizePx = 8.0;
constexpr qreal kNamedPointRadiusPx = 3.5;

// 绘制 A/B/C/D 的公共函数。
// point 已经是窗口像素坐标，因此这里不再做厘米或米的换算。
void drawNamedPoint(
    QPainter &painter,
    const QPointF &point,
    const QString &name)
{
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(30, 30, 30));
    painter.drawEllipse(
        point,
        kNamedPointRadiusPx,
        kNamedPointRadiusPx);

    // 标签向右下方稍微偏移，避免文字压住中心圆点。
    painter.setPen(ColorPalette::GrayDark);
    painter.drawText(
        point + QPointF(7.0, 5.0),
        name);
}
}

CollaborationGridView::CollaborationGridView(QWidget *parent)
    : QWidget(parent)
{
    // 只限制最小尺寸；实际场地会在 paintEvent 中按窗口大小等比例缩放。
    setMinimumSize(420, 360);
}

void CollaborationGridView::setdronePosition(
    double x,
    double y,
    double z)
{
    // ROS 坐标单位为米，方向约定：
    //   x 增大 -> 画面向上移动；
    //   y 增大 -> 画面向左移动；
    //   z 只用于界面文字显示，不参与二维位置计算。
    display_x_ = x;
    display_y_ = y;
    altitude_ = z;

    // 数据变化后请求 Qt 重新执行 paintEvent。
    update();
}

void CollaborationGridView::setcarPosition(
    double x,
    double y,
    double z)
{
    // 小车与无人机使用完全相同的坐标原点、方向和单位。
    car_display_x_ = x;
    car_display_y_ = y;
    car_altitude_ = z;
    update();
}

QRectF CollaborationGridView::mapRect() const
{
    // 给顶部坐标文字和场地边框预留空间。
    // 这些数值单位是窗口像素，不是厘米。
    const qreal left_margin = 55.0;
    const qreal right_margin = 30.0;
    const qreal top_margin = 45.0;
    const qreal bottom_margin = 45.0;

    // 防止窗口过小时得到 0 或负数，后续比例计算始终有合法结果。
    const qreal available_width =
        std::max<qreal>(
            1.0,
            width() - left_margin - right_margin);

    const qreal available_height =
        std::max<qreal>(
            1.0,
            height() - top_margin - bottom_margin);

    // 宽度方向和高度方向分别计算“每厘米占多少像素”，取较小值。
    // 因此场地一定完整显示，并始终保持 400:500 的真实长宽比例。
    const qreal scale = std::min(
        available_width / kMapWidthCm,
        available_height / kMapHeightCm);

    const qreal map_width = kMapWidthCm * scale;
    const qreal map_height = kMapHeightCm * scale;

    // 场地在可用区域内水平、竖直居中。
    const qreal left =
        left_margin + (available_width - map_width) / 2.0;

    const qreal top =
        top_margin + (available_height - map_height) / 2.0;

    return QRectF(left, top, map_width, map_height);
}

QPointF CollaborationGridView::mapPoint(
    qreal horizontal_cm,
    qreal vertical_cm) const
{
    const QRectF map = mapRect();
    const qreal scale = mapScale();

    // 场地尺寸图使用“左下角量尺寸”的方式：
    //   horizontal_cm 从场地左边向右增大；
    //   vertical_cm   从场地底边向上增大。
    //
    // Qt 窗口像素坐标的 Y 轴向下增大，所以竖直坐标必须使用
    // map.bottom() - vertical_cm * scale 才能实现“向上为正”。
    return QPointF(
        map.left() + horizontal_cm * scale,
        map.bottom() - vertical_cm * scale);
}

QRectF CollaborationGridView::mapObjectRect(
    qreal left_cm,
    qreal bottom_cm,
    qreal width_cm,
    qreal height_cm) const
{
    // QRectF 需要左上角，而现场尺寸通常从左下角给出。
    // 因此先把“底边 + 高度”换算成矩形左上角的场地坐标。
    const QPointF top_left =
        mapPoint(left_cm, bottom_cm + height_cm);

    const qreal scale = mapScale();

    return QRectF(
        top_left.x(),
        top_left.y(),
        width_cm * scale,
        height_cm * scale);
}

qreal CollaborationGridView::mapScale() const
{
    // mapRect 已保持 400:500 比例，因此用宽度或高度计算结果相同。
    // 返回值含义：现场 1 cm 在当前窗口中对应多少像素。
    return mapRect().width() / kMapWidthCm;
}

void CollaborationGridView::drawInspectionTrack(
    QPainter &painter) const
{
    const qreal scale = mapScale();

    // 先得到跑道的外接矩形，再通过 75 cm 圆角生成直圆跑道。
    const QRectF track_rect =
        mapObjectRect(
            kTrackLeftCm,
            kTrackBottomCm,
            kTrackWidthCm,
            kTrackHeightCm);

    QPainterPath track;
    track.addRoundedRect(
        track_rect,
        kTrackRadiusCm * scale,
        kTrackRadiusCm * scale,
        Qt::AbsoluteSize);

    painter.setBrush(Qt::NoBrush);
    painter.setPen(
        QPen(ColorPalette::GrayLight, 2.0));

    painter.drawPath(track);
}

void CollaborationGridView::drawCircularArea(
    QPainter &painter) const
{
    const qreal scale = mapScale();

    // 左下圆心既是图形中心，也是实时位置坐标系的原点。
    const QPointF center =
        mapPoint(
            kOriginHorizontalCm,
            kOriginVerticalCm);

    painter.setBrush(Qt::NoBrush);

    // 外圆半径是 37.5 cm，圆心为 (112.5, 112.5) cm。
    painter.setPen(
        QPen(ColorPalette::GrayLight, 2.0));
    painter.drawEllipse(
        center,
        kOuterCircleRadiusCm * scale,
        kOuterCircleRadiusCm * scale);

    // 内圆仅用于还原图片样式，半径可独立调整。
    painter.setPen(
        QPen(ColorPalette::BlueLight, 1.5));
    painter.drawEllipse(
        center,
        kInnerCircleRadiusCm * scale,
        kInnerCircleRadiusCm * scale);

    // 原点十字采用固定像素长度，红色无人机圆点会绘制在它上面。
    painter.drawLine(
        center + QPointF(-kOriginCrossHalfSizePx, 0.0),
        center + QPointF(kOriginCrossHalfSizePx, 0.0));

    painter.drawLine(
        center + QPointF(0.0, -kOriginCrossHalfSizePx),
        center + QPointF(0.0, kOriginCrossHalfSizePx));
}

void CollaborationGridView::drawCarMarker(
    QPainter &painter) const
{
    const qreal scale = mapScale();
    const QPointF origin =
        mapPoint(
            kOriginHorizontalCm,
            kOriginVerticalCm);

    // ROS 米坐标转换为窗口像素坐标：
    //
    //   car_display_x_ * 100     把米转换为厘米；
    //   再乘 scale              把厘米转换为像素。
    //
    // 坐标方向：
    //   x+ 向上，而 Qt 屏幕 Y 向下，所以屏幕 Y 要减去 x；
    //   y+ 向左，而 Qt 屏幕 X 向右，所以屏幕 X 要减去 y。
    const QPointF car_point(
        origin.x() -
            car_display_y_ * kCmPerMeter * scale,
        origin.y() -
            car_display_x_ * kCmPerMeter * scale);

    // 小车使用黄色圆点。深黄色描边用于浅色场地背景上的边界识别。
    painter.setPen(QPen(QColor(120, 95, 0), 1.5));
    painter.setBrush(ColorPalette::Yellow);
    painter.drawEllipse(
        car_point,
        kPositionMarkerRadiusPx,
        kPositionMarkerRadiusPx);
}

void CollaborationGridView::paintEvent(QPaintEvent *event)
{
    // 先让 QWidget 完成默认绘制流程，再覆盖绘制自定义场地。
    QWidget::paintEvent(event);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setRenderHint(
        QPainter::SmoothPixmapTransform,
        true);

    // 整个 Collaboration 页面背景。
    painter.fillRect(
        rect(),
        ColorPalette::BlueBlack);

    const QRectF map = mapRect();

    // 白色矩形是 400 cm x 500 cm 的实际场地边界。
    painter.setBrush(QColor(245, 247, 249));
    painter.setPen(
        QPen(ColorPalette::BlueLight, 2.0));
    painter.drawRect(map);

    // 第 1 层：绘制不随 ROS 数据变化的固定场地图形。
    drawInspectionTrack(painter);
    drawCircularArea(painter);

    // 第 2 层：先画黄色小车圆。
    // 默认小车中心就是 A 点，但此时还没有绘制 A 点的黑色中心圆和字母。
    drawCarMarker(painter);

    // 第 3 层：最后绘制 A/B/C/D 的黑色中心圆和字母。
    // A 点会画在黄色小车圆上层，因此小车中心和 A 点中心完全重合，
    // 同时仍然可以看到 A 点的小黑圆以及旁边的字母 A。
    QFont point_font = painter.font();
    point_font.setPixelSize(14);
    point_font.setBold(true);
    painter.setFont(point_font);

    drawNamedPoint(
        painter,
        mapPoint(
            kPointAHorizontalCm,
            kPointAVerticalCm),
        "A");

    drawNamedPoint(
        painter,
        mapPoint(
            kPointBHorizontalCm,
            kPointBVerticalCm),
        "B");

    drawNamedPoint(
        painter,
        mapPoint(
            kPointCHorizontalCm,
            kPointCVerticalCm),
        "C");

    drawNamedPoint(
        painter,
        mapPoint(
            kPointDHorizontalCm,
            kPointDVerticalCm),
        "D");

    const qreal scale = mapScale();
    const QPointF origin =
        mapPoint(
            kOriginHorizontalCm,
            kOriginVerticalCm);

    // 无人机使用与小车完全相同的坐标转换。
    // 默认 (0, 0) m 不产生偏移，所以红点位于左下圆心。
    const QPointF drone_point(
        origin.x() -
            display_y_ * kCmPerMeter * scale,
        origin.y() -
            display_x_ * kCmPerMeter * scale);

    painter.setPen(Qt::NoPen);
    painter.setBrush(ColorPalette::Red);
    painter.drawEllipse(
        drone_point,
        kPositionMarkerRadiusPx,
        kPositionMarkerRadiusPx);

    // 顶部只显示收到的 ROS 原始米坐标，不显示转换后的像素值。
    painter.setPen(ColorPalette::GrayLight);
    painter.drawText(
        QRectF(
            24.0,
            8.0,
            width() - 48.0,
            30.0),
        Qt::AlignLeft | Qt::AlignVCenter,
        QString(
            "drone  x=%1 m  y=%2 m  z=%3 m")
            .arg(display_x_, 0, 'f', 2)
            .arg(display_y_, 0, 'f', 2)
            .arg(altitude_, 0, 'f', 2));
}