#include "TileRender.hpp"

#include <QColor>
#include <QRgb>

#include <math.h>

#include "rclcpp/rclcpp.hpp"

constexpr int HIGHLIGHTED_SQUARES_BG_RGB[3] = {0u, 0u, 255u};
constexpr int BLACK_SQUARES_BG_RGB[3] = {0u, 0u, 0u};

TileRender::TileRender(const QPointF &center_position, size_t tile_width, size_t tile_height)
    : center_position_(center_position)
{
    tile_width_ = tile_width;
    tile_height_ = tile_height;
    auto tile_half_width = 0.5f * tile_width_;
    auto tile_half_height = 0.5f * tile_height_;
    left = center_position_.x() - tile_half_width;
    top = center_position_.y() - tile_half_height;
    right = center_position_.x() + tile_half_width;
    bottom = center_position_.y() + tile_half_height;
    highlighted = false;
}

QPointF TileRender::getCenterPosition()
{
    return center_position_;
}

bool TileRender::containsPoint(QPoint point)
{
    return point.x() >= left &&
           point.y() >= top &&
           point.x() <= right &&
           point.y() <= bottom;
}

void TileRender::toggleHighlight()
{
    highlighted = !highlighted;
}

void TileRender::toggleHighlight(bool highlight)
{
    highlighted = highlight;
}

void TileRender::paint(QPainter &painter)
{
    int r = (highlighted) ? HIGHLIGHTED_SQUARES_BG_RGB[0] : BLACK_SQUARES_BG_RGB[0];
    int g = (highlighted) ? HIGHLIGHTED_SQUARES_BG_RGB[1] : BLACK_SQUARES_BG_RGB[1];
    int b = (highlighted) ? HIGHLIGHTED_SQUARES_BG_RGB[2] : BLACK_SQUARES_BG_RGB[2];
    QRgb background_color = qRgb(r, g, b);
    painter.fillRect(left, top, tile_width_, tile_height_, background_color);
}