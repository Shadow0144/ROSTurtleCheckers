#include "TileRender.hpp"

#include <QColor>
#include <QRgb>

#include <math.h>

#include "rclcpp/rclcpp.hpp"

#include "CheckersConsts.hpp"

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
    reachabled = false;
    highlighted = false;
    selected = false;
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

void TileRender::setTurtlePiece(const TurtlePiecePtr &turtle)
{
    contained_turtle = turtle;
}

TurtlePiecePtr &TileRender::getTurtlePiece()
{
    return contained_turtle;
}

bool TileRender::containsPiece(TurtlePieceColor color)
{
    return (contained_turtle && (contained_turtle->getColor() == color));
}

bool TileRender::getPieceHighlight()
{
    if (contained_turtle)
    {
        return contained_turtle->getHighlighted();
    }
    return false;
}

bool TileRender::togglePieceHighlight()
{
    if (contained_turtle)
    {
        contained_turtle->toggleHighlight();
        return contained_turtle->getHighlighted();
    }
    return false;
}

void TileRender::togglePieceHighlight(bool highlight)
{
    if (contained_turtle)
    {
        contained_turtle->toggleHighlight(highlight);
    }
}

bool TileRender::getPieceSelect()
{
    if (contained_turtle)
    {
        return contained_turtle->getHighlighted();
    }
    return false;
}

bool TileRender::togglePieceSelect()
{
    if (contained_turtle)
    {
        contained_turtle->toggleSelect();
        return contained_turtle->getSelected();
    }
    return false;
}

void TileRender::togglePieceSelect(bool select)
{
    if (contained_turtle)
    {
        contained_turtle->toggleSelect(select);
    }
}

bool TileRender::getTileReachable()
{
    return reachabled;
}

void TileRender::toggleTileReachable()
{
    reachabled = !reachabled;
}

void TileRender::toggleTileReachable(bool reachable)
{
    reachabled = reachable;
}

bool TileRender::getTileHighlight()
{
    return highlighted;
}

void TileRender::toggleTileHighlight()
{
    highlighted = !highlighted;
}

void TileRender::toggleTileHighlight(bool highlight)
{
    highlighted = highlight;
}

bool TileRender::getTileSelect()
{
    return selected;
}

void TileRender::toggleTileSelect()
{
    selected = !selected;
}

void TileRender::toggleTileSelect(bool select)
{
    selected = select;
}

void TileRender::paint(QPainter &painter)
{
    int r = BLACK_SQUARES_BG_RGB[0];
    int g = BLACK_SQUARES_BG_RGB[1];
    int b = BLACK_SQUARES_BG_RGB[2];
    if (selected)
    {
        r = SELECTED_SQUARES_BG_RGB[0];
        g = SELECTED_SQUARES_BG_RGB[1];
        b = SELECTED_SQUARES_BG_RGB[2];
    }
    else if (highlighted)
    {
        r = HIGHLIGHTED_SQUARES_BG_RGB[0];
        g = HIGHLIGHTED_SQUARES_BG_RGB[1];
        b = HIGHLIGHTED_SQUARES_BG_RGB[2];
    }
    else if (reachabled)
    {
        r = REACHABLED_SQUARES_BG_RGB[0];
        g = REACHABLED_SQUARES_BG_RGB[1];
        b = REACHABLED_SQUARES_BG_RGB[2];
    }
    QRgb tile_color = qRgb(r, g, b);
    painter.fillRect(left, top, tile_width_, tile_height_, tile_color);
}