#include "TileRender.hpp"

#include <QColor>
#include <QRgb>

#include <math.h>

#include "rclcpp/rclcpp.hpp"

#include "CheckersConsts.hpp"

TileRender::TileRender(const QPointF &centerPosition)
    : m_centerPosition(centerPosition)
{
    m_left = m_centerPosition.x() - TILE_HALF_WIDTH;
    m_top = m_centerPosition.y() - TILE_HALF_HEIGHT;
    m_right = m_centerPosition.x() + TILE_HALF_WIDTH;
    m_bottom = m_centerPosition.y() + TILE_HALF_HEIGHT;
    m_isReachable = false;
    m_isHighlighted = false;
    m_isSelected = false;
    m_isLastSelected = false;
}

QPointF TileRender::getCenterPosition() const
{
    return m_centerPosition;
}

bool TileRender::containsPoint(QPoint point) const
{
    return (point.x() >= m_left &&
            point.y() >= m_top &&
            point.x() <= m_right &&
            point.y() <= m_bottom);
}

void TileRender::setTurtlePiece(const TurtlePiecePtr &turtle)
{
    m_containedTurtle = turtle;
    if (m_containedTurtle)
    {
        m_containedTurtle->move(m_centerPosition);
    }
}

const TurtlePiecePtr &TileRender::getTurtlePiece() const
{
    return m_containedTurtle;
}

TurtlePieceColor TileRender::getTurtlePieceColor() const
{
    if (m_containedTurtle)
    {
        return m_containedTurtle->getColor();
    }
    return TurtlePieceColor::None;
}

void TileRender::moveTurtlePiece(const TileRenderPtr &destinationTile)
{
    destinationTile->setTurtlePiece(m_containedTurtle);
    m_containedTurtle.reset();
}

void TileRender::clearTurtlePiece()
{
    m_containedTurtle.reset();
}

bool TileRender::containsPiece(TurtlePieceColor color) const
{
    return (m_containedTurtle && (m_containedTurtle->getColor() == color));
}

void TileRender::kingTurtlePiece()
{
    if (m_containedTurtle)
    {
        m_containedTurtle->toggleIsKinged(true);
    }
}

bool TileRender::getIsPieceHighlighted() const
{
    if (m_containedTurtle)
    {
        return m_containedTurtle->getIsHighlighted();
    }
    return false;
}

bool TileRender::toggleIsPieceHighlighted()
{
    if (m_containedTurtle)
    {
        m_containedTurtle->toggleIsHighlighted();
        return m_containedTurtle->getIsHighlighted();
    }
    return false;
}

void TileRender::toggleIsPieceHighlighted(bool isHighlighted)
{
    if (m_containedTurtle)
    {
        m_containedTurtle->toggleIsHighlighted(isHighlighted);
    }
}

bool TileRender::getIsPieceSelected() const
{
    if (m_containedTurtle)
    {
        return m_containedTurtle->getIsSelected();
    }
    return false;
}

bool TileRender::toggleIsPieceSelected()
{
    if (m_containedTurtle)
    {
        m_containedTurtle->toggleIsSelected();
        return m_containedTurtle->getIsSelected();
    }
    return false;
}

void TileRender::toggleIsPieceSelected(bool isSelected)
{
    if (m_containedTurtle)
    {
        m_containedTurtle->toggleIsSelected(isSelected);
    }
}

bool TileRender::getIsTileReachable() const
{
    return m_isReachable;
}

void TileRender::toggleIsTileReachable()
{
    m_isReachable = !m_isReachable;
}

void TileRender::toggleIsTileReachable(bool isReachable)
{
    m_isReachable = isReachable;
}

bool TileRender::getIsTileHighlighted() const
{
    return m_isHighlighted;
}

void TileRender::toggleIsTileHighlighted()
{
    m_isHighlighted = !m_isHighlighted;
}

void TileRender::toggleIsTileHighlighted(bool isHighlighted)
{
    m_isHighlighted = isHighlighted;
}

bool TileRender::getIsTileSelected() const
{
    return m_isSelected;
}

void TileRender::toggleIsTileSelected()
{
    m_isSelected = !m_isSelected;
}

void TileRender::toggleIsTileSelected(bool isSelected)
{
    m_isSelected = isSelected;
}

bool TileRender::getIsTileLastSelected() const
{
    return m_isLastSelected;
}

void TileRender::toggleIsTileLastSelected()
{
    m_isLastSelected = !m_isLastSelected;
}

void TileRender::toggleIsTileLastSelected(bool isLastSelected)
{
    m_isLastSelected = isLastSelected;
}

void TileRender::paint(QPainter &painter)
{
    int r = BLACK_SQUARES_BG_RGB[0];
    int g = BLACK_SQUARES_BG_RGB[1];
    int b = BLACK_SQUARES_BG_RGB[2];
    if (m_isSelected)
    {
        r = SELECTED_SQUARES_BG_RGB[0];
        g = SELECTED_SQUARES_BG_RGB[1];
        b = SELECTED_SQUARES_BG_RGB[2];
    }
    else if (m_isHighlighted)
    {
        r = HIGHLIGHTED_SQUARES_BG_RGB[0];
        g = HIGHLIGHTED_SQUARES_BG_RGB[1];
        b = HIGHLIGHTED_SQUARES_BG_RGB[2];
    }
    else if (m_isReachable)
    {
        r = REACHABLE_SQUARES_BG_RGB[0];
        g = REACHABLE_SQUARES_BG_RGB[1];
        b = REACHABLE_SQUARES_BG_RGB[2];
    }
    else if (m_isLastSelected)
    {
        r = LAST_SELECTED_SQUARES_BG_RGB[0];
        g = LAST_SELECTED_SQUARES_BG_RGB[1];
        b = LAST_SELECTED_SQUARES_BG_RGB[2];
    }
    QRgb tileColor = qRgb(r, g, b);
    painter.fillRect(m_left, m_top, TILE_WIDTH, TILE_HEIGHT, tileColor);
}