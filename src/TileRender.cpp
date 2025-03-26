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
}

QPointF TileRender::getCenterPosition()
{
    return m_centerPosition;
}

bool TileRender::containsPoint(QPoint point)
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

TurtlePiecePtr &TileRender::getTurtlePiece()
{
    return m_containedTurtle;
}

void TileRender::moveTurtlePiece(const TileRenderPtr &destinationTile)
{
    destinationTile->setTurtlePiece(m_containedTurtle);
    m_containedTurtle.reset();
}

bool TileRender::containsPiece(TurtlePieceColor color)
{
    return (m_containedTurtle && (m_containedTurtle->getColor() == color));
}

bool TileRender::getIsPieceHighlighted()
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

bool TileRender::getIsPieceSelected()
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

bool TileRender::getIsTileReachable()
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

bool TileRender::getIsTileHighlighted()
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

bool TileRender::getIsTileSelected()
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
    QRgb tileColor = qRgb(r, g, b);
    painter.fillRect(m_left, m_top, TILE_WIDTH, TILE_HEIGHT, tileColor);
}