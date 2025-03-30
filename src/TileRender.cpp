#include "TileRender.hpp"

#include <QPainter>
#include <QRgb>

#include "CheckersConsts.hpp"

#include <iostream>

TileRender::TileRender(int row, int col, const QPointF &centerPosition)
    : Tile(row, col),
      m_centerPosition(centerPosition)
{
    m_left = m_centerPosition.x() - TILE_HALF_WIDTH;
    m_top = m_centerPosition.y() - TILE_HALF_HEIGHT;
    m_right = m_centerPosition.x() + TILE_HALF_WIDTH;
    m_bottom = m_centerPosition.y() + TILE_HALF_HEIGHT;
}

const QPointF &TileRender::getCenterPosition() const
{
    return m_centerPosition;
}

bool TileRender::containsPoint(const QPointF &point) const
{
    return (point.x() >= m_left &&
            point.y() >= m_top &&
            point.x() <= m_right &&
            point.y() <= m_bottom);
}

void TileRender::setTurtlePiece(const TurtlePiecePtr &turtle)
{
    m_containedTurtle = turtle;
    std::static_pointer_cast<TurtlePieceRender>(m_containedTurtle)->setCenterPosition(m_centerPosition);
}

void TileRender::moveTurtlePiece(const TileRenderPtr &destinationTileRender)
{
    if (m_containedTurtle)
    {
        destinationTileRender->m_containedTurtle = m_containedTurtle;
        std::static_pointer_cast<TurtlePieceRender>(destinationTileRender->m_containedTurtle)->setCenterPosition(destinationTileRender->getCenterPosition());
    }
    m_containedTurtle.reset();
}

void TileRender::moveTurtlePiece(const TurtleGraveyardPtr &destinationGraveyard)
{
    auto turtleRender = std::static_pointer_cast<TurtlePieceRender>(m_containedTurtle);
    destinationGraveyard->addTurtlePiece(turtleRender);
    m_containedTurtle.reset();
}

void TileRender::paint(QPainter &painter) const
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
    else if (m_isLastMovedFrom)
    {
        r = LAST_MOVED_FROM_SQUARES_BG_RGB[0];
        g = LAST_MOVED_FROM_SQUARES_BG_RGB[1];
        b = LAST_MOVED_FROM_SQUARES_BG_RGB[2];
    }
    else if (m_isLastMovedTo)
    {
        r = LAST_MOVED_TO_SQUARES_BG_RGB[0];
        g = LAST_MOVED_TO_SQUARES_BG_RGB[1];
        b = LAST_MOVED_TO_SQUARES_BG_RGB[2];
    }
    else if (m_isLastJumpedOver)
    {
        r = LAST_JUMPED_OVER_SQUARES_BG_RGB[0];
        g = LAST_JUMPED_OVER_SQUARES_BG_RGB[1];
        b = LAST_JUMPED_OVER_SQUARES_BG_RGB[2];
    }
    QRgb tileColor = qRgb(r, g, b);
    painter.fillRect(m_left, m_top, TILE_WIDTH, TILE_HEIGHT, tileColor);
    if (m_containedTurtle)
    {
        std::static_pointer_cast<TurtlePieceRender>(m_containedTurtle)->paint(painter);
    }
}