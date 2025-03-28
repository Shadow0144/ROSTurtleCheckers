#include "TurtleGraveyard.hpp"

#include <QColor>
#include <QPointF>
#include <QRgb>

#include <math.h>

#include "rclcpp/rclcpp.hpp"

#include "CheckersConsts.hpp"

TurtleGraveyard::TurtleGraveyard(TurtlePieceColor owningPlayerColor, TurtlePieceColor viewingPlayerColor)
{
    if (owningPlayerColor == viewingPlayerColor)
    {
        m_left = WINDOW_WIDTH - TILE_WIDTH;
        m_nextPosition = QPointF(WINDOW_WIDTH - TILE_HALF_WIDTH, WINDOW_HEIGHT - TILE_HALF_HEIGHT);
        m_positionIncrement = QPointF(0, -TILE_HEIGHT);
    }
    else
    {
        m_left = 0;
        m_nextPosition = QPointF(TILE_HALF_WIDTH, HUD_HEIGHT + TILE_HALF_HEIGHT);
        m_positionIncrement = QPointF(0, TILE_HEIGHT);
    }
}

void TurtleGraveyard::addTurtlePiece(const std::shared_ptr<TileRender> &tile)
{
    auto &turtlePiece = tile->getTurtlePiece();
    if (turtlePiece)
    {
        m_slainTurtles.push_back(turtlePiece);
        turtlePiece->toggleIsDead(true);
        turtlePiece->move(m_nextPosition);
        tile->clearTurtlePiece();
        m_nextPosition += m_positionIncrement;
    }
}

void TurtleGraveyard::clear()
{
    m_slainTurtles.clear();
}

void TurtleGraveyard::paint(QPainter &painter)
{
    int r = GRAVEYARD_BG_RGB[0];
    int g = GRAVEYARD_BG_RGB[1];
    int b = GRAVEYARD_BG_RGB[2];
    QRgb graveyardColor = qRgb(r, g, b);
    painter.fillRect(m_left, HUD_HEIGHT, GRAVEYARD_WIDTH, BOARD_HEIGHT, graveyardColor);

    for (const auto &turtlePiece : m_slainTurtles)
    {
        turtlePiece->paint(painter);
    }
}