#include "TurtleGraveyard.hpp"

#include <QColor>
#include <QPointF>
#include <QRgb>

#include "CheckersConsts.hpp"
#include "TurtlePieceRender.hpp"

TurtleGraveyard::TurtleGraveyard(TurtlePieceColor owningPlayerColor, TurtlePieceColor viewingPlayerColor)
{
    if (owningPlayerColor == viewingPlayerColor)
    {
        m_left = WINDOW_WIDTH - TILE_WIDTH - TILE_WIDTH;
        m_nextPosition = QPointF(WINDOW_WIDTH - TILE_HALF_WIDTH - TILE_WIDTH, WINDOW_HEIGHT - TILE_HALF_HEIGHT);
        m_initialRowPosition = m_nextPosition;
        m_positionRowIncrement = QPointF(0, -TILE_HEIGHT);
        m_positionColIncrement = QPointF(TILE_WIDTH, 0);
    }
    else
    {
        m_left = 0;
        m_nextPosition = QPointF(TILE_HALF_WIDTH + TILE_WIDTH, HUD_HEIGHT + TILE_HALF_HEIGHT);
        m_initialRowPosition = m_nextPosition;
        m_positionRowIncrement = QPointF(0, TILE_HEIGHT);
        m_positionColIncrement = QPointF(-TILE_WIDTH, 0);
    }
    m_turtlesInColumn = 0u;
}

void TurtleGraveyard::addTurtlePiece(TurtlePieceRenderPtr &turtlePieceRender)
{
    if (turtlePieceRender)
    {
        m_slainTurtles.push_back(turtlePieceRender);
        turtlePieceRender->setIsDead(true);
        turtlePieceRender->setCenterPosition(m_nextPosition);
        m_nextPosition += m_positionRowIncrement;
        m_turtlesInColumn++;
        if (m_turtlesInColumn == NUM_PLAYABLE_ROWS)
        {
            m_nextPosition.setY(m_initialRowPosition.y());
            m_nextPosition += m_positionColIncrement;
            m_turtlesInColumn = 0u;
        }
    }
}

void TurtleGraveyard::clear()
{
    m_slainTurtles.clear();
}

void TurtleGraveyard::paint(QPainter &painter) const
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