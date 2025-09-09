#include "player/TurtleGraveyard.hpp"

#include <QWidget>
#include <QLabel>
#include <QPoint>

#include <algorithm>
#include <iostream>

#include "shared/CheckersConsts.hpp"
#include "player/TurtlePieceRender.hpp"

TurtleGraveyard::TurtleGraveyard(QWidget *parent, int x, bool left)
    : QWidget(parent)
{
    const auto padding = GRAVEYARD_PADDING;
    m_turtlePieceSize = std::min(PIECE_SIZE, (BOARD_HEIGHT / (NUM_PIECES_PER_PLAYER + 1)) - (2 * padding));

    m_x = x;
    m_y = BOARD_TOP;
    m_left = left;

    setGeometry(m_x, m_y, GRAVEYARD_WIDTH, BOARD_HEIGHT);

    if (left) // Left
    {
        m_turtleCenterInitialY = BOARD_TOP + TILE_HALF_HEIGHT + padding;
        m_turtleCenterYStep = std::min(TILE_HEIGHT, static_cast<int>(m_turtlePieceSize + padding));
    }
    else // Right
    {
        m_turtleCenterInitialY = (BOARD_TOP + BOARD_HEIGHT) - TILE_HALF_HEIGHT - padding;
        m_turtleCenterYStep = -std::min(TILE_HEIGHT, static_cast<int>(m_turtlePieceSize + padding));
    }
    m_turtleCenterX = m_x + (GRAVEYARD_WIDTH) / 2;
    m_turtleCenterY = m_turtleCenterInitialY;
}

void TurtleGraveyard::addTurtlePiece(TurtlePieceRenderPtr &turtlePieceRender)
{
    if (turtlePieceRender)
    {
        turtlePieceRender->setCenterPosition(QPoint(m_turtleCenterX, m_turtleCenterY));
        m_turtlePieceRenders.push_back(turtlePieceRender);
        m_turtleCenterY += m_turtleCenterYStep;
    }
}

void TurtleGraveyard::clear()
{
    m_turtlePieceRenders.clear();
    m_turtleCenterY = m_turtleCenterInitialY;
}

void TurtleGraveyard::paint(QPainter &painter)
{
    QRgb backgroundColor = qRgb(GRAVEYARD_BG_RGB[0], GRAVEYARD_BG_RGB[1], GRAVEYARD_BG_RGB[2]);
    painter.fillRect(m_x, m_y, GRAVEYARD_WIDTH, BOARD_HEIGHT, backgroundColor);

    for (auto &turtlePieceRender : m_turtlePieceRenders)
    {
        turtlePieceRender->paint(painter);
    }
}