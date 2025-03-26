#pragma once

#include <QPainter>
#include <QPen>
#include <QFont>
#include <QPointF>

#include "CheckersConsts.hpp"
#include "TurtlePiece.hpp"

#include <memory>

class HUD
{
public:
    HUD();

    void setPlayerColor(TurtlePieceColor playerColor);
    void setPiecesRemaining(size_t blackPiecesRemaining, size_t redPiecesRemaining);
    void setGameState(GameState gameState);

    void paint(QPainter &painter);

private:
    TurtlePieceColor m_playerColor;
    size_t m_blackPiecesRemaining;
    size_t m_redPiecesRemaining;
    GameState m_gameState;

    QFont m_turtleFont;
    QPen m_turtlePen;
};

typedef std::shared_ptr<HUD> HUDPtr;