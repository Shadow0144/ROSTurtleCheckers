#pragma once

#include <QPainter>
#include <QPen>
#include <QFont>
#include <QImage>
#include <QPointF>
#include <QRect>

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
    void setWinner(Winner winner);

    void paint(QPainter &painter);

private:
    TurtlePieceColor m_playerColor;
    size_t m_blackPiecesRemaining;
    size_t m_redPiecesRemaining;
    GameState m_gameState;
    Winner m_winner;

    QFont m_turtleFont;
    QPen m_turtlePen;

    QFont m_victoryFont;
    QPen m_victoryPen;
    QRect m_victoryTextCenteringRect;
    QPointF m_victoryImagePosition;
};

typedef std::shared_ptr<HUD> HUDPtr;