#pragma once

#include <QPainter>
#include <QPen>
#include <QFont>
#include <QImage>
#include <QPointF>
#include <QRect>
#include <QString>

#include <chrono>
#include <memory>

#include "shared/CheckersConsts.hpp"
#include "shared/TurtlePiece.hpp"

class HUD
{
public:
    HUD();

    void setPlayerColor(TurtlePieceColor playerColor);
    void setPiecesRemaining(size_t blackPiecesRemaining, size_t redPiecesRemaining);
    void enableTimers(bool usingTimers);
    void setTimeRemaining(uint64_t blackTimeRemainingSec, uint64_t redTimeRemainingSec);
    void setGameState(GameState gameState);
    void setWinner(Winner winner);

    void paint(QPainter &painter);

private:
    QString formatTimeRemaining(uint64_t timeRemaining);

    TurtlePieceColor m_playerColor;
    size_t m_blackPiecesRemaining;
    size_t m_redPiecesRemaining;
    uint64_t m_blackTimeRemainingSec;
    uint64_t m_redTimeRemainingSec;
    GameState m_gameState;
    Winner m_winner;

    bool m_usingTimers;
    std::chrono::system_clock::time_point m_lastTimestamp;

    QFont m_turtleFont;
    QPen m_turtlePen;

    QImage m_blackTurtleIcon;
    QImage m_redTurtleIcon;

    QFont m_victoryFont;
    QPen m_victoryPen;
    QRect m_victoryTextCenteringRect;
    QPointF m_victoryImagePosition;
    
    QRect m_blackRemainingTextRightingRect;
    QRect m_redRemainingTextRightingRect;

    QPen m_timerPen;
    QPen m_activeTimerPen;
    
    QPointF m_blackTurtleIconPosition;
    QPointF m_redTurtleIconPosition;
};

typedef std::shared_ptr<HUD> HUDPtr;