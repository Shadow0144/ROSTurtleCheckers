#pragma once

#include <QWidget>
#include <QLabel>

#include <chrono>

#include "shared/CheckersConsts.hpp"
#include "shared/TurtlePiece.hpp"

class HUD : public QWidget
{
public:
    HUD(QWidget *parent);

    void setPlayerColor(TurtlePieceColor playerColor);
    void setPiecesRemaining(size_t blackPiecesRemaining, size_t redPiecesRemaining);
    void enableTimers(bool usingTimers);
    void setTimeRemaining(uint64_t blackTimeRemainingSec, uint64_t redTimeRemainingSec);
    void updateTimers();
    void setGameState(GameState gameState);
    void setWinner(Winner winner);

    void reloadStrings();

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

    QLabel *m_currentTurnLabel;
    QLabel *m_blackPiecesRemainingLabel;
    QLabel *m_blackTimeRemainingLabel;
    QLabel *m_redPiecesRemainingLabel;
    QLabel *m_redTimeRemainingLabel;
};

typedef std::shared_ptr<HUD> HUDPtr;