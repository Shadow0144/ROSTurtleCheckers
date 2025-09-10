#pragma once

#include <QWidget>
#include <QVBoxLayout>
#include <QLabel>
#include <QImage>

#include <chrono>

#include "shared/CheckersConsts.hpp"
#include "shared/TurtlePiece.hpp"
#include "player/TranslatedQLabel.hpp"

class GameWinnerOverlayWidget : public QWidget
{
public:
    GameWinnerOverlayWidget(QWidget *parent);

    void setPlayerColor(TurtlePieceColor playerColor);
    void setWinner(Winner winner);

    void reloadStrings();

private:
    QVBoxLayout *m_victoryLayout;

    TranslatedQLabel *m_winnerLabel;

    QLabel *m_winnerImageLabel;
    QLabel *m_loserImageLabel;
    QLabel *m_drawImageLabel;

    TurtlePieceColor m_playerColor;
    Winner m_winner;
};

typedef std::shared_ptr<GameWinnerOverlayWidget> GameWinnerOverlayWidgetPtr;