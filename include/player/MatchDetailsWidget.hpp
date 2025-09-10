#pragma once

#include <QWidget>
#include <QLabel>

#include <string>
#include <vector>
#include <functional>

#include "shared/CheckersConsts.hpp"
#include "player/TranslatedQLabel.hpp"

class MatchDetailsWidget : public QWidget
{
public:
    MatchDetailsWidget(QWidget *parent,
                       const std::string &playerName,
                       const std::string &lobbyNameId,
                       const std::string &blackPlayerName,
                       const std::string &redPlayerName,
                       Winner winner);

    void reloadStrings();

private:
    Winner m_winner;

    std::string m_winningPlayerName;

    TranslatedQLabel *m_winnerLabel;
};