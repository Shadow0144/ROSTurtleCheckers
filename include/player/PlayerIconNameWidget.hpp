#pragma once

#include <QWidget>
#include <QLabel>

#include <string>
#include <vector>
#include <functional>

#include "shared/CheckersConsts.hpp"

class PlayerIconNameWidget : public QWidget
{
public:
    PlayerIconNameWidget(TurtlePieceColor playerColor,
                         const std::string &playerName = "");

    void setPlayerName(const std::string &playerName);
    const std::string &getPlayerName() const;
    void clearPlayerName();

    TurtlePieceColor getPlayerColor() const;

    void reloadStrings();

private:
    TurtlePieceColor m_playerColor;
    std::string m_playerName;

    QLabel *m_playerIconLabel;
    QLabel *m_playerNameLabel;
};