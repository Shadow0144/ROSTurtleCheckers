#pragma once

#include <QWidget>

#include <string>
#include <vector>
#include <functional>

#include "shared/CheckersConsts.hpp"

class MatchDetailsWidget : public QWidget
{
public:
    MatchDetailsWidget(QWidget *parent,
                       const std::string &playerName,
                       const std::string &lobbyNameId,
                       const std::string &blackPlayerName,
                       const std::string &redPlayerName,
                       Winner winner);
};