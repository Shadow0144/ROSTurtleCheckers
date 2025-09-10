#pragma once

#include <QWidget>
#include <QLabel>
#include <QPushButton>

#include <string>
#include <vector>
#include <functional>

#include "player/PlayerIconNameWidget.hpp"
#include "player/TranslatedQPushButton.hpp"

class LobbyDetailsWidget : public QWidget
{
public:
    LobbyDetailsWidget(QWidget *parent,
                       const std::string &lobbyName,
                       const std::string &lobbyId,
                       const std::string &blackPlayerName,
                       const std::string &redPlayerName,
                       bool hasPassword,
                       const std::function<void()> &onJoinFunction);

    void reloadStrings();

private:
    std::string m_blackPlayerName;
    std::string m_redPlayerName;

    PlayerIconNameWidget *m_blackPlayerIconNameWidget;
    PlayerIconNameWidget *m_redPlayerIconNameWidget;

    TranslatedQPushButton *m_joinLobbyButton;
};