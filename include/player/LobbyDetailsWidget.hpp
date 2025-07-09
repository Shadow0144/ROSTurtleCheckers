#pragma once

#include <QWidget>

#include <memory>
#include <string>
#include <vector>
#include <functional>

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
};

typedef std::shared_ptr<LobbyDetailsWidget> LobbyDetailsWidgetPtr;