#pragma once

#include <QWidget>
#include <QLabel>
#include <QPushButton>

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

    void reloadStrings();

private:
    std::string m_blackPlayerName;
    std::string m_redPlayerName;

    QLabel *m_blackPlayerNameLabel;
    QLabel *m_redPlayerNameLabel;

    QPushButton *m_joinLobbyButton;
};