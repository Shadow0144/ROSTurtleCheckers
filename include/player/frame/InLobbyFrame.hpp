#pragma once

// #ifndef Q_MOC_RUN			  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
//  NO LINT
// #endif

#include <QFrame>
#include <QLabel>
#include <QPushButton>
#include <QCheckBox>
#include <QComboBox>

#include <memory>
#include <string>

#include "shared/CheckersConsts.hpp"

class CheckersPlayerWindow;

class InLobbyFrame : public QFrame
{
    Q_OBJECT
public:
    InLobbyFrame(CheckersPlayerWindow *parentWindow);
    ~InLobbyFrame();

    void showEvent(QShowEvent *event) override;

    void setLobbyInfo(const std::string &blackPlayerName,
                      const std::string &redPlayerName,
                      TurtlePieceColor lobbyOwnerColor,
                      bool blackPlayerReady,
                      bool redPlayerReady,
                      uint64_t timerSeconds);
    void playerJoinedLobby(const std::string &playerName, TurtlePieceColor playerColor);
    void playerLeftLobby(const std::string &playerName);
    void updateLobbyOwner(const std::string &lobbyOwnerPlayerName);
    void setLobbyOwnerColor(TurtlePieceColor lobbyOwnerColor);
    void setPlayerReady(const std::string &playerName, bool ready);
    void setTimer(uint64_t timerSeconds);

private:
    void handleBlackKickButton();
    void handleRedKickButton();

    void handleLeaveLobbyButton();

    void handleBlackReadyButtonToggled(int state);
    void handleRedReadyButtonToggled(int state);

    void handleTimerIndexChanged(int index);

    CheckersPlayerWindow *m_playerWindow;

    QCheckBox *m_blackReadyInLobbyCheckBox;
    QCheckBox *m_redReadyInLobbyCheckBox;

    QLabel *m_lobbyNameLabel;
    QLabel *m_lobbyIdLabel;
    QLabel *m_blackPlayerLobbyOwnerLabel;
    QLabel *m_redPlayerLobbyOwnerLabel;
    QLabel *m_blackPlayerNameLabel;
    QLabel *m_redPlayerNameLabel;

    QPushButton *m_blackPlayerKickButton;
    QPushButton *m_redPlayerKickButton;

    QComboBox *m_timerComboBox;

    std::string m_blackPlayerName;
    std::string m_redPlayerName;
    TurtlePieceColor m_lobbyOwnerColor;
    bool m_blackPlayerReady;
    bool m_redPlayerReady;
    std::chrono::seconds m_timer;
};

typedef std::unique_ptr<InLobbyFrame> InLobbyFrameUniPtr;