#pragma once

// #ifndef Q_MOC_RUN			  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
//  NO LINT
// #endif

#include <QFrame>
#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QCheckBox>
#include <QComboBox>
#include <QTimer>

#include <memory>
#include <string>
#include <chrono>

#include "shared/CheckersConsts.hpp"
#include "player/LanguageSelectorWidget.hpp"
#include "player/TitleWidget.hpp"
#include "player/DialogWidget.hpp"
#include "player/ChatBox.hpp"

class CheckersPlayerWindow;

class InLobbyFrame : public QFrame
{
    Q_OBJECT
public:
    InLobbyFrame(CheckersPlayerWindow *parentWindow);
    ~InLobbyFrame();

    void showEvent(QShowEvent *event) override;
    void hideEvent(QHideEvent *event) override;

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

    void clearChat();

    // Reply from server
    void addChatMessage(const std::string &playerName,
                        TurtlePieceColor playerColor,
                        const std::string &chatMessage,
                        std::chrono::time_point<std::chrono::system_clock> timeStamp);

    // Send to server
    void reportPlayer(const std::string &chatMessages);
    void sendChatMessage(const std::string &chatMessage);

    void reloadStrings();

private:
    void handleBlackKickButton();
    void handleRedKickButton();

    void handleLeaveLobbyButton();

    void handleReportPlayerConfirmButton();
    void handleReportPlayerCancelButton();

    void handleBlackReadyButtonToggled(int state);
    void handleRedReadyButtonToggled(int state);

    void handleTimerIndexChanged(int index);

    void updateGameStartTimer();

    CheckersPlayerWindow *m_playerWindow;

    LanguageSelectorWidget *m_languageSelector;

    TitleWidget *m_titleWidget;

    QWidget *m_inLobbyWidget;

    QCheckBox *m_blackReadyInLobbyCheckBox;
    QCheckBox *m_redReadyInLobbyCheckBox;

    QLabel *m_lobbyNameLabel;
    QLabel *m_lobbyIdLabel;
    QLabel *m_blackPlayerLobbyOwnerLabel;
    QLabel *m_redPlayerLobbyOwnerLabel;
    QLabel *m_blackPlayerNameLabel;
    QLabel *m_redPlayerNameLabel;
    QLabel *m_gameStartTimerLabel;
    QLabel *m_timerLabel;

    std::chrono::seconds m_secondsBeforeStart;
    const std::chrono::seconds MAX_SECONDS_BEFORE_START{3u};
    QTimer *m_gameStartTimer;

    QPushButton *m_blackPlayerKickButton;
    QPushButton *m_redPlayerKickButton;
    QPushButton *m_leaveLobbyInLobbyButton;

    QComboBox *m_timerComboBox;

    ChatBox *m_chatBox;

    DialogWidget *m_reportPlayerConfirmDialog;
    std::string m_reportingChatMessages;

    std::string m_blackPlayerName;
    std::string m_redPlayerName;
    TurtlePieceColor m_lobbyOwnerColor;
    bool m_blackPlayerReady;
    bool m_redPlayerReady;
    std::chrono::seconds m_timer;
};

typedef std::unique_ptr<InLobbyFrame> InLobbyFrameUniPtr;