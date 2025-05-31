#pragma once

// #ifndef Q_MOC_RUN			  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
//  NO LINT
// #endif

#include <QFrame>
#include <QLineEdit>
#include <QImage>
#include <QLabel>
#include <QPushButton>
#include <QRadioButton>
#include <QButtonGroup>
#include <QCheckBox>
#include <QPainter>
#include <QPaintEvent>
#include <QVector>

#include <memory>
#include <string>
#include <vector>

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
                      bool redPlayerReady);
    void playerJoinedLobby(const std::string &playerName, TurtlePieceColor playerColor);
    void playerLeftLobby(const std::string &playerName);
    void updateLobbyOwner(const std::string &lobbyOwnerPlayerName);
    void setLobbyOwnerColor(TurtlePieceColor lobbyOwnerColor);
    void setPlayerReady(const std::string &playerName, bool ready);

private:
    void handleBlackKickButton();
    void handleRedKickButton();

    void handleLeaveLobbyButton();

    void handleBlackReadyButtonToggled(int state);
    void handleRedReadyButtonToggled(int state);

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

    std::string m_blackPlayerName;
    std::string m_redPlayerName;
    TurtlePieceColor m_lobbyOwnerColor;
    bool m_blackPlayerReady;
    bool m_redPlayerReady;
};

typedef std::unique_ptr<InLobbyFrame> InLobbyFrameUniPtr;