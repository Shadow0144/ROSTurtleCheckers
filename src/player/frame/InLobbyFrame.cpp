#include "player/frame/InLobbyFrame.hpp"

#include <QFrame>
#include <QMouseEvent>
#include <QPaintEvent>
#include <QPointF>
#include <QStackedLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QScrollArea>
#include <QLabel>
#include <QString>
#include <QRegularExpression>
#include <QRegularExpressionValidator>
#include <QPushButton>
#include <QRadioButton>
#include <QButtonGroup>
#include <QCheckBox>
#include <QIcon>
#include <QPixmap>
#include <QSpacerItem>
#include <QStyle>
#include <QSizePolicy>

#include <cstdlib>
#include <ctime>
#include <functional>
#include <memory>
#include <string>
#include <chrono>
#include <vector>
#include <iostream>

#include "shared/CheckersConsts.hpp"
#include "player/Parameters.hpp"
#include "player/CheckersPlayerWindow.hpp"
#include "player/ImageLibrary.hpp"

InLobbyFrame::InLobbyFrame(
    CheckersPlayerWindow *parentWindow)
    : QFrame(parentWindow, Qt::WindowFlags())
{
    m_playerWindow = parentWindow;
    m_blackPlayerName = "";
    m_redPlayerName = "";
    m_blackPlayerReady = false;
    m_redPlayerReady = false;

    auto inLobbyLayout = new QVBoxLayout(this);
    inLobbyLayout->setAlignment(Qt::AlignCenter);

    auto inLobbyTitleLabel = new QLabel("Turtle Checkers");
    auto titleFont = inLobbyTitleLabel->font();
    titleFont.setPointSize(TITLE_FONT_SIZE);
    inLobbyTitleLabel->setFont(titleFont);
    inLobbyLayout->addWidget(inLobbyTitleLabel);

    auto lobbyNameLayout = new QHBoxLayout();

    m_lobbyNameLabel = new QLabel("");
    lobbyNameLayout->addWidget(m_lobbyNameLabel);

    m_lobbyIdLabel = new QLabel("");
    lobbyNameLayout->addWidget(m_lobbyIdLabel);

    inLobbyLayout->addLayout(lobbyNameLayout);

    std::string readyInLobbyString = "Ready";

    auto blackPlayerLayout = new QHBoxLayout();

    m_blackReadyInLobbyCheckBox = new QCheckBox(readyInLobbyString.c_str());
    blackPlayerLayout->addWidget(m_blackReadyInLobbyCheckBox);

    auto blackTurtleIconLabel = new QLabel();
    auto blackTurtleIcon = QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::Black));
    auto scaledBlackTurtleIcon = blackTurtleIcon.scaled(ICON_HEIGHT_WIDTH, ICON_HEIGHT_WIDTH,
                                                        Qt::KeepAspectRatio, Qt::SmoothTransformation);
    blackTurtleIconLabel->setPixmap(scaledBlackTurtleIcon);
    blackPlayerLayout->addWidget(blackTurtleIconLabel);

    std::string openString = "Open";
    bool blackPlayerJoined = !m_blackPlayerName.empty();
    bool redPlayerJoined = !m_redPlayerName.empty();

    m_blackPlayerNameLabel = new QLabel(blackPlayerJoined ? m_blackPlayerName.c_str() : openString.c_str());
    m_blackPlayerNameLabel->setEnabled(blackPlayerJoined);
    blackPlayerLayout->addWidget(m_blackPlayerNameLabel);

    m_blackReadyInLobbyCheckBox->setEnabled(false);
    connect(m_blackReadyInLobbyCheckBox, &QCheckBox::stateChanged, this,
            &InLobbyFrame::handleBlackReadyButtonToggled);

    inLobbyLayout->addLayout(blackPlayerLayout);

    auto redPlayerLayout = new QHBoxLayout();

    m_redReadyInLobbyCheckBox = new QCheckBox(readyInLobbyString.c_str());
    redPlayerLayout->addWidget(m_redReadyInLobbyCheckBox);

    auto redTurtleIconLabel = new QLabel();
    auto redTurtleIcon = QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::Red));
    auto scaledRedTurtleIcon = redTurtleIcon.scaled(ICON_HEIGHT_WIDTH, ICON_HEIGHT_WIDTH,
                                                    Qt::KeepAspectRatio, Qt::SmoothTransformation);
    redTurtleIconLabel->setPixmap(scaledRedTurtleIcon);
    redPlayerLayout->addWidget(redTurtleIconLabel);

    m_redPlayerNameLabel = new QLabel(redPlayerJoined ? m_redPlayerName.c_str() : openString.c_str());
    m_redPlayerNameLabel->setEnabled(redPlayerJoined);
    redPlayerLayout->addWidget(m_redPlayerNameLabel);

    m_redReadyInLobbyCheckBox->setEnabled(false);
    connect(m_redReadyInLobbyCheckBox, &QCheckBox::stateChanged, this,
            &InLobbyFrame::handleRedReadyButtonToggled);

    inLobbyLayout->addLayout(redPlayerLayout);

    auto inLobbyButtonLayout = new QHBoxLayout();

    std::string leaveLobbyInLobbyString = "Leave Lobby";
    auto leaveLobbyInLobbyButton = new QPushButton(leaveLobbyInLobbyString.c_str());
    connect(leaveLobbyInLobbyButton, &QPushButton::released, this,
            &InLobbyFrame::handleLeaveLobbyButton);
    inLobbyButtonLayout->addWidget(leaveLobbyInLobbyButton);

    inLobbyLayout->addLayout(inLobbyButtonLayout);
}

InLobbyFrame::~InLobbyFrame()
{
}

void InLobbyFrame::showEvent(QShowEvent *event)
{
	(void)event; // NO LINT

    m_lobbyNameLabel->setText(Parameters::getLobbyName().c_str());
    std::string lobbyIdWithHash = "#" + Parameters::getLobbyId();
    m_lobbyIdLabel->setText(lobbyIdWithHash.c_str());
}

void InLobbyFrame::setLobbyInfo(const std::string &blackPlayerName,
                                const std::string &redPlayerName,
                                bool blackPlayerReady,
                                bool redPlayerReady)
{
    m_blackPlayerName = blackPlayerName;
    m_redPlayerName = redPlayerName;
    m_blackPlayerReady = blackPlayerReady;
    m_redPlayerReady = redPlayerReady;

    auto playerName = Parameters::getPlayerName();
    if (playerName == blackPlayerName)
    {
        Parameters::setPlayerColor(TurtlePieceColor::Black);
    }
    else if (playerName == redPlayerName)
    {
        Parameters::setPlayerColor(TurtlePieceColor::Red);
    }
    else
    {
        Parameters::setPlayerColor(TurtlePieceColor::None);
    }

    std::string openString = "Open";
    bool blackPlayerJoined = !m_blackPlayerName.empty();
    bool redPlayerJoined = !m_redPlayerName.empty();

    m_blackPlayerNameLabel->setText(blackPlayerJoined ? m_blackPlayerName.c_str() : openString.c_str());
    m_redPlayerNameLabel->setText(redPlayerJoined ? m_redPlayerName.c_str() : openString.c_str());

    auto playerColor = Parameters::getPlayerColor();
    if (playerColor == TurtlePieceColor::Black)
    {
        m_blackReadyInLobbyCheckBox->setEnabled(true);
        if (m_blackPlayerReady)
        {
            m_blackReadyInLobbyCheckBox->setCheckState(Qt::Checked);
        }
        else
        {
            m_blackReadyInLobbyCheckBox->setCheckState(Qt::Unchecked);
        }
    }
    else
    {
        m_blackReadyInLobbyCheckBox->setEnabled(false);
        if (m_blackPlayerReady)
        {
            m_blackReadyInLobbyCheckBox->setCheckState(Qt::Checked);
        }
        else
        {
            m_blackReadyInLobbyCheckBox->setCheckState(Qt::Unchecked);
        }
    }

    if (playerColor == TurtlePieceColor::Red)
    {
        m_redReadyInLobbyCheckBox->setEnabled(true);
        if (m_redPlayerReady)
        {
            m_redReadyInLobbyCheckBox->setCheckState(Qt::Checked);
        }
        else
        {
            m_redReadyInLobbyCheckBox->setCheckState(Qt::Unchecked);
        }
    }
    else
    {
        m_redReadyInLobbyCheckBox->setEnabled(false);
        if (m_redPlayerReady)
        {
            m_redReadyInLobbyCheckBox->setCheckState(Qt::Checked);
        }
        else
        {
            m_redReadyInLobbyCheckBox->setCheckState(Qt::Unchecked);
        }
    }
}

void InLobbyFrame::playerJoinedLobby(const std::string &playerName, TurtlePieceColor playerColor)
{
    if (playerName == Parameters::getPlayerName())
    {
        // Do nothing
    }

    switch (playerColor)
    {
    case TurtlePieceColor::Black:
    {
        m_blackPlayerName = playerName;
        m_blackPlayerNameLabel->setText(m_blackPlayerName.c_str());
        m_blackPlayerNameLabel->setEnabled(true);
    }
    break;
    case TurtlePieceColor::Red:
    {
        m_redPlayerName = playerName;
        m_redPlayerNameLabel->setText(m_redPlayerName.c_str());
        m_redPlayerNameLabel->setEnabled(true);
    }
    break;
    case TurtlePieceColor::None:
    {
        // Do nothing
    }
    break;
    }
}

void InLobbyFrame::playerLeftLobby(const std::string &playerName)
{
    if (playerName == Parameters::getPlayerName())
    {
        // Do nothing
    }

    if (playerName == m_blackPlayerName)
    {
        m_blackPlayerNameLabel->setText("Open");
        m_blackPlayerNameLabel->setEnabled(false);
        m_blackReadyInLobbyCheckBox->setCheckState(Qt::Unchecked);
        m_blackPlayerName.clear();
    }
    else if (playerName == m_redPlayerName)
    {
        m_redPlayerNameLabel->setText("Open");
        m_redPlayerNameLabel->setEnabled(false);
        m_redReadyInLobbyCheckBox->setCheckState(Qt::Unchecked);
        m_redPlayerName.clear();
    }
}

void InLobbyFrame::setPlayerReady(const std::string &playerName, bool ready)
{
    if (playerName == Parameters::getPlayerName())
    {
        // Do nothing
    }
    else
    {
        if (playerName == m_blackPlayerName)
        {
            m_blackReadyInLobbyCheckBox->setCheckState(ready ? Qt::Checked : Qt::Unchecked);
        }
        else if (playerName == m_redPlayerName)
        {
            m_redReadyInLobbyCheckBox->setCheckState(ready ? Qt::Checked : Qt::Unchecked);
        }
    }
}

void InLobbyFrame::handleLeaveLobbyButton()
{
    m_playerWindow->leaveLobby();
    m_playerWindow->moveToMainMenuFrame();
}

void InLobbyFrame::handleBlackReadyButtonToggled(int state)
{
    m_blackPlayerReady = (state == static_cast<int>(Qt::CheckState::Checked));
    if (Parameters::getPlayerColor() == TurtlePieceColor::Black)
    {
        m_playerWindow->setReady((state == static_cast<int>(Qt::CheckState::Checked)));
    }
}

void InLobbyFrame::handleRedReadyButtonToggled(int state)
{
    m_redPlayerReady = (state == static_cast<int>(Qt::CheckState::Checked));
    if (Parameters::getPlayerColor() == TurtlePieceColor::Red)
    {
        m_playerWindow->setReady((state == static_cast<int>(Qt::CheckState::Checked)));
    }
}