#include "player/frame/MainMenuFrame.hpp"

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
#include "player/CheckersPlayerWindow.hpp"
#include "player/ImageLibrary.hpp"
#include "player/Parameters.hpp"

MainMenuFrame::MainMenuFrame(
    CheckersPlayerWindow *parentWindow)
    : QFrame(parentWindow, Qt::WindowFlags())
{
    m_playerWindow = parentWindow;

    auto mainLayout = new QVBoxLayout(this);
    mainLayout->setAlignment(Qt::AlignCenter);

    auto titleLabel = new QLabel("Turtle Checkers");
    auto titleFont = titleLabel->font();
    titleFont.setPointSize(TITLE_FONT_SIZE);
    titleLabel->setFont(titleFont);
    mainLayout->addWidget(titleLabel);

    auto playerNameLayout = new QHBoxLayout();

    auto playerNameLabel = new QLabel("Player name: ");
    playerNameLayout->addWidget(playerNameLabel);

    m_playerNameLabel = new QLabel("");
    playerNameLayout->addWidget(m_playerNameLabel);

    std::string logOutAccountString = "Log Out";
    auto logOutAccountButton = new QPushButton(logOutAccountString.c_str());
    connect(logOutAccountButton, &QPushButton::released, this, &MainMenuFrame::handleLogOutAccountButton);
    playerNameLayout->addWidget(logOutAccountButton);

    mainLayout->addLayout(playerNameLayout);

    auto buttonLayout = new QHBoxLayout();

    std::string createLobbyString = "Create Lobby";
    m_createLobbyButton = new QPushButton(createLobbyString.c_str());
    connect(m_createLobbyButton, &QPushButton::released, this, &MainMenuFrame::handleCreateLobbyButton);
    buttonLayout->addWidget(m_createLobbyButton);

    std::string joinLobbyString = "Join Lobby";
    m_joinLobbyButton = new QPushButton(joinLobbyString.c_str());
    connect(m_joinLobbyButton, &QPushButton::released, this, &MainMenuFrame::handleJoinLobbyButton);
    buttonLayout->addWidget(m_joinLobbyButton);

    std::string quitString = "Quit";
    auto quitButton = new QPushButton(quitString.c_str());
    connect(quitButton, &QPushButton::released, this, &MainMenuFrame::handleQuitButton);
    buttonLayout->addWidget(quitButton);

    mainLayout->addLayout(buttonLayout);
}

MainMenuFrame::~MainMenuFrame()
{
}

void MainMenuFrame::showEvent(QShowEvent *event)
{
    (void)event; // NO LINT

    m_playerNameLabel->setText(Parameters::getPlayerName().c_str());
}

void MainMenuFrame::handleLogOutAccountButton()
{
    m_playerWindow->logOutAccount();
}

void MainMenuFrame::handleCreateLobbyButton()
{
    m_playerWindow->moveToCreateLobbyFrame();
}

void MainMenuFrame::handleJoinLobbyButton()
{
    m_playerWindow->moveToLobbyListFrame();
}

void MainMenuFrame::handleQuitButton()
{
    m_playerWindow->close();
}