#include "player/frame/MainMenuFrame.hpp"

#include <QFrame>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QPushButton>

#include <cstdlib>
#include <memory>
#include <string>
#include <iostream>

#include "shared/CheckersConsts.hpp"
#include "player/TitleWidget.hpp"
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

    auto titleWidget = new TitleWidget();
    mainLayout->addWidget(titleWidget);

    auto playerNameLayout = new QHBoxLayout();

    m_playerNameLabel = new QLabel("");
    auto playerNameFont = m_playerNameLabel->font();
    playerNameFont.setPointSize(PLAYER_NAME_FONT_SIZE);
    m_playerNameLabel->setFont(playerNameFont);
    playerNameLayout->addWidget(m_playerNameLabel);

    std::string statisticsString = "Statistics";
    auto statisticsButton = new QPushButton(statisticsString.c_str());
    statisticsButton->setFixedWidth(MENU_BUTTON_WIDTH);
    connect(statisticsButton, &QPushButton::released, this, &MainMenuFrame::handleStatisticsButton);
    playerNameLayout->addWidget(statisticsButton);

    std::string logOutAccountString = "Log Out";
    auto logOutAccountButton = new QPushButton(logOutAccountString.c_str());
    logOutAccountButton->setFixedWidth(MENU_BUTTON_WIDTH);
    connect(logOutAccountButton, &QPushButton::released, this, &MainMenuFrame::handleLogOutAccountButton);
    playerNameLayout->addWidget(logOutAccountButton);

    mainLayout->addLayout(playerNameLayout);

    auto buttonLayout = new QHBoxLayout();

    std::string createLobbyString = "Create Lobby";
    m_createLobbyButton = new QPushButton(createLobbyString.c_str());
    m_createLobbyButton->setFixedWidth(MENU_BUTTON_WIDTH);
    connect(m_createLobbyButton, &QPushButton::released, this, &MainMenuFrame::handleCreateLobbyButton);
    buttonLayout->addWidget(m_createLobbyButton);

    std::string joinLobbyString = "Join Lobby";
    m_joinLobbyButton = new QPushButton(joinLobbyString.c_str());
    m_joinLobbyButton->setFixedWidth(MENU_BUTTON_WIDTH);
    connect(m_joinLobbyButton, &QPushButton::released, this, &MainMenuFrame::handleJoinLobbyButton);
    buttonLayout->addWidget(m_joinLobbyButton);

    std::string quitString = "Quit";
    auto quitButton = new QPushButton(quitString.c_str());
    quitButton->setFixedWidth(MENU_BUTTON_WIDTH);
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

void MainMenuFrame::handleStatisticsButton()
{
    m_playerWindow->moveToStatisticsFrame();
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