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

    m_playerNameLabel = new QLabel("");
    auto playerNameFont = m_playerNameLabel->font();
    playerNameFont.setPointSize(PLAYER_NAME_FONT_SIZE);
    m_playerNameLabel->setFont(playerNameFont);
    m_playerNameLabel->setAlignment(Qt::AlignCenter);
    m_playerNameLabel->setContentsMargins(0, 0, 0, 20);
    mainLayout->addWidget(m_playerNameLabel);

    auto createLobbyLayout = new QHBoxLayout();
    std::string createLobbyString = "Create Lobby";
    m_createLobbyButton = new QPushButton(createLobbyString.c_str());
    m_createLobbyButton->setFixedWidth(MENU_BUTTON_WIDTH);
    connect(m_createLobbyButton, &QPushButton::released, this, &MainMenuFrame::handleCreateLobbyButton);
    createLobbyLayout->addWidget(m_createLobbyButton);
    mainLayout->addLayout(createLobbyLayout);

    auto joinLobbyLayout = new QHBoxLayout();
    std::string joinLobbyString = "Join Lobby";
    m_joinLobbyButton = new QPushButton(joinLobbyString.c_str());
    m_joinLobbyButton->setFixedWidth(MENU_BUTTON_WIDTH);
    m_joinLobbyButton->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);
    connect(m_joinLobbyButton, &QPushButton::released, this, &MainMenuFrame::handleJoinLobbyButton);
    joinLobbyLayout->addWidget(m_joinLobbyButton);
    mainLayout->addLayout(joinLobbyLayout);

    auto statisticsLayout = new QHBoxLayout();
    std::string statisticsString = "Statistics";
    auto statisticsButton = new QPushButton(statisticsString.c_str());
    statisticsButton->setFixedWidth(MENU_BUTTON_WIDTH);
    statisticsButton->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);
    connect(statisticsButton, &QPushButton::released, this, &MainMenuFrame::handleStatisticsButton);
    statisticsLayout->addWidget(statisticsButton);
    mainLayout->addLayout(statisticsLayout);

    auto changeAccountPasswordLayout = new QHBoxLayout();
    std::string changeAccountPasswordAccountString = "Change Password";
    auto changeAccountPasswordAccountButton = new QPushButton(changeAccountPasswordAccountString.c_str());
    changeAccountPasswordAccountButton->setFixedWidth(MENU_BUTTON_WIDTH);
    changeAccountPasswordAccountButton->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);
    connect(changeAccountPasswordAccountButton, &QPushButton::released, this, &MainMenuFrame::handleChangeAccountPasswordButton);
    changeAccountPasswordLayout->addWidget(changeAccountPasswordAccountButton);
    mainLayout->addLayout(changeAccountPasswordLayout);

    auto logOutLayout = new QHBoxLayout();
    std::string logOutAccountString = "Log Out";
    auto logOutAccountButton = new QPushButton(logOutAccountString.c_str());
    logOutAccountButton->setFixedWidth(MENU_BUTTON_WIDTH);
    logOutAccountButton->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);
    connect(logOutAccountButton, &QPushButton::released, this, &MainMenuFrame::handleLogOutAccountButton);
    logOutLayout->addWidget(logOutAccountButton);
    mainLayout->addLayout(logOutLayout);

    auto quitLayout = new QHBoxLayout();
    std::string quitString = "Quit";
    auto quitButton = new QPushButton(quitString.c_str());
    quitButton->setFixedWidth(MENU_BUTTON_WIDTH);
    quitButton->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);
    connect(quitButton, &QPushButton::released, this, &MainMenuFrame::handleQuitButton);
    quitLayout->addWidget(quitButton);
    mainLayout->addLayout(quitLayout);
}

MainMenuFrame::~MainMenuFrame()
{
}

void MainMenuFrame::showEvent(QShowEvent *event)
{
    (void)event; // NO LINT

    m_playerNameLabel->setText(Parameters::getPlayerName().c_str());
}

void MainMenuFrame::handleCreateLobbyButton()
{
    m_playerWindow->moveToCreateLobbyFrame();
}

void MainMenuFrame::handleJoinLobbyButton()
{
    m_playerWindow->moveToLobbyListFrame();
}

void MainMenuFrame::handleStatisticsButton()
{
    m_playerWindow->moveToStatisticsFrame();
}

void MainMenuFrame::handleChangeAccountPasswordButton()
{
    m_playerWindow->moveToChangeAccountPasswordFrame();
}

void MainMenuFrame::handleLogOutAccountButton()
{
    m_playerWindow->logOutAccount();
}

void MainMenuFrame::handleQuitButton()
{
    m_playerWindow->close();
}