#include "player/frame/MainMenuFrame.hpp"

#include <QFrame>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QPushButton>

#include <cstdlib>
#include <memory>
#include <string>

#include "player/CheckersPlayerWindow.hpp"
#include "shared/CheckersConsts.hpp"
#include "player/Parameters.hpp"
#include "player/TitleWidget.hpp"
#include "player/LanguageSelectorWidget.hpp"
#include "player/TranslatedQPushButton.hpp"

MainMenuFrame::MainMenuFrame(
    CheckersPlayerWindow *parentWindow)
    : QFrame(parentWindow, Qt::WindowFlags())
{
    m_playerWindow = parentWindow;

    auto mainLayout = new QVBoxLayout(this);
    mainLayout->setAlignment(Qt::AlignCenter);

    m_languageSelector = new LanguageSelectorWidget(this);

    m_titleWidget = new TitleWidget();
    mainLayout->addWidget(m_titleWidget);

    m_playerNameLabel = new QLabel("");
    auto playerNameFont = m_playerNameLabel->font();
    playerNameFont.setPointSize(PLAYER_NAME_FONT_SIZE);
    m_playerNameLabel->setFont(playerNameFont);
    m_playerNameLabel->setAlignment(Qt::AlignCenter);
    m_playerNameLabel->setContentsMargins(0, 0, 0, 20);
    mainLayout->addWidget(m_playerNameLabel);

    auto createLobbyLayout = new QHBoxLayout();
    m_createLobbyButton = new TranslatedQPushButton("Create Lobby");
    m_createLobbyButton->setFixedWidth(MENU_BUTTON_WIDTH);
    connect(m_createLobbyButton, &QPushButton::released, this, &MainMenuFrame::handleCreateLobbyButton);
    createLobbyLayout->addWidget(m_createLobbyButton);
    mainLayout->addLayout(createLobbyLayout);

    auto joinLobbyLayout = new QHBoxLayout();
    m_joinLobbyButton = new TranslatedQPushButton("Join Lobby");
    m_joinLobbyButton->setFixedWidth(MENU_BUTTON_WIDTH);
    m_joinLobbyButton->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);
    connect(m_joinLobbyButton, &QPushButton::released, this, &MainMenuFrame::handleJoinLobbyButton);
    joinLobbyLayout->addWidget(m_joinLobbyButton);
    mainLayout->addLayout(joinLobbyLayout);

    auto statisticsLayout = new QHBoxLayout();
    m_statisticsButton = new TranslatedQPushButton("Statistics");
    m_statisticsButton->setFixedWidth(MENU_BUTTON_WIDTH);
    m_statisticsButton->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);
    connect(m_statisticsButton, &QPushButton::released, this, &MainMenuFrame::handleStatisticsButton);
    statisticsLayout->addWidget(m_statisticsButton);
    mainLayout->addLayout(statisticsLayout);

    auto changeAccountPasswordLayout = new QHBoxLayout();
    m_changeAccountPasswordAccountButton = new TranslatedQPushButton("Change Password");
    m_changeAccountPasswordAccountButton->setFixedWidth(MENU_BUTTON_WIDTH);
    m_changeAccountPasswordAccountButton->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);
    connect(m_changeAccountPasswordAccountButton, &QPushButton::released, this, &MainMenuFrame::handleChangeAccountPasswordButton);
    changeAccountPasswordLayout->addWidget(m_changeAccountPasswordAccountButton);
    mainLayout->addLayout(changeAccountPasswordLayout);

    auto logOutLayout = new QHBoxLayout();
    m_logOutAccountButton = new TranslatedQPushButton("Log Out");
    m_logOutAccountButton->setFixedWidth(MENU_BUTTON_WIDTH);
    m_logOutAccountButton->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);
    connect(m_logOutAccountButton, &QPushButton::released, this, &MainMenuFrame::handleLogOutAccountButton);
    logOutLayout->addWidget(m_logOutAccountButton);
    mainLayout->addLayout(logOutLayout);

    auto quitLayout = new QHBoxLayout();
    m_quitButton = new TranslatedQPushButton("Quit");
    m_quitButton->setFixedWidth(MENU_BUTTON_WIDTH);
    m_quitButton->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);
    connect(m_quitButton, &QPushButton::released, this, &MainMenuFrame::handleQuitButton);
    quitLayout->addWidget(m_quitButton);
    mainLayout->addLayout(quitLayout);
}

MainMenuFrame::~MainMenuFrame()
{
}

void MainMenuFrame::showEvent(QShowEvent *event)
{
    (void)event; // NO LINT

    m_playerNameLabel->setText(Parameters::getPlayerName().c_str());

    m_languageSelector->setCurrentIndex(static_cast<int>(Parameters::getLanguage()));
    reloadStrings();
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

void MainMenuFrame::reloadStrings()
{
    m_titleWidget->reloadStrings();

    m_createLobbyButton->reloadStrings();
    m_joinLobbyButton->reloadStrings();
    m_statisticsButton->reloadStrings();
    m_changeAccountPasswordAccountButton->reloadStrings();
    m_logOutAccountButton->reloadStrings();
    m_quitButton->reloadStrings();
}