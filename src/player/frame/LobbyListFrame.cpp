#include "player/frame/LobbyListFrame.hpp"

#include <QWidget>
#include <QFrame>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QScrollArea>
#include <QLabel>
#include <QPushButton>
#include <QRadioButton>
#include <QIcon>
#include <QPixmap>
#include <QSpacerItem>
#include <QSizePolicy>
#include <QProgressBar>

#include <cstdlib>
#include <memory>
#include <string>
#include <vector>
#include <iostream>

#include "player/CheckersPlayerWindow.hpp"
#include "shared/CheckersConsts.hpp"
#include "player/Parameters.hpp"
#include "player/ImageLibrary.hpp"
#include "player/StringLibrary.hpp"
#include "shared/TurtleLogger.hpp"
#include "player/TitleWidget.hpp"
#include "player/LanguageSelectorWidget.hpp"
#include "player/LobbyDetailsWidget.hpp"

LobbyListFrame::LobbyListFrame(
    CheckersPlayerWindow *parentWindow)
    : QFrame(parentWindow, Qt::WindowFlags())
{
    m_playerWindow = parentWindow;

    auto lobbyListLayout = new QVBoxLayout(this);
    lobbyListLayout->setAlignment(Qt::AlignCenter);

    m_languageSelector = new LanguageSelectorWidget(this);

    m_titleWidget = new TitleWidget();
    lobbyListLayout->addWidget(m_titleWidget);

    m_lobbyListScrollArea = new QScrollArea();
    m_lobbyListScrollArea->setWidgetResizable(true);
    m_lobbyListScrollArea->setFixedSize(LOBBY_LIST_SCROLL_W, LOBBY_LIST_SCROLL_H);
    m_lobbyListScrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);

    displayProgressBar();

    lobbyListLayout->addWidget(m_lobbyListScrollArea);

    auto lobbyListButtonLayout = new QHBoxLayout();
    lobbyListButtonLayout->setAlignment(Qt::AlignCenter);

    m_refreshButton = new QPushButton(StringLibrary::getTranslatedString("Refresh"));
    m_refreshButton->setFixedWidth(MENU_BUTTON_WIDTH);
    connect(m_refreshButton, &QPushButton::released, this,
            &LobbyListFrame::handleRefreshButton);
    lobbyListButtonLayout->addWidget(m_refreshButton);

    m_cancelButton = new QPushButton(StringLibrary::getTranslatedString("Cancel"));
    m_cancelButton->setFixedWidth(MENU_BUTTON_WIDTH);
    connect(m_cancelButton, &QPushButton::released, this,
            &LobbyListFrame::handleCancelButton);
    lobbyListButtonLayout->addWidget(m_cancelButton);

    lobbyListLayout->addLayout(lobbyListButtonLayout);

    // Needs to be in front of the title widget
    m_languageSelector->raise();
}

LobbyListFrame::~LobbyListFrame()
{
}

void LobbyListFrame::showEvent(QShowEvent *event)
{
    (void)event; // NO LINT

    displayProgressBar();
    m_playerWindow->getLobbyList();

    m_languageSelector->setCurrentIndex(static_cast<int>(Parameters::getLanguage()));
    reloadStrings();
}

void LobbyListFrame::displayProgressBar()
{
    auto lobbyListProgressBarWidget = new QWidget();
    auto lobbyListProgressBarLayout = new QVBoxLayout();
    lobbyListProgressBarWidget->setLayout(lobbyListProgressBarLayout);
    lobbyListProgressBarWidget->setContentsMargins(50, 50, 50, 50);
    lobbyListProgressBarLayout->setAlignment(Qt::AlignCenter);

    auto lobbyListProgressBar = new QProgressBar();
    lobbyListProgressBar->setRange(0, 0);
    lobbyListProgressBarLayout->addWidget(lobbyListProgressBar);

    // This will delete the previous widget, layout, and children of them
    m_lobbyDetailsWidgets.clear(); // Clear these since they will all be deleted
    m_lobbyListScrollArea->setWidget(lobbyListProgressBarWidget);
}

void LobbyListFrame::displayLobbyList(const std::vector<std::string> &lobbyNames,
                                      const std::vector<std::string> &lobbyIds,
                                      const std::vector<bool> &hasPasswords,
                                      const std::vector<std::string> &blackPlayerNames,
                                      const std::vector<std::string> &redPlayerNames)
{
    m_lobbyNames = lobbyNames;
    m_lobbyIds = lobbyIds;
    m_hasPasswords = hasPasswords;

    auto lobbyListLayoutWidget = new QWidget();
    lobbyListLayoutWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    auto lobbyListLayout = new QVBoxLayout(lobbyListLayoutWidget);

    const auto numLobbies = m_lobbyNames.size();
    if (m_lobbyIds.size() != numLobbies)
    {
        TurtleLogger::logError("Lobby name vector does not match size of lobby id vector");
    }
    m_lobbyDetailsWidgets.clear();
    for (size_t i = 0u; i < numLobbies; i++)
    {
        // Add a lobby details widget
        auto lobbyDetailsWidget = new LobbyDetailsWidget(this,
                                                         m_lobbyNames[i],
                                                         m_lobbyIds[i],
                                                         blackPlayerNames[i],
                                                         redPlayerNames[i],
                                                         m_hasPasswords[i],
                                                         [i, this]()
                                                         { this->handleJoinLobbyButton(i); });
        m_lobbyDetailsWidgets.push_back(lobbyDetailsWidget);
        lobbyListLayout->addWidget(lobbyDetailsWidget);
    }

    auto spacer = new QSpacerItem(0, 0, QSizePolicy::Preferred, QSizePolicy::Expanding);
    lobbyListLayout->addItem(spacer);

    // This will delete the previous widget, layout, and children of them
    m_lobbyListScrollArea->setWidget(lobbyListLayoutWidget);

    update();
}

void LobbyListFrame::handleRefreshButton()
{
    displayProgressBar();
    m_playerWindow->getLobbyList();
}

void LobbyListFrame::handleCancelButton()
{
    m_playerWindow->moveToMainMenuFrame();
}

void LobbyListFrame::handleJoinLobbyButton(size_t lobbyIndex)
{
    Parameters::setLobbyName(m_lobbyNames[lobbyIndex]);
    Parameters::setLobbyId(m_lobbyIds[lobbyIndex]);
    if (m_hasPasswords[lobbyIndex])
    {
        m_playerWindow->moveToLobbyPasswordFrame();
    }
    else
    {
        m_playerWindow->joinLobby("");
    }
}

void LobbyListFrame::reloadStrings()
{
    m_titleWidget->reloadStrings();

    m_refreshButton->setText(StringLibrary::getTranslatedString("Refresh"));
    m_cancelButton->setText(StringLibrary::getTranslatedString("Cancel"));

    for (auto &lobbyDetailsWidget : m_lobbyDetailsWidgets)
    {
        lobbyDetailsWidget->reloadStrings();
    }
}