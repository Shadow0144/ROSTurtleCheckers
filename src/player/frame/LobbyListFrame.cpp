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
    m_playerDesiredColor = TurtlePieceColor::None;

    auto mainLayout = new QVBoxLayout(this);
    mainLayout->setAlignment(Qt::AlignCenter);

    m_languageSelector = new LanguageSelectorWidget(this);

    m_titleWidget = new TitleWidget();
    mainLayout->addWidget(m_titleWidget);

    auto contentWidget = new QWidget();
    auto contentLayout = new QVBoxLayout();
    contentWidget->setLayout(contentLayout);
    contentLayout->addStretch();
    contentLayout->setAlignment(Qt::AlignHCenter);

    m_lobbyListScrollArea = new QScrollArea();
    m_lobbyListScrollArea->setWidgetResizable(true);
    m_lobbyListScrollArea->setFixedSize(LOBBY_LIST_SCROLL_W, LOBBY_LIST_SCROLL_H);
    m_lobbyListScrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);

    m_lobbyListLayoutWidget = nullptr;
    buildLobbyList();

    contentLayout->addWidget(m_lobbyListScrollArea);

    auto joinLobbyDesiredColorLayout = new QHBoxLayout();
    joinLobbyDesiredColorLayout->setAlignment(Qt::AlignCenter);

    m_blackRadioButton = new QRadioButton();
    m_randomRadioButton = new QRadioButton();
    m_redRadioButton = new QRadioButton();

    m_blackRadioButton->setIcon(QIcon(QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::Black))));
    m_randomRadioButton->setIcon(QIcon(QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::None))));
    m_redRadioButton->setIcon(QIcon(QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::Red))));

    m_randomRadioButton->setChecked(true);

    connect(m_blackRadioButton, &QRadioButton::toggled, this, &LobbyListFrame::onBlackTurtleToggled);
    connect(m_randomRadioButton, &QRadioButton::toggled, this, &LobbyListFrame::onRandomTurtleToggled);
    connect(m_redRadioButton, &QRadioButton::toggled, this, &LobbyListFrame::onRedTurtleToggled);

    joinLobbyDesiredColorLayout->addWidget(m_blackRadioButton);
    joinLobbyDesiredColorLayout->addWidget(m_randomRadioButton);
    joinLobbyDesiredColorLayout->addWidget(m_redRadioButton);

    contentLayout->addLayout(joinLobbyDesiredColorLayout);

    mainLayout->addWidget(contentWidget);

    auto menuButtonWidget = new QWidget();
    menuButtonWidget->setContentsMargins(0, 5, 0, 5);
    auto menuButtonLayout = new QHBoxLayout();
    menuButtonWidget->setLayout(menuButtonLayout);
    menuButtonLayout->setAlignment(Qt::AlignCenter);

    m_refreshJoinLobbyButton = new QPushButton(StringLibrary::getTranslatedString("Refresh"));
    m_refreshJoinLobbyButton->setFixedWidth(MENU_BUTTON_WIDTH);
    connect(m_refreshJoinLobbyButton, &QPushButton::released, this,
            &LobbyListFrame::handleRefreshJoinLobbyButton);
    menuButtonLayout->addWidget(m_refreshJoinLobbyButton);

    m_cancelJoinLobbyButton = new QPushButton(StringLibrary::getTranslatedString("Cancel"));
    m_cancelJoinLobbyButton->setFixedWidth(MENU_BUTTON_WIDTH);
    connect(m_cancelJoinLobbyButton, &QPushButton::released, this,
            &LobbyListFrame::handleCancelJoinLobbyButton);
    menuButtonLayout->addWidget(m_cancelJoinLobbyButton);

    mainLayout->addWidget(menuButtonWidget);

    // Needs to be in front of the title widget
    m_languageSelector->raise();
}

LobbyListFrame::~LobbyListFrame()
{
}

void LobbyListFrame::showEvent(QShowEvent *event)
{
    (void)event; // NO LINT

    if (m_lobbyListLayoutWidget)
    {
        delete m_lobbyListLayoutWidget;
        m_lobbyListLayoutWidget = nullptr;
    }

    m_playerWindow->getLobbyList();
    m_randomRadioButton->setChecked(true);

    m_languageSelector->setCurrentIndex(static_cast<int>(Parameters::getLanguage()));
    reloadStrings();
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
    m_blackPlayerNames = blackPlayerNames;
    m_redPlayerNames = redPlayerNames;

    buildLobbyList();
}

void LobbyListFrame::handleCommitJoinLobbyButton(size_t lobbyIndex)
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

void LobbyListFrame::handleRefreshJoinLobbyButton()
{
    m_playerWindow->getLobbyList();
}

void LobbyListFrame::handleCancelJoinLobbyButton()
{
    m_playerWindow->moveToMainMenuFrame();
}

void LobbyListFrame::onBlackTurtleToggled(bool checked)
{
    if (checked)
    {
        m_playerDesiredColor = TurtlePieceColor::Black;
        Parameters::setPlayerColor(m_playerDesiredColor);
    }
}

void LobbyListFrame::onRandomTurtleToggled(bool checked)
{
    if (checked)
    {
        m_playerDesiredColor = TurtlePieceColor::None;
        Parameters::setPlayerColor(m_playerDesiredColor);
    }
}

void LobbyListFrame::onRedTurtleToggled(bool checked)
{
    if (checked)
    {
        m_playerDesiredColor = TurtlePieceColor::Red;
        Parameters::setPlayerColor(m_playerDesiredColor);
    }
}

void LobbyListFrame::buildLobbyList()
{
    if (m_lobbyListLayoutWidget)
    {
        delete m_lobbyListLayoutWidget;
        m_lobbyListLayoutWidget = nullptr;
    }

    m_lobbyListLayoutWidget = new QWidget();
    m_lobbyListLayoutWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    auto lobbyListLayout = new QVBoxLayout(m_lobbyListLayoutWidget);

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
                                                         m_blackPlayerNames[i],
                                                         m_redPlayerNames[i],
                                                         m_hasPasswords[i],
                                                         [i, this]()
                                                         { this->handleCommitJoinLobbyButton(i); });
        m_lobbyDetailsWidgets.push_back(lobbyDetailsWidget);
        lobbyListLayout->addWidget(lobbyDetailsWidget);
    }

    auto spacer = new QSpacerItem(0, 0, QSizePolicy::Preferred, QSizePolicy::Expanding);
    lobbyListLayout->addItem(spacer);

    // This will delete the previous widget, layout, and children of them
    m_lobbyListScrollArea->setWidget(m_lobbyListLayoutWidget);

    update();
}

void LobbyListFrame::reloadStrings()
{
    m_titleWidget->reloadStrings();

    m_refreshJoinLobbyButton->setText(StringLibrary::getTranslatedString("Refresh"));
    m_cancelJoinLobbyButton->setText(StringLibrary::getTranslatedString("Cancel"));

    for (auto &lobbyDetailsWidget : m_lobbyDetailsWidgets)
    {
        lobbyDetailsWidget->reloadStrings();
    }
}