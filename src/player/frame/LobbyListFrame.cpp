#include "player/frame/LobbyListFrame.hpp"

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

LobbyListFrame::LobbyListFrame(
    CheckersPlayerWindow *parentWindow)
    : QFrame(parentWindow, Qt::WindowFlags())
{
    m_playerWindow = parentWindow;
    m_playerDesiredColor = TurtlePieceColor::None;

    auto joinLobbyLayout = new QVBoxLayout(this);
    joinLobbyLayout->setAlignment(Qt::AlignCenter);

    auto joinLobbyTitleLabel = new QLabel("Turtle Checkers");
    auto titleFont = joinLobbyTitleLabel->font();
    titleFont.setPointSize(TITLE_FONT_SIZE);
    joinLobbyTitleLabel->setFont(titleFont);
    joinLobbyLayout->addWidget(joinLobbyTitleLabel);

    auto lobbiesLabel = new QLabel("Lobbies:");
    joinLobbyLayout->addWidget(lobbiesLabel);

    m_lobbyListScrollArea = new QScrollArea();
    m_lobbyListScrollArea->setFixedSize(LOBBY_SCROLL_W, LOBBY_SCROLL_H);
    m_lobbyListScrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
    m_lobbyListScrollArea->setObjectName("LobbyListScrollArea");

    m_lobbyListLayoutWidget = nullptr;
    buildLobbyList();

    joinLobbyLayout->addWidget(m_lobbyListScrollArea);

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

    joinLobbyLayout->addLayout(joinLobbyDesiredColorLayout);

    auto joinLobbyButtonLayout = new QHBoxLayout();

    std::string refreshjoinLobbyString = "Refresh";
    auto refreshJoinLobbyButton = new QPushButton(refreshjoinLobbyString.c_str());
    connect(refreshJoinLobbyButton, &QPushButton::released, this,
            &LobbyListFrame::handleRefreshJoinLobbyButton);
    joinLobbyButtonLayout->addWidget(refreshJoinLobbyButton);

    std::string canceljoinLobbyString = "Cancel";
    auto cancelJoinLobbyButton = new QPushButton(canceljoinLobbyString.c_str());
    connect(cancelJoinLobbyButton, &QPushButton::released, this,
            &LobbyListFrame::handleCancelJoinLobbyButton);
    joinLobbyButtonLayout->addWidget(cancelJoinLobbyButton);

    joinLobbyLayout->addLayout(joinLobbyButtonLayout);
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
        std::cerr << "Lobby name vector does not match size of lobby id vector" << std::endl;
    }
    for (size_t i = 0u; i < numLobbies; i++)
    {
        auto lobbyLayoutWidget = new QWidget();
        lobbyLayoutWidget->setProperty("lobby", true);
        lobbyLayoutWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        // Update the style
        lobbyLayoutWidget->style()->unpolish(lobbyLayoutWidget);
        lobbyLayoutWidget->style()->polish(lobbyLayoutWidget);
        lobbyLayoutWidget->update();

        auto lobbyLayout = new QHBoxLayout(lobbyLayoutWidget);

        auto lobbyNameLayout = new QHBoxLayout();

        auto lobbyNameLabel = new QLabel(m_lobbyNames[i].c_str());
        lobbyNameLayout->addWidget(lobbyNameLabel);

        std::string lobbyIdWithHash = "#" + m_lobbyIds[i];
        auto lobbyIdLabel = new QLabel(lobbyIdWithHash.c_str());
        lobbyNameLayout->addWidget(lobbyIdLabel);

        lobbyLayout->addLayout(lobbyNameLayout);

        auto blackTurtleIconLabel = new QLabel();
        auto blackTurtleIcon = QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::Black));
        auto scaledBlackTurtleIcon = blackTurtleIcon.scaled(ICON_HEIGHT_WIDTH, ICON_HEIGHT_WIDTH,
                                                            Qt::KeepAspectRatio, Qt::SmoothTransformation);
        blackTurtleIconLabel->setPixmap(scaledBlackTurtleIcon);
        lobbyLayout->addWidget(blackTurtleIconLabel);

        std::string openString = "Open";
        bool blackPlayerJoined = !m_blackPlayerNames[i].empty();
        bool redPlayerJoined = !m_redPlayerNames[i].empty();

        auto blackPlayerNameLabel = new QLabel(blackPlayerJoined ? m_blackPlayerNames[i].c_str() : openString.c_str());
        blackPlayerNameLabel->setEnabled(blackPlayerJoined);
        lobbyLayout->addWidget(blackPlayerNameLabel);

        auto redTurtleIconLabel = new QLabel();
        auto redTurtleIcon = QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::Red));
        auto scaledRedTurtleIcon = redTurtleIcon.scaled(ICON_HEIGHT_WIDTH, ICON_HEIGHT_WIDTH,
                                                        Qt::KeepAspectRatio, Qt::SmoothTransformation);
        redTurtleIconLabel->setPixmap(scaledRedTurtleIcon);
        lobbyLayout->addWidget(redTurtleIconLabel);

        auto redPlayerNameLabel = new QLabel(redPlayerJoined ? m_redPlayerNames[i].c_str() : openString.c_str());
        redPlayerNameLabel->setEnabled(redPlayerJoined);
        lobbyLayout->addWidget(redPlayerNameLabel);

        auto lockIconLabel = new QLabel();
        auto lockIcon = QPixmap::fromImage(ImageLibrary::getLockImage());
        auto scaledLockIcon = lockIcon.scaled(ICON_HEIGHT_WIDTH, ICON_HEIGHT_WIDTH,
                                              Qt::KeepAspectRatio, Qt::SmoothTransformation);
        lockIconLabel->setPixmap(scaledLockIcon);
        auto lockIconLabelSizePolicy = lockIconLabel->sizePolicy();
        lockIconLabelSizePolicy.setRetainSizeWhenHidden(true);
        lockIconLabel->setSizePolicy(lockIconLabelSizePolicy);
        lockIconLabel->setVisible(m_hasPasswords[i]);
        lobbyLayout->addWidget(lockIconLabel);

        std::string joinLobbyString = "Join";
        auto joinLobbyButton = new QPushButton(joinLobbyString.c_str());
        if (!blackPlayerJoined || !redPlayerJoined)
        {
            joinLobbyButton->setEnabled(true);
        }
        else
        {
            // Lobby is full
            joinLobbyButton->setEnabled(false);
        }
        connect(joinLobbyButton, &QPushButton::released, this,
                [i, this]()
                { this->handleCommitJoinLobbyButton(i); });
        lobbyLayout->addWidget(joinLobbyButton);

        lobbyListLayout->addWidget(lobbyLayoutWidget);
    }

    auto spacer = new QSpacerItem(0, 0, QSizePolicy::Preferred, QSizePolicy::Expanding);
    lobbyListLayout->addItem(spacer);

    m_lobbyListScrollArea->setWidget(m_lobbyListLayoutWidget);

    update();
}