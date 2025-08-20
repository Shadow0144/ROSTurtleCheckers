#include "player/LobbyDetailsWidget.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QVariant>
#include <QPushButton>
#include <QString>
#include <QStyle>

#include <string>
#include <vector>
#include <functional>

#include "shared/CheckersConsts.hpp"
#include "player/ImageLibrary.hpp"
#include "player/StringLibrary.hpp"

LobbyDetailsWidget::LobbyDetailsWidget(QWidget *parent,
                                       const std::string &lobbyName,
                                       const std::string &lobbyId,
                                       const std::string &blackPlayerName,
                                       const std::string &redPlayerName,
                                       bool hasPassword,
                                       const std::function<void()> &onJoinFunction)
    : QWidget(parent)
{
    m_blackPlayerName = blackPlayerName;
    m_redPlayerName = redPlayerName;

    setProperty("details", QVariant(true));
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    // Update the style
    style()->unpolish(this);
    style()->polish(this);
    update();

    auto lobbyLayout = new QHBoxLayout(this);

    auto lobbyNameLayout = new QHBoxLayout();

    auto lobbyNameLabel = new QLabel(lobbyName.c_str());
    lobbyNameLayout->addWidget(lobbyNameLabel);

    std::string lobbyIdWithHash = "#" + lobbyId;
    auto lobbyIdLabel = new QLabel(lobbyIdWithHash.c_str());
    lobbyIdLabel->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    lobbyIdLabel->setFixedWidth(LOBBY_LIST_LOBBY_NAME_WIDTH);
    lobbyNameLayout->addWidget(lobbyIdLabel);

    lobbyLayout->addLayout(lobbyNameLayout);

    auto blackTurtleIconLabel = new QLabel();
    auto blackTurtleIcon = QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::Black));
    auto scaledBlackTurtleIcon = blackTurtleIcon.scaled(ICON_HEIGHT_WIDTH, ICON_HEIGHT_WIDTH,
                                                        Qt::KeepAspectRatio, Qt::SmoothTransformation);
    blackTurtleIconLabel->setPixmap(scaledBlackTurtleIcon);
    lobbyLayout->addWidget(blackTurtleIconLabel);

    bool blackPlayerJoined = !m_blackPlayerName.empty();
    bool redPlayerJoined = !m_redPlayerName.empty();

    m_blackPlayerNameLabel = new QLabel(blackPlayerJoined ? m_blackPlayerName.c_str() : StringLibrary::getTranslatedString("Open"));
    m_blackPlayerNameLabel->setEnabled(blackPlayerJoined);
    m_blackPlayerNameLabel->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    m_blackPlayerNameLabel->setFixedWidth(LOBBY_LIST_PLAYER_NAME_WIDTH);
    lobbyLayout->addWidget(m_blackPlayerNameLabel);

    auto redTurtleIconLabel = new QLabel();
    auto redTurtleIcon = QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::Red));
    auto scaledRedTurtleIcon = redTurtleIcon.scaled(ICON_HEIGHT_WIDTH, ICON_HEIGHT_WIDTH,
                                                    Qt::KeepAspectRatio, Qt::SmoothTransformation);
    redTurtleIconLabel->setPixmap(scaledRedTurtleIcon);
    lobbyLayout->addWidget(redTurtleIconLabel);

    m_redPlayerNameLabel = new QLabel(redPlayerJoined ? m_redPlayerName.c_str() : StringLibrary::getTranslatedString("Open"));
    m_redPlayerNameLabel->setEnabled(redPlayerJoined);
    m_redPlayerNameLabel->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    m_redPlayerNameLabel->setFixedWidth(LOBBY_LIST_PLAYER_NAME_WIDTH);
    lobbyLayout->addWidget(m_redPlayerNameLabel);

    auto lockIconLabel = new QLabel();
    auto lockIcon = QPixmap::fromImage(ImageLibrary::getLockImage());
    auto scaledLockIcon = lockIcon.scaled(ICON_HEIGHT_WIDTH, ICON_HEIGHT_WIDTH,
                                          Qt::KeepAspectRatio, Qt::SmoothTransformation);
    lockIconLabel->setPixmap(scaledLockIcon);
    auto lockIconLabelSizePolicy = lockIconLabel->sizePolicy();
    lockIconLabelSizePolicy.setRetainSizeWhenHidden(true);
    lockIconLabel->setSizePolicy(lockIconLabelSizePolicy);
    lockIconLabel->setVisible(hasPassword);
    lobbyLayout->addWidget(lockIconLabel);

    m_joinLobbyButton = new QPushButton(StringLibrary::getTranslatedString("Join"));
    if (!blackPlayerJoined || !redPlayerJoined)
    {
        m_joinLobbyButton->setEnabled(true);
    }
    else
    {
        // Lobby is full
        m_joinLobbyButton->setEnabled(false);
    }
    connect(m_joinLobbyButton, &QPushButton::released, parent, onJoinFunction);
    lobbyLayout->addWidget(m_joinLobbyButton);
}

void LobbyDetailsWidget::reloadStrings()
{
    m_blackPlayerNameLabel->setText(m_blackPlayerName.empty() ? StringLibrary::getTranslatedString("Open") : m_blackPlayerName.c_str());
    m_redPlayerNameLabel->setText(m_redPlayerName.empty() ? StringLibrary::getTranslatedString("Open") : m_redPlayerName.c_str());

    m_joinLobbyButton->setText(StringLibrary::getTranslatedString("Join"));
}