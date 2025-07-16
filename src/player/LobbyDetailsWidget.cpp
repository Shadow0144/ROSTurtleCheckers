#include "player/LobbyDetailsWidget.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QVariant>
#include <QPushButton>
#include <QString>
#include <QStyle>

#include "shared/CheckersConsts.hpp"
#include "player/ImageLibrary.hpp"

LobbyDetailsWidget::LobbyDetailsWidget(QWidget *parent,
                                       const std::string &lobbyName,
                                       const std::string &lobbyId,
                                       const std::string &blackPlayerName,
                                       const std::string &redPlayerName,
                                       bool hasPassword,
                                       const std::function<void()> &onJoinFunction)
{
    setProperty("lobby", QVariant(true));
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

    std::string openString = "Open";
    bool blackPlayerJoined = !blackPlayerName.empty();
    bool redPlayerJoined = !redPlayerName.empty();

    auto blackPlayerNameLabel = new QLabel(blackPlayerJoined ? blackPlayerName.c_str() : openString.c_str());
    blackPlayerNameLabel->setEnabled(blackPlayerJoined);
    blackPlayerNameLabel->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    blackPlayerNameLabel->setFixedWidth(LOBBY_LIST_PLAYER_NAME_WIDTH);
    lobbyLayout->addWidget(blackPlayerNameLabel);

    auto redTurtleIconLabel = new QLabel();
    auto redTurtleIcon = QPixmap::fromImage(ImageLibrary::getTurtleImage(TurtlePieceColor::Red));
    auto scaledRedTurtleIcon = redTurtleIcon.scaled(ICON_HEIGHT_WIDTH, ICON_HEIGHT_WIDTH,
                                                    Qt::KeepAspectRatio, Qt::SmoothTransformation);
    redTurtleIconLabel->setPixmap(scaledRedTurtleIcon);
    lobbyLayout->addWidget(redTurtleIconLabel);

    auto redPlayerNameLabel = new QLabel(redPlayerJoined ? redPlayerName.c_str() : openString.c_str());
    redPlayerNameLabel->setEnabled(redPlayerJoined);
    redPlayerNameLabel->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    redPlayerNameLabel->setFixedWidth(LOBBY_LIST_PLAYER_NAME_WIDTH);
    lobbyLayout->addWidget(redPlayerNameLabel);

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
    connect(joinLobbyButton, &QPushButton::released, parent, onJoinFunction);
    lobbyLayout->addWidget(joinLobbyButton);
}