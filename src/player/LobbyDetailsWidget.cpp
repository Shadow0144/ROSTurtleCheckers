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
#include "player/PlayerIconNameWidget.hpp"
#include "player/TranslatedQPushButton.hpp"

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

    m_blackPlayerIconNameWidget = new PlayerIconNameWidget(TurtlePieceColor::Black, m_blackPlayerName);
    lobbyLayout->addWidget(m_blackPlayerIconNameWidget);

    m_redPlayerIconNameWidget = new PlayerIconNameWidget(TurtlePieceColor::Red, m_redPlayerName);
    lobbyLayout->addWidget(m_redPlayerIconNameWidget);

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

    m_joinLobbyButton = new TranslatedQPushButton("Join");
    m_joinLobbyButton->setEnabled(m_blackPlayerName.empty() || m_redPlayerName.empty());
    connect(m_joinLobbyButton, &QPushButton::released, parent, onJoinFunction);
    lobbyLayout->addWidget(m_joinLobbyButton);
}

void LobbyDetailsWidget::reloadStrings()
{
    m_blackPlayerIconNameWidget->reloadStrings();
    m_redPlayerIconNameWidget->reloadStrings();

    m_joinLobbyButton->reloadStrings();
}