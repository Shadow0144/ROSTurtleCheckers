#include "player/PlayerIconNameWidget.hpp"

#include <QHBoxLayout>
#include <QLabel>
#include <QString>

#include <string>

#include "shared/CheckersConsts.hpp"
#include "player/ImageLibrary.hpp"
#include "player/StringLibrary.hpp"

PlayerIconNameWidget::PlayerIconNameWidget(TurtlePieceColor playerColor,
                                           const std::string &playerName)
{
    m_playerColor = playerColor;

    auto playerNameIconLayout = new QHBoxLayout(this);

    m_playerIconLabel = new QLabel();
    auto turtleIcon = QPixmap::fromImage(ImageLibrary::getTurtleImage(playerColor));
    auto scaledTurtleIcon = turtleIcon.scaled(ICON_HEIGHT_WIDTH, ICON_HEIGHT_WIDTH,
                                              Qt::KeepAspectRatio, Qt::SmoothTransformation);
    m_playerIconLabel->setPixmap(scaledTurtleIcon);
    playerNameIconLayout->addWidget(m_playerIconLabel);

    m_playerNameLabel = new QLabel();
    m_playerNameLabel->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    m_playerNameLabel->setFixedWidth(LOBBY_LIST_PLAYER_NAME_WIDTH);
    playerNameIconLayout->addWidget(m_playerNameLabel);

    setPlayerName(playerName);
}

void PlayerIconNameWidget::setPlayerName(const std::string &playerName)
{
    m_playerName = playerName;
    m_playerNameLabel->setEnabled(!m_playerName.empty());
    reloadStrings();
}

const std::string &PlayerIconNameWidget::getPlayerName() const
{
    return m_playerName;
}

void PlayerIconNameWidget::clearPlayerName()
{
    m_playerName.clear();
    m_playerNameLabel->setEnabled(false);
    reloadStrings();
}

TurtlePieceColor PlayerIconNameWidget::getPlayerColor() const
{
    return m_playerColor;
}

void PlayerIconNameWidget::reloadStrings()
{
    // This have to be manually handled since "Open" needs to be translated, but the player name must not be
    m_playerNameLabel->setText(m_playerName.empty() ? StringLibrary::getTranslatedString("Open") : m_playerName.c_str());
}