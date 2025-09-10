#include "player/MatchDetailsWidget.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QVariant>
#include <QString>
#include <QStyle>

#include <string>
#include <vector>
#include <functional>

#include "shared/CheckersConsts.hpp"
#include "player/ImageLibrary.hpp"
#include "player/PlayerIconNameWidget.hpp"
#include "player/TranslatedQLabel.hpp"

MatchDetailsWidget::MatchDetailsWidget(QWidget *parent,
                                       const std::string &playerName,
                                       const std::string &lobbyNameId,
                                       const std::string &blackPlayerName,
                                       const std::string &redPlayerName,
                                       Winner winner)
    : QWidget(parent)
{
    m_winner = winner;

    setProperty("details", QVariant(true));
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    // Update the style
    style()->unpolish(this);
    style()->polish(this);
    update();

    auto matchLayout = new QHBoxLayout(this);

    auto matchNameLayout = new QHBoxLayout();
    auto matchIdLabel = new QLabel(lobbyNameId.c_str());
    matchIdLabel->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    matchIdLabel->setFixedWidth(MATCH_LIST_LOBBY_NAME_WIDTH);
    matchNameLayout->addWidget(matchIdLabel);
    matchLayout->addLayout(matchNameLayout);

    auto winIcon = QPixmap::fromImage(ImageLibrary::getWinnerImage());
    auto scaledWinIcon = winIcon.scaled(ICON_HEIGHT_WIDTH, ICON_HEIGHT_WIDTH,
                                        Qt::KeepAspectRatio, Qt::SmoothTransformation);

    auto loseIcon = QPixmap::fromImage(ImageLibrary::getLoserImage());
    auto scaledLoseIcon = loseIcon.scaled(ICON_HEIGHT_WIDTH, ICON_HEIGHT_WIDTH,
                                          Qt::KeepAspectRatio, Qt::SmoothTransformation);

    auto drawIcon = QPixmap::fromImage(ImageLibrary::getDrawImage());
    auto scaledDrawIcon = drawIcon.scaled(ICON_HEIGHT_WIDTH, ICON_HEIGHT_WIDTH,
                                          Qt::KeepAspectRatio, Qt::SmoothTransformation);

    auto winnerLayout = new QHBoxLayout();
    auto winnerIconLabel = new QLabel();
    m_winnerLabel = new TranslatedQLabel();
    m_winnerLabel->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    m_winnerLabel->setFixedWidth(MATCH_LIST_PLAYER_NAME_WIDTH);

    switch (m_winner)
    {
    case Winner::None:
    {
        m_winningPlayerName = "";
        m_winnerLabel->setText("No winner");
    }
    break;
    case Winner::Black:
    {
        m_winningPlayerName = blackPlayerName;
        m_winnerLabel->setText("%s won", {m_winningPlayerName});
        if (playerName == blackPlayerName)
        {
            winnerIconLabel->setPixmap(scaledWinIcon);
        }
        else if (playerName == redPlayerName)
        {
            winnerIconLabel->setPixmap(scaledLoseIcon);
        }
    }
    break;
    case Winner::Red:
    {
        m_winningPlayerName = redPlayerName;
        m_winnerLabel->setText("%s won", {m_winningPlayerName});
        if (playerName == blackPlayerName)
        {
            winnerIconLabel->setPixmap(scaledLoseIcon);
        }
        else if (playerName == redPlayerName)
        {
            winnerIconLabel->setPixmap(scaledWinIcon);
        }
    }
    break;
    case Winner::Draw:
    {
        m_winningPlayerName = "";
        m_winnerLabel->setText("Draw");
        if (playerName == blackPlayerName)
        {
            winnerIconLabel->setPixmap(scaledDrawIcon);
        }
        else if (playerName == redPlayerName)
        {
            winnerIconLabel->setPixmap(scaledDrawIcon);
        }
    }
    break;
    default:
    {
        m_winningPlayerName = "";
        m_winnerLabel->setText("No winner");
    }
    break;
    }

    winnerLayout->addWidget(winnerIconLabel);
    winnerLayout->addWidget(m_winnerLabel);
    matchLayout->addLayout(winnerLayout);

    auto blackPlayerIconNameWidget = new PlayerIconNameWidget(TurtlePieceColor::Black, blackPlayerName);
    matchLayout->addWidget(blackPlayerIconNameWidget);

    auto redPlayerIconNameWidget = new PlayerIconNameWidget(TurtlePieceColor::Red, redPlayerName);
    matchLayout->addWidget(redPlayerIconNameWidget);
}

void MatchDetailsWidget::reloadStrings()
{
    m_winnerLabel->reloadStrings();
}