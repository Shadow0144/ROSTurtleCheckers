#include "player/frame/StatisticsFrame.hpp"

#include <QFrame>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QScrollArea>
#include <QLabel>
#include <QLineEdit>
#include <QString>
#include <QRegularExpression>
#include <QRegularExpressionValidator>
#include <QPushButton>
#include <QCheckBox>
#include <QComboBox>
#include <QPixmap>
#include <QSpacerItem>

#include <cstdlib>
#include <ctime>
#include <memory>
#include <string>
#include <iostream>

#include "shared/CheckersConsts.hpp"
#include "player/Parameters.hpp"
#include "player/TitleWidget.hpp"
#include "player/MatchDetailsWidget.hpp"
#include "player/CheckersPlayerWindow.hpp"
#include "player/ImageLibrary.hpp"

StatisticsFrame::StatisticsFrame(
    CheckersPlayerWindow *parentWindow)
    : QFrame(parentWindow, Qt::WindowFlags())
{
    m_playerWindow = parentWindow;

    auto statisticsLayout = new QVBoxLayout(this);
    statisticsLayout->setAlignment(Qt::AlignCenter);

    auto statisticsContentLayout = new QGridLayout();
    statisticsContentLayout->setAlignment(Qt::AlignCenter);
    statisticsLayout->addLayout(statisticsContentLayout);

    auto titleWidget = new TitleWidget();
    statisticsContentLayout->addWidget(titleWidget, 0, 0, 1, 4);

    auto playerNameLabel = new QLabel("Player: ");
    playerNameLabel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    statisticsContentLayout->addWidget(playerNameLabel, 1, 0);
    m_playerNameLineEdit = new QLineEdit("");
    m_playerNameLineEdit->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    statisticsContentLayout->addWidget(m_playerNameLineEdit, 1, 1, 1, 2);

    m_searchPlayerButton = new QPushButton("Search");
    m_searchPlayerButton->setFixedWidth(MENU_BUTTON_WIDTH);
    std::string playerNameRegex = "^[a-zA-Z][a-zA-Z0-9_]{0," + std::to_string(MAX_CHARS_PLAYER_NAME) + "}$";
    auto playerNameValidator = new QRegularExpressionValidator(QRegularExpression(playerNameRegex.c_str()));
    m_playerNameLineEdit->setValidator(playerNameValidator);
    connect(m_searchPlayerButton, &QPushButton::released, this, &StatisticsFrame::handleSearchPlayerButton);
    statisticsContentLayout->addWidget(m_searchPlayerButton, 1, 3);

    auto matchesPlayedLabel = new QLabel("Matches played: ");
    matchesPlayedLabel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    statisticsContentLayout->addWidget(matchesPlayedLabel, 2, 0);
    m_matchesPlayedLabel = new QLabel("0");
    m_matchesPlayedLabel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    statisticsContentLayout->addWidget(m_matchesPlayedLabel, 2, 1);

    auto matchesWonLabel = new QLabel("Matches won: ");
    matchesWonLabel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    statisticsContentLayout->addWidget(matchesWonLabel, 2, 2);
    m_matchesWonLabel = new QLabel("0");
    m_matchesWonLabel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    statisticsContentLayout->addWidget(m_matchesWonLabel, 2, 3);

    auto matchesLostLabel = new QLabel("Matches lost: ");
    matchesLostLabel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    statisticsContentLayout->addWidget(matchesLostLabel, 3, 0);
    m_matchesLostLabel = new QLabel("0");
    m_matchesLostLabel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    statisticsContentLayout->addWidget(m_matchesLostLabel, 3, 1);

    auto matchesDrawnLabel = new QLabel("Matches drawn: ");
    matchesDrawnLabel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    statisticsContentLayout->addWidget(matchesDrawnLabel, 3, 2);
    m_matchesDrawnLabel = new QLabel("0");
    m_matchesDrawnLabel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    statisticsContentLayout->addWidget(m_matchesDrawnLabel, 3, 3);

    m_matchListScrollArea = new QScrollArea();
    m_matchListScrollArea->setWidgetResizable(true);
    m_matchListScrollArea->setFixedSize(STATISTICS_SCROLL_W, STATISTICS_SCROLL_H);
    m_matchListScrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);

    statisticsContentLayout->addWidget(m_matchListScrollArea, 4, 0, 1, 4);

    auto statisticsButtonWidget = new QWidget();
    statisticsButtonWidget->setContentsMargins(0, 10, 0, 10);
    auto statisticsButtonLayout = new QHBoxLayout();
    statisticsButtonWidget->setLayout(statisticsButtonLayout);

    std::string backString = "Back";
    auto backButton = new QPushButton(backString.c_str());
    backButton->setFixedWidth(MENU_BUTTON_WIDTH);
    connect(backButton, &QPushButton::released, this,
            &StatisticsFrame::handleBackButton);
    statisticsButtonLayout->addWidget(backButton);

    statisticsContentLayout->addWidget(statisticsButtonWidget, 5, 0, 1, 4);
}

StatisticsFrame::~StatisticsFrame()
{
}

void StatisticsFrame::showEvent(QShowEvent *event)
{
    (void)event; // NO LINT

    m_playerNameLineEdit->setEnabled(false);
    m_searchPlayerButton->setEnabled(false);
    buildProgressBar();
    const auto playerName = Parameters::getPlayerName();
    m_playerNameLineEdit->setText(playerName.c_str());
    m_playerWindow->requestStatistics(playerName);
}

void StatisticsFrame::buildProgressBar()
{
    auto matchListProgressBarWidget = new QWidget();
    auto matchListProgressBarLayout = new QVBoxLayout();
    matchListProgressBarWidget->setLayout(matchListProgressBarLayout);
    matchListProgressBarWidget->setContentsMargins(50, 50, 50, 50);
    matchListProgressBarLayout->setAlignment(Qt::AlignCenter);

    auto matchListProgressBar = new QProgressBar();
    matchListProgressBar->setRange(0, 0);
    matchListProgressBarLayout->addWidget(matchListProgressBar);

    m_matchListScrollArea->setWidget(matchListProgressBarWidget);
}

void StatisticsFrame::displayStatistics(const std::string &playerName,
                                        const std::vector<std::string> &lobbyNameIds,
                                        const std::vector<std::string> &blackPlayerNames,
                                        const std::vector<std::string> &redPlayerNames,
                                        const std::vector<uint64_t> &winners,
                                        uint64_t matchesPlayed,
                                        uint64_t matchesWon,
                                        uint64_t matchesLost,
                                        uint64_t matchesDrawn)
{
    m_playerNameLineEdit->setText(playerName.c_str());
    m_matchesPlayedLabel->setText(QString::number(matchesPlayed));
    m_matchesWonLabel->setText(QString::number(matchesWon));
    m_matchesLostLabel->setText(QString::number(matchesLost));
    m_matchesDrawnLabel->setText(QString::number(matchesDrawn));

    m_lobbyNameIds = lobbyNameIds;
    m_blackPlayerNames = blackPlayerNames;
    m_redPlayerNames = redPlayerNames;
    m_winners = winners;

    buildMatchList(playerName);
}

void StatisticsFrame::buildMatchList(const std::string &playerName)
{
    auto matchListLayoutWidget = new QWidget();
    matchListLayoutWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    auto matchListLayout = new QVBoxLayout(matchListLayoutWidget);

    const auto numLobbies = m_lobbyNameIds.size();
    if (m_lobbyNameIds.size() != m_blackPlayerNames.size() ||
        m_blackPlayerNames.size() != m_redPlayerNames.size() ||
        m_redPlayerNames.size() != m_winners.size())
    {
        std::cerr << "Match vector size does not match size of other vectors" << std::endl;
    }
    for (size_t i = 0u; i < numLobbies; i++)
    {
        // Add a match details widget
        matchListLayout->addWidget(new MatchDetailsWidget(this,
                                                          playerName,
                                                          m_lobbyNameIds[i],
                                                          m_blackPlayerNames[i],
                                                          m_redPlayerNames[i],
                                                          static_cast<Winner>(m_winners[i])));
    }

    auto spacer = new QSpacerItem(0, 0, QSizePolicy::Preferred, QSizePolicy::Expanding);
    matchListLayout->addItem(spacer);

    m_matchListScrollArea->setWidget(matchListLayoutWidget);

    m_playerNameLineEdit->setEnabled(true);
    m_searchPlayerButton->setEnabled(true);

    update();
}

void StatisticsFrame::handleSearchPlayerButton()
{
    m_playerNameLineEdit->setEnabled(false);
    m_searchPlayerButton->setEnabled(false);
    buildProgressBar();
    const auto playerName = m_playerNameLineEdit->text().toStdString();
    m_playerWindow->requestStatistics(playerName);
}

void StatisticsFrame::handleBackButton()
{
    m_playerWindow->moveToMainMenuFrame();
}