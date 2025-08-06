#include "player/frame/StatisticsFrame.hpp"

#include <QFrame>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QScrollArea>
#include <QLabel>
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

    auto titleWidget = new TitleWidget();
    statisticsLayout->addWidget(titleWidget);

    auto playerNameLayout = new QHBoxLayout();
    auto playerNameLabel = new QLabel("Player: ");
    playerNameLayout->addWidget(playerNameLabel);
    m_playerNameLabel = new QLabel("");
    m_playerNameLabel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    playerNameLayout->addWidget(m_playerNameLabel);
    statisticsLayout->addLayout(playerNameLayout);

    auto playerSummaryStatsLayout = new QGridLayout();

    auto matchesPlayedLabel = new QLabel("Matches played: ");
    playerSummaryStatsLayout->addWidget(matchesPlayedLabel, 0, 0);
    m_matchesPlayedLabel = new QLabel("");
    playerSummaryStatsLayout->addWidget(m_matchesPlayedLabel, 0, 1);

    auto matchesWonLabel = new QLabel("Matches won: ");
    playerSummaryStatsLayout->addWidget(matchesWonLabel, 0, 2);
    m_matchesWonLabel = new QLabel("");
    playerSummaryStatsLayout->addWidget(m_matchesWonLabel, 0, 3);

    auto matchesLostLabel = new QLabel("Matches lost: ");
    playerSummaryStatsLayout->addWidget(matchesLostLabel, 1, 0);
    m_matchesLostLabel = new QLabel("");
    playerSummaryStatsLayout->addWidget(m_matchesLostLabel, 1, 1);

    auto matchesDrawedLabel = new QLabel("Matches drawed: ");
    playerSummaryStatsLayout->addWidget(matchesDrawedLabel, 1, 2);
    m_matchesDrawedLabel = new QLabel("");
    playerSummaryStatsLayout->addWidget(m_matchesDrawedLabel, 1, 3);

    statisticsLayout->addLayout(playerSummaryStatsLayout);

    m_matchListScrollArea = new QScrollArea();
    m_matchListScrollArea->setWidgetResizable(true);
    m_matchListScrollArea->setFixedSize(STATISTICS_SCROLL_W, STATISTICS_SCROLL_H);
    m_matchListScrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);

    m_matchListLayoutWidget = nullptr;
    buildMatchList();

    statisticsLayout->addWidget(m_matchListScrollArea);

    auto statisticsButtonLayout = new QHBoxLayout();

    std::string backString = "Back";
    auto backButton = new QPushButton(backString.c_str());
    backButton->setFixedWidth(MENU_BUTTON_WIDTH);
    connect(backButton, &QPushButton::released, this,
            &StatisticsFrame::handleBackButton);
    statisticsButtonLayout->addWidget(backButton);

    statisticsLayout->addLayout(statisticsButtonLayout);
}

StatisticsFrame::~StatisticsFrame()
{
}

void StatisticsFrame::showEvent(QShowEvent *event)
{
    (void)event; // NO LINT

    const auto playerName = Parameters::getPlayerName();
    m_playerNameLabel->setText(playerName.c_str());
    m_playerWindow->requestStatistics(playerName);
}

void StatisticsFrame::displayStatistics(const std::string &playerName,
                                        const std::vector<std::string> &lobbyNameIds,
                                        const std::vector<std::string> &blackPlayerNames,
                                        const std::vector<std::string> &redPlayerNames,
                                        const std::vector<uint64_t> &winners,
                                        uint64_t matchesPlayed,
                                        uint64_t matchesWon,
                                        uint64_t matchesLost,
                                        uint64_t matchesDrawed)
{
    m_playerNameLabel->setText(playerName.c_str());
    m_matchesPlayedLabel->setText(QString::number(matchesPlayed));
    m_matchesWonLabel->setText(QString::number(matchesWon));
    m_matchesLostLabel->setText(QString::number(matchesLost));
    m_matchesDrawedLabel->setText(QString::number(matchesDrawed));

    m_lobbyNameIds = lobbyNameIds;
    m_blackPlayerNames = blackPlayerNames;
    m_redPlayerNames = redPlayerNames;
    m_winners = winners;

    buildMatchList();
}

void StatisticsFrame::buildMatchList()
{
    if (m_matchListLayoutWidget)
    {
        delete m_matchListLayoutWidget;
        m_matchListLayoutWidget = nullptr;
    }

    m_matchListLayoutWidget = new QWidget();
    m_matchListLayoutWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    auto matchListLayout = new QVBoxLayout(m_matchListLayoutWidget);

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
                                                          m_lobbyNameIds[i],
                                                          m_blackPlayerNames[i],
                                                          m_redPlayerNames[i],
                                                          static_cast<Winner>(m_winners[i])));
    }

    auto spacer = new QSpacerItem(0, 0, QSizePolicy::Preferred, QSizePolicy::Expanding);
    matchListLayout->addItem(spacer);

    m_matchListScrollArea->setWidget(m_matchListLayoutWidget);

    update();
}

void StatisticsFrame::handleBackButton()
{
    m_playerWindow->moveToMainMenuFrame();
}

void StatisticsFrame::handleViewPlayerButton(const std::string &playerName)
{
}