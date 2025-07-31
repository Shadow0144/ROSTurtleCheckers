#include "player/frame/StatisticsFrame.hpp"

#include <QFrame>
#include <QHBoxLayout>
#include <QVBoxLayout>
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

    m_playerNameLabel = new QLabel("");
    statisticsLayout->addWidget(m_playerNameLabel);

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
}

void StatisticsFrame::handleBackButton()
{
    m_playerWindow->moveToMainMenuFrame();
}

void StatisticsFrame::handleViewPlayerButton(const std::string &playerName)
{
}