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
#include <QProgressBar>
#include <QSpacerItem>

#include <cstdlib>
#include <ctime>
#include <memory>
#include <string>
#include <vector>

#include "player/CheckersPlayerWindow.hpp"
#include "shared/TurtleLogger.hpp"
#include "shared/CheckersConsts.hpp"
#include "player/Parameters.hpp"
#include "player/StringLibrary.hpp"
#include "player/TitleWidget.hpp"
#include "player/LanguageSelectorWidget.hpp"
#include "player/MatchDetailsWidget.hpp"

StatisticsFrame::StatisticsFrame(
    CheckersPlayerWindow *parentWindow)
    : QFrame(parentWindow, Qt::WindowFlags())
{
    m_playerWindow = parentWindow;

    auto statisticsLayout = new QVBoxLayout(this);
    statisticsLayout->setAlignment(Qt::AlignCenter);

    m_languageSelector = new LanguageSelectorWidget(this);

    auto statisticsContentLayout = new QGridLayout();
    statisticsContentLayout->setAlignment(Qt::AlignCenter);
    statisticsLayout->addLayout(statisticsContentLayout);

    auto spacer = new QSpacerItem(0, 0, QSizePolicy::Preferred, QSizePolicy::MinimumExpanding);
    statisticsContentLayout->addItem(spacer, 0, 0, 1, 4);

    m_titleWidget = new TitleWidget();
    statisticsContentLayout->addWidget(m_titleWidget, 1, 0, 1, 4);

    m_playerNameLabel = new QLabel(StringLibrary::getTranslatedString("Player: "));
    m_playerNameLabel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    statisticsContentLayout->addWidget(m_playerNameLabel, 2, 0);
    m_playerNameLineEdit = new QLineEdit("");
    m_playerNameLineEdit->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    statisticsContentLayout->addWidget(m_playerNameLineEdit, 2, 1, 1, 2);

    m_searchPlayerButton = new QPushButton(StringLibrary::getTranslatedString("Search"));
    m_searchPlayerButton->setFixedWidth(MENU_BUTTON_WIDTH);
    std::string playerNameRegex = "^[a-zA-Z][a-zA-Z0-9_]{0," + std::to_string(MAX_CHARS_PLAYER_NAME) + "}$";
    auto playerNameValidator = new QRegularExpressionValidator(QRegularExpression(playerNameRegex.c_str()));
    m_playerNameLineEdit->setValidator(playerNameValidator);
    connect(m_searchPlayerButton, &QPushButton::released, this, &StatisticsFrame::handleSearchPlayerButton);
    statisticsContentLayout->addWidget(m_searchPlayerButton, 2, 3);

    m_matchesPlayedLabel = new QLabel(StringLibrary::getTranslatedString("Matches played: "));
    m_matchesPlayedLabel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    statisticsContentLayout->addWidget(m_matchesPlayedLabel, 3, 0);
    m_matchesPlayedNumberLabel = new QLabel("0");
    m_matchesPlayedNumberLabel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    statisticsContentLayout->addWidget(m_matchesPlayedNumberLabel, 3, 1);

    m_matchesWonLabel = new QLabel(StringLibrary::getTranslatedString("Matches won: "));
    m_matchesWonLabel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    statisticsContentLayout->addWidget(m_matchesWonLabel, 3, 2);
    m_matchesWonNumberLabel = new QLabel("0");
    m_matchesWonNumberLabel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    statisticsContentLayout->addWidget(m_matchesWonNumberLabel, 3, 3);

    m_matchesLostLabel = new QLabel(StringLibrary::getTranslatedString("Matches lost: "));
    m_matchesLostLabel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    statisticsContentLayout->addWidget(m_matchesLostLabel, 4, 0);
    m_matchesLostNumberLabel = new QLabel("0");
    m_matchesLostNumberLabel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    statisticsContentLayout->addWidget(m_matchesLostNumberLabel, 4, 1);

    m_matchesDrawnLabel = new QLabel(StringLibrary::getTranslatedString("Matches drawn: "));
    m_matchesDrawnLabel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    statisticsContentLayout->addWidget(m_matchesDrawnLabel, 4, 2);
    m_matchesDrawnNumberLabel = new QLabel("0");
    m_matchesDrawnNumberLabel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    statisticsContentLayout->addWidget(m_matchesDrawnNumberLabel, 4, 3);

    m_matchListScrollArea = new QScrollArea();
    m_matchListScrollArea->setWidgetResizable(true);
    m_matchListScrollArea->setFixedSize(STATISTICS_SCROLL_W, STATISTICS_SCROLL_H);
    m_matchListScrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);

    statisticsContentLayout->addWidget(m_matchListScrollArea, 5, 0, 1, 4);

    auto statisticsButtonWidget = new QWidget();
    auto statisticsButtonLayout = new QHBoxLayout();
    statisticsButtonWidget->setLayout(statisticsButtonLayout);
    statisticsButtonWidget->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);

    m_backButton = new QPushButton(StringLibrary::getTranslatedString("Back"));
    m_backButton->setFixedWidth(MENU_BUTTON_WIDTH);
    connect(m_backButton, &QPushButton::released, this,
            &StatisticsFrame::handleBackButton);
    statisticsButtonLayout->addWidget(m_backButton);

    statisticsContentLayout->addWidget(statisticsButtonWidget, 6, 0, 1, 4);

    spacer = new QSpacerItem(0, 0, QSizePolicy::Preferred, QSizePolicy::MinimumExpanding);
    statisticsContentLayout->addItem(spacer, 7, 0, 1, 4);
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

    m_languageSelector->setCurrentIndex(static_cast<int>(Parameters::getLanguage()));
    reloadStrings();
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
    m_matchesPlayedNumberLabel->setText(QString::number(matchesPlayed));
    m_matchesWonNumberLabel->setText(QString::number(matchesWon));
    m_matchesLostNumberLabel->setText(QString::number(matchesLost));
    m_matchesDrawnNumberLabel->setText(QString::number(matchesDrawn));

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
        TurtleLogger::logError("Match vector size does not match size of other vectors");
    }
    m_matchDetailsWidgets.clear();
    for (size_t i = 0u; i < numLobbies; i++)
    {
        // Add a match details widget
        auto matchDetailsWidget = new MatchDetailsWidget(this,
                                                         playerName,
                                                         m_lobbyNameIds[i],
                                                         m_blackPlayerNames[i],
                                                         m_redPlayerNames[i],
                                                         static_cast<Winner>(m_winners[i]));
        m_matchDetailsWidgets.push_back(matchDetailsWidget);
        matchListLayout->addWidget(matchDetailsWidget);
    }

    auto spacer = new QSpacerItem(0, 0, QSizePolicy::Preferred, QSizePolicy::Expanding);
    matchListLayout->addItem(spacer);

    // This will delete the previous widget and all its children
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

void StatisticsFrame::reloadStrings()
{
    m_titleWidget->reloadStrings();

    for (auto &matchDetailsWidget : m_matchDetailsWidgets)
    {
        matchDetailsWidget->reloadStrings();
    }

    m_playerNameLabel->setText(StringLibrary::getTranslatedString("Player: "));

    m_searchPlayerButton->setText(StringLibrary::getTranslatedString("Search"));

    m_matchesPlayedLabel->setText(StringLibrary::getTranslatedString("Matches played: "));
    m_matchesWonLabel->setText(StringLibrary::getTranslatedString("Matches won: "));
    m_matchesLostLabel->setText(StringLibrary::getTranslatedString("Matches lost: "));
    m_matchesDrawnLabel->setText(StringLibrary::getTranslatedString("Matches drawn: "));

    m_backButton->setText(StringLibrary::getTranslatedString("Back"));
}