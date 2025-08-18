#pragma once

// #ifndef Q_MOC_RUN			  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
//  NO LINT
// #endif

#include <QFrame>
#include <QPushButton>
#include <QScrollArea>
#include <QLabel>
#include <QLineEdit>

#include <memory>
#include <string>
#include <vector>

#include "player/TitleWidget.hpp"
#include "player/LanguageSelectorWidget.hpp"
#include "player/MatchDetailsWidget.hpp"

class CheckersPlayerWindow;

class StatisticsFrame : public QFrame
{
    Q_OBJECT
public:
    StatisticsFrame(CheckersPlayerWindow *parentWindow);
    ~StatisticsFrame();

    void showEvent(QShowEvent *event) override;

    void displayStatistics(const std::string &playerName,
                           const std::vector<std::string> &lobbyNameIds,
                           const std::vector<std::string> &blackPlayerNames,
                           const std::vector<std::string> &redPlayerNames,
                           const std::vector<uint64_t> &winners,
                           uint64_t matchesPlayed,
                           uint64_t matchesWon,
                           uint64_t matchesLost,
                           uint64_t matchesDrawn);

    void reloadStrings();

private:
    void handleSearchPlayerButton();
    void handleBackButton();

    void buildProgressBar();
    void buildMatchList(const std::string &playerName);

    CheckersPlayerWindow *m_playerWindow;

    TitleWidget *m_titleWidget;

    LanguageSelectorWidget *m_languageSelector;

    QLineEdit *m_playerNameLineEdit;

    QPushButton *m_searchPlayerButton;

    QLabel *m_playerNameLabel;
    QLabel *m_matchesPlayedLabel;
    QLabel *m_matchesPlayedNumberLabel;
    QLabel *m_matchesWonLabel;
    QLabel *m_matchesWonNumberLabel;
    QLabel *m_matchesLostLabel;
    QLabel *m_matchesLostNumberLabel;
    QLabel *m_matchesDrawnLabel;
    QLabel *m_matchesDrawnNumberLabel;

    QScrollArea *m_matchListScrollArea;

    std::vector<MatchDetailsWidget *> m_matchDetailsWidgets;

    QPushButton *m_backButton;

    std::vector<std::string> m_lobbyNameIds;
    std::vector<std::string> m_blackPlayerNames;
    std::vector<std::string> m_redPlayerNames;
    std::vector<uint64_t> m_winners;
};

typedef std::unique_ptr<StatisticsFrame> StatisticsFrameUniPtr;