#pragma once

// #ifndef Q_MOC_RUN			  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
//  NO LINT
// #endif

#include <QFrame>
#include <QPushButton>
#include <QRadioButton>
#include <QScrollArea>
#include <QLabel>

#include <memory>
#include <string>
#include <vector>

#include "shared/CheckersConsts.hpp"

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
                           uint64_t matchesDrawed);

private:
    void handleBackButton();
    void handleViewPlayerButton(const std::string &playerName);

    void buildMatchList();

    CheckersPlayerWindow *m_playerWindow;

    QLabel *m_playerNameLabel;
    QLabel *m_matchesPlayedLabel;
    QLabel *m_matchesWonLabel;
    QLabel *m_matchesLostLabel;
    QLabel *m_matchesDrawedLabel;

	QWidget *m_matchListLayoutWidget;

    QScrollArea *m_matchListScrollArea;

    std::vector<std::string> m_lobbyNameIds;
    std::vector<std::string> m_blackPlayerNames;
    std::vector<std::string> m_redPlayerNames;
    std::vector<uint64_t> m_winners;
};

typedef std::unique_ptr<StatisticsFrame> StatisticsFrameUniPtr;