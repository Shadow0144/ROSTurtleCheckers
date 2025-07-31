#pragma once

// #ifndef Q_MOC_RUN			  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
//  NO LINT
// #endif

#include <QFrame>
#include <QLineEdit>
#include <QLabel>
#include <QPushButton>

#include <memory>

#include "shared/CheckersConsts.hpp"

class CheckersPlayerWindow;

class MainMenuFrame : public QFrame
{
	Q_OBJECT
public:
	MainMenuFrame(CheckersPlayerWindow *parentWindow);
	~MainMenuFrame();

	void showEvent(QShowEvent *event) override;

private:
	void handleStatisticsButton();
	void handleLogOutAccountButton();
	void handleCreateLobbyButton();
	void handleJoinLobbyButton();
	void handleQuitButton();

	CheckersPlayerWindow *m_playerWindow;

	QLineEdit *m_playerNameLineEdit;

	QLabel *m_playerNameLabel;

	QPushButton *m_createLobbyButton;
	QPushButton *m_joinLobbyButton;
};

typedef std::unique_ptr<MainMenuFrame> MainMenuFrameUniPtr;