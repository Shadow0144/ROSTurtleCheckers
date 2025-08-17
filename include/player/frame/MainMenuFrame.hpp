#pragma once

// #ifndef Q_MOC_RUN			  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
//  NO LINT
// #endif

#include <QFrame>
#include <QLabel>
#include <QPushButton>

#include <memory>

#include "player/TitleWidget.hpp"
#include "player/LanguageSelectorWidget.hpp"

class CheckersPlayerWindow;

class MainMenuFrame : public QFrame
{
	Q_OBJECT
public:
	MainMenuFrame(CheckersPlayerWindow *parentWindow);
	~MainMenuFrame();

	void showEvent(QShowEvent *event) override;

	void reloadStrings();

private:
	void handleCreateLobbyButton();
	void handleJoinLobbyButton();
	void handleStatisticsButton();
	void handleChangeAccountPasswordButton();
	void handleLogOutAccountButton();
	void handleQuitButton();

	CheckersPlayerWindow *m_playerWindow;

	TitleWidget *m_titleWidget;

	LanguageSelectorWidget *m_languageSelector;

	QLabel *m_playerNameLabel;

	QPushButton *m_createLobbyButton;
	QPushButton *m_joinLobbyButton;
	QPushButton *m_statisticsButton;
	QPushButton *m_changeAccountPasswordAccountButton;
	QPushButton *m_logOutAccountButton;
	QPushButton *m_quitButton;
};

typedef std::unique_ptr<MainMenuFrame> MainMenuFrameUniPtr;