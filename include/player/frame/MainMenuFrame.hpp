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
#include "player/TranslatedQPushButton.hpp"

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

	TranslatedQPushButton *m_createLobbyButton;
	TranslatedQPushButton *m_joinLobbyButton;
	TranslatedQPushButton *m_statisticsButton;
	TranslatedQPushButton *m_changeAccountPasswordAccountButton;
	TranslatedQPushButton *m_logOutAccountButton;
	TranslatedQPushButton *m_quitButton;
};

typedef std::unique_ptr<MainMenuFrame> MainMenuFrameUniPtr;