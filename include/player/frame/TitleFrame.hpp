#pragma once

// #ifndef Q_MOC_RUN			  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
//  NO LINT
// #endif

#include <QFrame>
#include <QString>
#include <QLabel>
#include <QPushButton>

#include <memory>

#include "player/TitleWidget.hpp"
#include "player/LanguageSelectorWidget.hpp"
#include "player/TranslatedQLabel.hpp"
#include "player/TranslatedQPushButton.hpp"

class CheckersPlayerWindow;

class TitleFrame : public QFrame
{
	Q_OBJECT
public:
	TitleFrame(CheckersPlayerWindow *parentWindow);
	~TitleFrame();

	void showEvent(QShowEvent *event) override;

	void setConnectedToServer(bool connected);

	void reloadStrings();

private:
	void handleCreateAccountButton();
	void handleLogInAccountButton();
	void handleQuitButton();

	CheckersPlayerWindow *m_playerWindow;

	bool m_connectedToServer;

	TitleWidget *m_titleWidget;

	LanguageSelectorWidget *m_languageSelector;

	TranslatedQLabel *m_serverConnectionStatusLabel;

	TranslatedQPushButton *m_createAccountButton;
	TranslatedQPushButton *m_logInAccountButton;
	TranslatedQPushButton *m_quitButton;
};

typedef std::unique_ptr<TitleFrame> TitleFrameUniPtr;