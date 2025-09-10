#pragma once

// #ifndef Q_MOC_RUN			  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
//  NO LINT
// #endif

#include <QFrame>
#include <QLineEdit>
#include <QLabel>
#include <QPushButton>
#include <QString>

#include <memory>

#include "player/LanguageSelectorWidget.hpp"
#include "player/TitleWidget.hpp"
#include "player/TranslatedQLabel.hpp"
#include "player/TranslatedQPushButton.hpp"

class CheckersPlayerWindow;

class LobbyPasswordFrame : public QFrame
{
	Q_OBJECT
public:
	LobbyPasswordFrame(CheckersPlayerWindow *parentWindow);
	~LobbyPasswordFrame();
	
	void showEvent(QShowEvent* event) override;

	void setPasswordIncorrect();

	void reloadStrings();

public slots:
	void validatePasswordText(const QString &lobbyPassword);

private:
	void handleCancelButton();
	void handleConfirmPasswordButton();

	CheckersPlayerWindow *m_playerWindow;

	LanguageSelectorWidget *m_languageSelector;

	TitleWidget *m_titleWidget;

	QLabel *m_lobbyNameLabel;
	QLabel *m_lobbyIdLabel;
	TranslatedQLabel *m_lobbyPasswordLabel;
	TranslatedQLabel *m_passwordIncorrectLabel;

	QLineEdit *m_lobbyPasswordLineEdit;

	TranslatedQPushButton *m_confirmPasswordButton;
	TranslatedQPushButton *m_cancelJoinLobbyButton;
};

typedef std::unique_ptr<LobbyPasswordFrame> LobbyPasswordFrameUniPtr;