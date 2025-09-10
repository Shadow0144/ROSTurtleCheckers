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
#include <string>

#include "player/TitleWidget.hpp"
#include "player/LanguageSelectorWidget.hpp"
#include "player/TranslatedQLabel.hpp"
#include "player/TranslatedQPushButton.hpp"

class CheckersPlayerWindow;

class LogInAccountFrame : public QFrame
{
	Q_OBJECT
public:
	LogInAccountFrame(CheckersPlayerWindow *parentWindow);
	~LogInAccountFrame();
	
	void showEvent(QShowEvent* event) override;

	void failedLogIn(const std::string &errorMessage);

	void reloadStrings();

public slots:
	void validatePlayerNameText(const QString &playerName);
	void validatePasswordText(const QString &playerPassword);

private:
	void handleLogInAccountButton();
	void handleCancelButton();

	CheckersPlayerWindow *m_playerWindow;

	TitleWidget *m_titleWidget;

	LanguageSelectorWidget *m_languageSelector;

	TranslatedQLabel *m_playerNameLabel;
	TranslatedQLabel *m_playerPasswordLabel;
	TranslatedQLabel *m_errorMessageLabel;

	QLineEdit *m_playerNameLineEdit;
	QLineEdit *m_passwordLineEdit;

	TranslatedQPushButton *m_logInAccountButton;
	TranslatedQPushButton *m_cancelButton;

	bool m_playerNameValid;
	bool m_playerPasswordValid;
	bool m_loggingInAccount;

	std::string m_errorMessage;
};

typedef std::unique_ptr<LogInAccountFrame> LogInAccountFrameUniPtr;