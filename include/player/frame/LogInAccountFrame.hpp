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

#include "shared/CheckersConsts.hpp"

class CheckersPlayerWindow;

class LogInAccountFrame : public QFrame
{
	Q_OBJECT
public:
	LogInAccountFrame(CheckersPlayerWindow *parentWindow);
	~LogInAccountFrame();
	
	void showEvent(QShowEvent* event) override;

	void failedLogIn(const std::string &errorMessage);

public slots:
	void validatePlayerNameText(const QString &playerName);
	void validatePasswordText(const QString &playerPassword);

private:
	void handleLogInAccountButton();
	void handleCancelButton();

	CheckersPlayerWindow *m_playerWindow;

	QLineEdit *m_playerNameLineEdit;
	QLineEdit *m_passwordLineEdit;

	QLabel *m_errorMessageLabel;

	QPushButton *m_logInAccountButton;

	bool m_playerNameValid;
	bool m_playerPasswordValid;
};

typedef std::unique_ptr<LogInAccountFrame> LogInAccountFrameUniPtr;