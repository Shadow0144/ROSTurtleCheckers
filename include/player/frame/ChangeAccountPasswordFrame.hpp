#pragma once

// #ifndef Q_MOC_RUN			  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
//  NO LINT
// #endif

#include <QFrame>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QString>

#include <memory>
#include <string>

#include "player/LanguageSelectorWidget.hpp"
#include "player/TitleWidget.hpp"

class CheckersPlayerWindow;

class ChangeAccountPasswordFrame : public QFrame
{
	Q_OBJECT
public:
	ChangeAccountPasswordFrame(CheckersPlayerWindow *parentWindow);
	~ChangeAccountPasswordFrame();

	void showEvent(QShowEvent *event) override;

	void succeededChange();
	void failedChange(const std::string &errorMessage);

	void reloadStrings();

public slots:
	void validatePreviousPasswordText(const QString &previousPlayerPassword);
	void validateNewPasswordText(const QString &newPlayerPassword);

private:
	void handleChangeAccountPasswordButton();
	void handleCancelButton();

	CheckersPlayerWindow *m_playerWindow;

	LanguageSelectorWidget *m_languageSelector;

	TitleWidget *m_titleWidget;

	QLabel *m_playerNameLabel;
	QLabel *m_currentPlayerPasswordLabel;
	QLabel *m_newPlayerPasswordLabel;
	QLabel *m_passwordWarningLabel1;
	QLabel *m_passwordWarningLabel2;
	QLabel *m_passwordWarningLabel3;
	QLabel *m_passwordWarningLabel4;
	QLabel *m_passwordWarningLabel5;
	QLabel *m_passwordWarningLabel6;
	QLabel *m_messageLabel;

	QLineEdit *m_previousPasswordLineEdit;
	QLineEdit *m_newPasswordLineEdit;

	QPushButton *m_changeAccountPasswordButton;
	QPushButton *m_cancelButton;

	bool m_previousPlayerPasswordValid;
	bool m_newPlayerPasswordValid;
	bool m_changingPassword;

	std::string m_resultMessage;
};

typedef std::unique_ptr<ChangeAccountPasswordFrame> ChangeAccountPasswordFrameUniPtr;