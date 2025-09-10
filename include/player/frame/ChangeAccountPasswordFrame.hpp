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
#include "player/TranslatedQLabel.hpp"
#include "player/TranslatedQPushButton.hpp"

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
	TranslatedQLabel *m_currentPlayerPasswordLabel;
	TranslatedQLabel *m_newPlayerPasswordLabel;
	TranslatedQLabel *m_passwordWarningLabel1;
	TranslatedQLabel *m_passwordWarningLabel2;
	TranslatedQLabel *m_passwordWarningLabel3;
	TranslatedQLabel *m_passwordWarningLabel4;
	QLabel *m_passwordWarningLabel5;
	TranslatedQLabel *m_passwordWarningLabel6;
	TranslatedQLabel *m_messageLabel;

	QLineEdit *m_previousPasswordLineEdit;
	QLineEdit *m_newPasswordLineEdit;

	TranslatedQPushButton *m_changeAccountPasswordButton;
	TranslatedQPushButton *m_cancelButton;

	bool m_previousPlayerPasswordValid;
	bool m_newPlayerPasswordValid;
	bool m_changingPassword;

	std::string m_resultMessage;
};

typedef std::unique_ptr<ChangeAccountPasswordFrame> ChangeAccountPasswordFrameUniPtr;