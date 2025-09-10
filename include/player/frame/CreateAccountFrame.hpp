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

#include "player/LanguageSelectorWidget.hpp"
#include "player/TitleWidget.hpp"
#include "player/TranslatedQLabel.hpp"
#include "player/TranslatedQPushButton.hpp"

class CheckersPlayerWindow;

class CreateAccountFrame : public QFrame
{
	Q_OBJECT
public:
	CreateAccountFrame(CheckersPlayerWindow *parentWindow);
	~CreateAccountFrame();

	void showEvent(QShowEvent *event) override;

	void failedCreate(const std::string &errorMessage);

	void reloadStrings();

public slots:
	void validatePlayerNameText(const QString &playerName);
	void validatePasswordText(const QString &playerPassword);

private:
	void handleCreateAccountButton();
	void handleCancelButton();

	CheckersPlayerWindow *m_playerWindow;

	LanguageSelectorWidget *m_languageSelector;

	TitleWidget *m_titleWidget;

	QLineEdit *m_playerNameLineEdit;
	QLineEdit *m_passwordLineEdit;

	TranslatedQLabel *m_playerNameLabel;
	TranslatedQLabel *m_playerPasswordLabel;
	TranslatedQLabel *m_passwordWarningLabel1;
	TranslatedQLabel *m_passwordWarningLabel2;
	TranslatedQLabel *m_passwordWarningLabel3;
	TranslatedQLabel *m_passwordWarningLabel4;
	QLabel *m_passwordWarningLabel5;
	TranslatedQLabel *m_passwordWarningLabel6;
	TranslatedQLabel *m_errorMessageLabel;

	TranslatedQPushButton *m_createAccountButton;
	TranslatedQPushButton *m_cancelButton;

	bool m_playerNameValid;
	bool m_playerPasswordValid;
	bool m_creatingAccount;

	std::string m_errorMessage;
};

typedef std::unique_ptr<CreateAccountFrame> CreateAccountFrameUniPtr;