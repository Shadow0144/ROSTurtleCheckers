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

#include "shared/CheckersConsts.hpp"

class CheckersPlayerWindow;

class ChangeAccountPasswordFrame : public QFrame
{
	Q_OBJECT
public:
	ChangeAccountPasswordFrame(CheckersPlayerWindow *parentWindow);
	~ChangeAccountPasswordFrame();
	
	void showEvent(QShowEvent* event) override;

	void succeededChange();
	void failedChange(const std::string &errorMessage);

public slots:
	void validatePreviousPasswordText(const QString &previousPlayerPassword);
	void validateNewPasswordText(const QString &newPlayerPassword);

private:
	void handleChangeAccountPasswordButton();
	void handleCancelButton();

	CheckersPlayerWindow *m_playerWindow;

	QLabel *m_playerNameLabel;
	QLabel *m_messageLabel;

	QLineEdit *m_previousPasswordLineEdit;
	QLineEdit *m_newPasswordLineEdit;

	QPushButton *m_changeAccountPasswordButton;

	bool m_previousPlayerPasswordValid;
	bool m_newPlayerPasswordValid;
};

typedef std::unique_ptr<ChangeAccountPasswordFrame> ChangeAccountPasswordFrameUniPtr;