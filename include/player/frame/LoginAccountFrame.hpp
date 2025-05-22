#pragma once

// #ifndef Q_MOC_RUN			  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
//  NO LINT
// #endif

#include <QFrame>
#include <QLineEdit>
#include <QImage>
#include <QLabel>
#include <QPushButton>
#include <QRadioButton>
#include <QButtonGroup>
#include <QCheckBox>
#include <QPainter>
#include <QPaintEvent>
#include <QVector>
#include <QStackedLayout>

#include <memory>
#include <string>
#include <vector>

#include "shared/CheckersConsts.hpp"

class CheckersPlayerWindow;

class LoginAccountFrame : public QFrame
{
	Q_OBJECT
public:
	LoginAccountFrame(CheckersPlayerWindow *parentWindow);
	~LoginAccountFrame();
	
	void showEvent(QShowEvent* event) override;

	void failedLogin(const std::string &errorMessage);

public slots:
	void validatePlayerNameText(const QString &playerName);
	void validatePasswordText(const QString &playerPassword);

private:
	void handleLoginAccountButton();
	void handleCancelButton();

	CheckersPlayerWindow *m_playerWindow;

	QLineEdit *m_playerNameLineEdit;
	QLineEdit *m_passwordLineEdit;

	QLabel *m_errorMessageLabel;

	QPushButton *m_loginAccountButton;

	bool m_playerNameValid;
	bool m_playerPasswordValid;
};

typedef std::unique_ptr<LoginAccountFrame> LoginAccountFrameUniPtr;