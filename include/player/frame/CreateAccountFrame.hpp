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

class CreateAccountFrame : public QFrame
{
	Q_OBJECT
public:
	CreateAccountFrame(CheckersPlayerWindow *parentWindow);
	~CreateAccountFrame();

public slots:
	void validatePlayerNameText(const QString &playerName);
	void onCreatePlayerPasswordTextChanged(const QString &playerPassword);

private:
	void handleCancelCreatePlayerButton();
	void handleCommitCreatePlayerButton();

	CheckersPlayerWindow *m_playerWindow;

	QLineEdit *m_playerNameLineEdit;
	QLineEdit *m_createPlayerPasswordLineEdit;

	QPushButton *m_commitCreatePlayerButton;

	std::string m_playerName;
};

typedef std::unique_ptr<CreateAccountFrame> CreateAccountFrameUniPtr;