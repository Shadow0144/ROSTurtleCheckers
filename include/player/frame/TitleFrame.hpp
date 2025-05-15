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

class TitleFrame : public QFrame
{
	Q_OBJECT
public:
	TitleFrame(CheckersPlayerWindow *parentWindow);
	~TitleFrame();

	void setConnectedToServer(bool connected);

private:
	void handleCreateAccountButton();
	void handleLoginButton();
	void handleQuitButton();

	CheckersPlayerWindow *m_playerWindow;

	bool m_connectedToServer;

	QPushButton *m_createAccountButton;
	QPushButton *m_loginAccountButton;

	QLabel *m_connectedToServerLabel;
};

typedef std::unique_ptr<TitleFrame> TitleFrameUniPtr;