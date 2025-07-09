#pragma once

// #ifndef Q_MOC_RUN			  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
//  NO LINT
// #endif

#include <QFrame>
#include <QString>
#include <QLabel>
#include <QPushButton>

#include <memory>

#include "shared/CheckersConsts.hpp"

class CheckersPlayerWindow;

class TitleFrame : public QFrame
{
	Q_OBJECT
public:
	TitleFrame(CheckersPlayerWindow *parentWindow);
	~TitleFrame();
	
	void showEvent(QShowEvent* event) override;

	void setConnectedToServer(bool connected);

private:
	void handleCreateAccountButton();
	void handleLogInAccountButton();
	void handleQuitButton();

	CheckersPlayerWindow *m_playerWindow;

	bool m_connectedToServer;

	QPushButton *m_createAccountButton;
	QPushButton *m_logInAccountButton;

	QLabel *m_serverConnectionStatusLabel;
	const QString m_connectingString = "Connecting to server...";
	const QString m_connectedString = "Connected to server!";
};

typedef std::unique_ptr<TitleFrame> TitleFrameUniPtr;