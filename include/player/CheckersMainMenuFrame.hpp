#pragma once

// #ifndef Q_MOC_RUN			  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
//  NO LINT
// #endif

#include <QFrame>
#include <QLineEdit>
#include <QImage>
#include <QPushButton>
#include <QPainter>
#include <QPaintEvent>
#include <QVector>

#include <memory>
#include <string>
#include <vector>

#include "shared/CheckersConsts.hpp"

class CheckersPlayerWindow;

class CheckersMainMenuFrame : public QFrame
{
	Q_OBJECT
public:
	CheckersMainMenuFrame(CheckersPlayerWindow *parentWindow);
	~CheckersMainMenuFrame();

	void setConnectedToServer(bool connected);

	const std::string &getPlayerName() const;
	const std::string &getLobbyName() const;

public slots:
	void validatePlayerNameText(const QString &playerName);

private:
	void handleCreateLobbyButton();
	void handleJoinLobbyButton();
	void handleQuitButton();

	CheckersPlayerWindow *m_playerWindow;

	bool m_connectedToServer;

	QLineEdit *m_playerNameLineEdit;

	QPushButton *m_createLobbyButton;
	QPushButton *m_joinLobbyButton;
	QPushButton *m_quitButton;

	QString m_playerNameLineEditDefaultStyleSheet;
	QString m_playerNameLineEditInvalidStyleSheet;

	QString m_buttonDefaultStyleSheet;
	QString m_buttonDisabledStyleSheet;

	std::string m_lobbyName;
	std::string m_playerName;
	TurtlePieceColor m_playerColor;
};

typedef std::unique_ptr<CheckersMainMenuFrame> CheckersMainMenuFrameUniPtr;