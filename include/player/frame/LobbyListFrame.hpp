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
#include <QScrollArea>

#include <memory>
#include <string>
#include <vector>

#include "shared/CheckersConsts.hpp"

class CheckersPlayerWindow;

class LobbyListFrame : public QFrame
{
	Q_OBJECT
public:
	LobbyListFrame(CheckersPlayerWindow *parentWindow);
	~LobbyListFrame();

	void showEvent(QShowEvent* event) override;

	void displayLobbyList(const std::vector<std::string> &lobbyNames,
						  const std::vector<std::string> &lobbyIds,
						  const std::vector<bool> &hasPasswords,
						  const std::vector<std::string> &blackPlayerNames,
						  const std::vector<std::string> &redPlayerNames);

	const std::string &getPlayerName() const;
	const std::string &getLobbyName() const;
	const std::string &getLobbyId() const;

	void setPlayerName(const std::string &playerName);

	void playerJoinedLobby(const std::string &playerName, TurtlePieceColor playerColor);
	void playerLeftLobby(const std::string &playerName);
	void setPlayerReady(const std::string &playerName, bool ready);

private:
	void handleJoinLobbyButton();
	void handleRefreshJoinLobbyButton();
	void handleCancelJoinLobbyButton();
	void handleCommitJoinLobbyButton(size_t lobbyIndex);

	void onBlackTurtleToggled(bool checked);
	void onRandomTurtleToggled(bool checked);
	void onRedTurtleToggled(bool checked);

	void buildLobbyList();

	CheckersPlayerWindow *m_playerWindow;

	QWidget *m_lobbyListLayoutWidget;

	QScrollArea *m_lobbyListScrollArea;

	QPushButton *m_joinLobbyButton;
	QPushButton *m_commitJoinLobbyButton;

	QRadioButton *m_joinLobbyBlackRadioButton;
	QRadioButton *m_joinLobbyRandomRadioButton;
	QRadioButton *m_joinLobbyRedRadioButton;

	TurtlePieceColor m_playerDesiredColor;

	std::vector<std::string> m_lobbyNames;
	std::vector<std::string> m_lobbyIds;
	std::vector<bool> m_hasPasswords;
	std::vector<std::string> m_blackPlayerNames;
	std::vector<std::string> m_redPlayerNames;
};

typedef std::unique_ptr<LobbyListFrame> LobbyListFrameUniPtr;