#pragma once

// #ifndef Q_MOC_RUN			  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
//  NO LINT
// #endif

#include <QFrame>
#include <QPushButton>
#include <QRadioButton>
#include <QScrollArea>

#include <memory>
#include <string>
#include <vector>

#include "shared/CheckersConsts.hpp"
#include "player/LanguageSelectorWidget.hpp"
#include "player/TitleWidget.hpp"
#include "player/LobbyDetailsWidget.hpp"

class CheckersPlayerWindow;

class LobbyListFrame : public QFrame
{
	Q_OBJECT
public:
	LobbyListFrame(CheckersPlayerWindow *parentWindow);
	~LobbyListFrame();

	void showEvent(QShowEvent *event) override;

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

	void reloadStrings();

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

	LanguageSelectorWidget *m_languageSelector;

	TitleWidget *m_titleWidget;

	QWidget *m_lobbyListLayoutWidget;

	QScrollArea *m_lobbyListScrollArea;

	QPushButton *m_refreshJoinLobbyButton;
	QPushButton *m_cancelJoinLobbyButton;

	QRadioButton *m_blackRadioButton;
	QRadioButton *m_randomRadioButton;
	QRadioButton *m_redRadioButton;

	TurtlePieceColor m_playerDesiredColor;

	std::vector<std::string> m_lobbyNames;
	std::vector<std::string> m_lobbyIds;
	std::vector<bool> m_hasPasswords;
	std::vector<std::string> m_blackPlayerNames;
	std::vector<std::string> m_redPlayerNames;

	std::vector<LobbyDetailsWidget *> m_lobbyDetailsWidgets;
};

typedef std::unique_ptr<LobbyListFrame> LobbyListFrameUniPtr;