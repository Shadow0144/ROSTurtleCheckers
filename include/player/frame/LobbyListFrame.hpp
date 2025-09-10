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
#include "player/TranslatedQPushButton.hpp"

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

	void reloadStrings();

private:
	void handleRefreshButton();
	void handleCancelButton();
	void handleJoinLobbyButton(size_t lobbyIndex);

	void displayProgressBar();

	CheckersPlayerWindow *m_playerWindow;

	LanguageSelectorWidget *m_languageSelector;

	TitleWidget *m_titleWidget;

	QScrollArea *m_lobbyListScrollArea;

	TranslatedQPushButton *m_refreshButton;
	TranslatedQPushButton *m_cancelButton;

	std::vector<std::string> m_lobbyNames;
	std::vector<std::string> m_lobbyIds;
	std::vector<bool> m_hasPasswords;

	std::vector<LobbyDetailsWidget *> m_lobbyDetailsWidgets;
};

typedef std::unique_ptr<LobbyListFrame> LobbyListFrameUniPtr;