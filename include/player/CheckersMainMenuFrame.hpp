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

class CheckersMainMenuFrame : public QFrame
{
	Q_OBJECT
public:
	CheckersMainMenuFrame(CheckersPlayerWindow *parentWindow);
	~CheckersMainMenuFrame();

	void setConnectedToServer(bool connected);

	void connectedToLobby(const std::string &lobbyName,
						  const std::string &lobbyId,
						  const std::string &blackPlayerName,
						  const std::string &redPlayerName,
						  bool blackPlayerReady,
						  bool redPlayerReady);

	void displayLobbyList(const std::vector<std::string> &lobbyNames,
						  const std::vector<std::string> &lobbyIds,
						  const std::vector<std::string> &blackPlayerNames,
						  const std::vector<std::string> &redPlayerNames);

	const std::string &getPlayerName() const;
	const std::string &getLobbyName() const;
	const std::string &getLobbyId() const;

	void playerJoinedLobby(const std::string &playerName, TurtlePieceColor playerColor);
	void playerLeftLobby(const std::string &playerName);
	void setPlayerReady(const std::string &playerName, bool ready);

public slots:
	void validatePlayerNameText(const QString &playerName);
	void validatelobbyNameText(const QString &lobbyName);

private:
	QWidget *createMainMenuScreen();
	QWidget *createCreateLobbyScreen();
	QWidget *createJoinLobbyScreen();
	QWidget *createInLobbyScreen();

	void handleCreateLobbyButton();
	void handleCancelCreateLobbyButton();
	void handleCommitCreateLobbyButton();
	void handleJoinLobbyButton();
	void handleRefreshJoinLobbyButton();
	void handleCancelJoinLobbyButton();
	void handleCommitJoinLobbyButton(size_t lobbyIndex);
	void handleLeaveLobbyButton();
	void handleQuitButton();

	void onBlackTurtleToggled(bool checked);
	void onRandomTurtleToggled(bool checked);
	void onRedTurtleToggled(bool checked);

	void handleBlackReadyButtonToggled(int state);
	void handleRedReadyButtonToggled(int state);

	CheckersPlayerWindow *m_playerWindow;

	bool m_connectedToServer;

	QStackedLayout *m_windowLayout;
	static constexpr int MAIN_MENU_INDEX = 0;
	static constexpr int CREATE_LOBBY_INDEX = 1;
	static constexpr int JOIN_LOBBY_INDEX = 2;
	static constexpr int IN_LOBBY_INDEX = 3;

	QLineEdit *m_playerNameLineEdit;
	QLineEdit *m_lobbyNameLineEdit;

	QPushButton *m_createLobbyButton;
	QPushButton *m_commitCreateLobbyButton;
	QPushButton *m_joinLobbyButton;

	QRadioButton *m_createLobbyBlackRadioButton;
	QRadioButton *m_createLobbyRandomRadioButton;
	QRadioButton *m_createLobbyRedRadioButton;

	QRadioButton *m_joinLobbyBlackRadioButton;
	QRadioButton *m_joinLobbyRandomRadioButton;
	QRadioButton *m_joinLobbyRedRadioButton;

	QCheckBox *m_blackReadyInLobbyCheckBox;
	QCheckBox *m_redReadyInLobbyCheckBox;

	QLabel *m_blackPlayerNameLabel;
	QLabel *m_redPlayerNameLabel;

	QString m_labelStyleSheet;
	QString m_openNameLabelStyleSheet;

	QString m_lineEditValidStyleSheet;
	QString m_lineEditInvalidStyleSheet;

	QString m_buttonDefaultStyleSheet;
	QString m_buttonDisabledStyleSheet;

	QString m_selectedRadioButtonStyleSheet;
	QString m_unselectedRadioButtonStyleSheet;

	QString m_unreadyButtonStyleSheet;
	QString m_disabledReadyButtonStyleSheet;
	QString m_readyButtonStyleSheet;

	std::string m_playerName;
	std::string m_lobbyName;
	std::string m_lobbyId;
	TurtlePieceColor m_playerDesiredColor;
	TurtlePieceColor m_playerColor;
	std::string m_blackPlayerName;
	std::string m_redPlayerName;
	bool m_blackPlayerReady;
	bool m_redPlayerReady;

	std::vector<std::string> m_lobbyNames;
	std::vector<std::string> m_lobbyIds;
	std::vector<std::string> m_blackPlayerNames;
	std::vector<std::string> m_redPlayerNames;
};

typedef std::unique_ptr<CheckersMainMenuFrame> CheckersMainMenuFrameUniPtr;