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

	const std::string &getPlayerName() const;
	const std::string &getLobbyName() const;
	const std::string &getLobbyId() const;

	void setPlayerName(const std::string &playerName);

	void setPasswordIncorrect();

public slots:
	void onEnterLobbyPasswordTextChanged(const QString &lobbyPassword);

private:
	void handleCancelEnterLobbyPasswordButton();
	void handleConfirmPassword(size_t lobbyIndex);

	CheckersPlayerWindow *m_playerWindow;

	QLineEdit *m_enterLobbyPasswordLineEdit;

	QPushButton *m_createLobbyButton;
	QPushButton *m_commitCreateLobbyButton;
	QPushButton *m_joinLobbyButton;
	QPushButton *m_commitJoinLobbyButton;
	QLabel *m_passwordIncorrectLabel;

	std::string m_playerName;
	std::string m_lobbyName;
	std::string m_lobbyId;
};

typedef std::unique_ptr<LoginAccountFrame> LoginAccountFrameUniPtr;