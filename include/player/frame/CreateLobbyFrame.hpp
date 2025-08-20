#pragma once

// #ifndef Q_MOC_RUN			  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
//  NO LINT
// #endif

#include <QFrame>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QRadioButton>
#include <QString>

#include <memory>

#include "shared/CheckersConsts.hpp"
#include "player/LanguageSelectorWidget.hpp"
#include "player/TitleWidget.hpp"

class CheckersPlayerWindow;

class CreateLobbyFrame : public QFrame
{
	Q_OBJECT
public:
	CreateLobbyFrame(CheckersPlayerWindow *parentWindow);
	~CreateLobbyFrame();

	void showEvent(QShowEvent *event) override;

	void reloadStrings();

public slots:
	void validateLobbyNameText(const QString &lobbyName);
	void validatePasswordText(const QString &lobbyPassword);

private:
	void handleCancelButton();
	void handleCreateLobbyButton();

	void onBlackTurtleToggled(bool checked);
	void onRandomTurtleToggled(bool checked);
	void onRedTurtleToggled(bool checked);

	CheckersPlayerWindow *m_playerWindow;

	LanguageSelectorWidget *m_languageSelector;

	TitleWidget *m_titleWidget;

	QLabel *m_lobbyNameLabel;
	QLabel *m_lobbyPasswordLabel;

	QLineEdit *m_lobbyNameLineEdit;
	QLineEdit *m_lobbyPasswordLineEdit;

	QPushButton *m_createLobbyButton;
	QPushButton *m_cancelCreateLobbyButton;

	QRadioButton *m_blackRadioButton;
	QRadioButton *m_randomRadioButton;
	QRadioButton *m_redRadioButton;

	TurtlePieceColor m_playerDesiredColor;
};

typedef std::unique_ptr<CreateLobbyFrame> CreateLobbyFrameUniPtr;