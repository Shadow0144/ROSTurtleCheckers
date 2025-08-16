#pragma once

// #ifndef Q_MOC_RUN			  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
//  NO LINT
// #endif

#include <QFrame>
#include <QLineEdit>
#include <QLabel>
#include <QPushButton>
#include <QString>

#include <memory>

class CheckersPlayerWindow;

class LobbyPasswordFrame : public QFrame
{
	Q_OBJECT
public:
	LobbyPasswordFrame(CheckersPlayerWindow *parentWindow);
	~LobbyPasswordFrame();
	
	void showEvent(QShowEvent* event) override;

	void setPasswordIncorrect();

public slots:
	void validatePasswordText(const QString &lobbyPassword);

private:
	void handleCancelButton();
	void handleConfirmPasswordButton();

	CheckersPlayerWindow *m_playerWindow;

	QLineEdit *m_lobbyPasswordLineEdit;

	QPushButton *m_confirmPasswordButton;

	QLabel *m_lobbyNameLabel;
	QLabel *m_lobbyIdLabel;
	QLabel *m_passwordIncorrectLabel;
};

typedef std::unique_ptr<LobbyPasswordFrame> LobbyPasswordFrameUniPtr;