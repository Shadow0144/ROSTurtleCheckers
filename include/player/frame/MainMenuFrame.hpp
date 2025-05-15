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

class MainMenuFrame : public QFrame
{
	Q_OBJECT
public:
	MainMenuFrame(CheckersPlayerWindow *parentWindow);
	~MainMenuFrame();

public slots:
	void validatePlayerNameText(const QString &playerName);

private:
	void handleCreateLobbyButton();
	void handleJoinLobbyButton();
	void handleQuitButton();

	CheckersPlayerWindow *m_playerWindow;

	QLineEdit *m_playerNameLineEdit;

	QPushButton *m_createLobbyButton;
	QPushButton *m_joinLobbyButton;
};

typedef std::unique_ptr<MainMenuFrame> MainMenuFrameUniPtr;