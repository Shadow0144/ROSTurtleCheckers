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

class CreateLobbyFrame : public QFrame
{
	Q_OBJECT
public:
	CreateLobbyFrame(CheckersPlayerWindow *parentWindow);
	~CreateLobbyFrame();
	
	void showEvent(QShowEvent* event) override;

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

	QLineEdit *m_lobbyNameLineEdit;
	QLineEdit *m_lobbyPasswordLineEdit;

	QPushButton *m_createLobbyButton;

	QRadioButton *m_blackRadioButton;
	QRadioButton *m_randomRadioButton;
	QRadioButton *m_redRadioButton;

	TurtlePieceColor m_playerDesiredColor;
};

typedef std::unique_ptr<CreateLobbyFrame> CreateLobbyFrameUniPtr;