#pragma once

// #ifndef Q_MOC_RUN			  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
//  NO LINT
// #endif

#include <QWidget>
#include <QFrame>
#include <QImage>
#include <QPainter>
#include <QPaintEvent>
#include <QVector>
#include <QStackedLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>

#include <memory>
#include <string>
#include <vector>

#include "shared/CheckersConsts.hpp"
#include "player/CheckersBoardRender.hpp"
#include "player/TileRender.hpp"
#include "player/TurtlePieceRender.hpp"
#include "player/TurtleGraveyard.hpp"
#include "player/HUD.hpp"

class CheckersPlayerWindow;

class CheckersGameFrame : public QFrame
{
	Q_OBJECT
public:
	CheckersGameFrame(
		CheckersPlayerWindow *parentWindow,
		const std::string &playerName);
	~CheckersGameFrame();

	void connectedToGame(const std::string &lobbyName, const std::string &lobbyId, TurtlePieceColor playerColor);
	void requestedPieceMoveAccepted(bool moveAccepted);
	void requestedReachableTiles(const std::vector<size_t> &reachableTileIndices);
	void declaredWinner(Winner winner);
	void gameStarted(GameState gameState, const std::vector<size_t> &movableTileIndices);
	void updatedBoard(size_t sourceTileIndex, size_t destinationTileIndex, GameState gameState,
					  int slainPieceTileIndex, bool kingPiece, const std::vector<size_t> &movableTileIndices);
	void drawDeclined();
	void drawOffered();

protected:
	void mouseMoveEvent(QMouseEvent *event) override;
	void mousePressEvent(QMouseEvent *event) override;

	void paintEvent(QPaintEvent *event) override;

private:
	bool isOwnTurn();

	void clearSelections();

	void handleOfferRematchButton();
	void handleOfferDrawButton();
	void handleDeclineDrawButton();
	void handleForfitButton();
	void handleOfferDrawConfirmButton();
	void handleOfferDrawCancelButton();
	void handleForfitConfirmButton();
	void handleForfitCancelButton();
	void handleLeaveGameButton();

	CheckersPlayerWindow *m_playerWindow;

	QPushButton *m_offerDrawButton;
	QPushButton *m_forfitButton;

	QWidget *m_offerDrawConfirmLayoutWidget;
	QWidget *m_forfitConfirmLayoutWidget;
	QWidget *m_drawOfferedLayoutWidget;
	QWidget *m_offeringDrawLayoutWidget;

	QWidget *m_leaveGameLayoutWidget;
	QPushButton *m_leaveGameButton;

	std::string m_playerName;
	std::string m_lobbyName;
	std::string m_lobbyId;
	TurtlePieceColor m_playerColor;

	GameState m_gameState;

	Winner m_winner;

	CheckersBoardRenderPtr m_board;
	TurtleGraveyardPtr m_blackPlayerGraveyard; // Black player's graveyard containing the slain red pieces
	TurtleGraveyardPtr m_redPlayerGraveyard;   // Red player's graveyard containing the slain black pieces
	HUDPtr m_hud;

	bool m_showingDialog;
};

typedef std::unique_ptr<CheckersGameFrame> CheckersGameFrameUniPtr;