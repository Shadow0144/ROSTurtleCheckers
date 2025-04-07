#pragma once

// #ifndef Q_MOC_RUN			  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
//  NO LINT
// #endif

#include "CheckersConsts.hpp"
#include "CheckersBoardRender.hpp"
#include "TileRender.hpp"
#include "TurtlePieceRender.hpp"
#include "TurtleGraveyard.hpp"
#include "HUD.hpp"

#include <QFrame>
#include <QImage>
#include <QPainter>
#include <QPaintEvent>
#include <QVector>

#include <string>
#include <vector>

class CheckersPlayerWindow;

class CheckersGameFrame : public QFrame
{
	Q_OBJECT
public:
	CheckersGameFrame(
		const std::weak_ptr<CheckersPlayerWindow> &playerApp,
		const std::string &playerName,
		QWidget *parent = 0,
		Qt::WindowFlags windowFlags = Qt::WindowFlags());
	~CheckersGameFrame();

	const std::string &getLobbyName() const;

	void connectedToGame(const std::string &lobbyName, TurtlePieceColor playerColor);
	void requestedPieceMoveAccepted(bool moveAccepted);
	void requestedReachableTiles(const std::vector<size_t> &reachableTileIndices);
	void declaredWinner(Winner winner);
	void gameStarted(GameState gameState, const std::vector<size_t> &movableTileIndices);
	void updatedBoard(size_t sourceTileIndex, size_t destinationTileIndex, GameState gameState,
					  int slainPieceTileIndex, bool kingPiece, const std::vector<size_t> &movableTileIndices);

protected:
	void mouseMoveEvent(QMouseEvent *event) override;
	void mousePressEvent(QMouseEvent *event) override;

	void paintEvent(QPaintEvent *event) override;

private:
	bool isOwnTurn();

	void clearSelections();

	std::weak_ptr<CheckersPlayerWindow> m_playerApp;

	std::string m_lobbyName;
	std::string m_playerName;
	TurtlePieceColor m_playerColor;

	GameState m_gameState;

	Winner m_winner;

	CheckersBoardRenderPtr m_board;
	TurtleGraveyardPtr m_blackPlayerGraveyard; // Black player's graveyard containing the slain red pieces
	TurtleGraveyardPtr m_redPlayerGraveyard;   // Red player's graveyard containing the slain black pieces
	HUDPtr m_hud;
};

typedef std::unique_ptr<CheckersGameFrame> CheckersGameFrameUniPtr;
typedef std::shared_ptr<CheckersGameFrame> CheckersGameFrameShrPtr;