#pragma once

// #ifndef Q_MOC_RUN			  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
//  NO LINT
// #endif

#include <QFrame>
#include <QWidget>
#include <QImage>
#include <QPainter>
#include <QPaintEvent>
#include <QVector>
#include <QStackedLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QTimer>

#include <memory>
#include <string>
#include <vector>

#include "shared/CheckersConsts.hpp"
#include "player/CheckersBoardRender.hpp"
#include "player/TileRender.hpp"
#include "player/TurtlePieceRender.hpp"
#include "player/TurtleGraveyard.hpp"
#include "player/HUD.hpp"
#include "player/DialogWidget.hpp"
#include "player/ChatBox.hpp"

class CheckersPlayerWindow;

class GameFrame : public QFrame
{
	Q_OBJECT
public:
	GameFrame(
		CheckersPlayerWindow *parentWindow);
	~GameFrame();

	void showEvent(QShowEvent *event) override;
	void hideEvent(QHideEvent *event) override;

	void connectedToGame();

	// Reply from server
	void addChatMessage(const std::string &playerName,
						TurtlePieceColor playerColor,
						const std::string &chatMessage,
						std::chrono::time_point<std::chrono::system_clock> timeStamp);

	void requestedPieceMoveAccepted(bool moveAccepted);
	void requestedReachableTiles(const std::vector<size_t> &reachableTileIndices);
	void declaredWinner(Winner winner);
	void gameStarted(GameState gameState, const std::vector<size_t> &movableTileIndices,
					 size_t blackTimeRemainSec, size_t redTimeRemainSec);
	void updatedBoard(size_t sourceTileIndex, size_t destinationTileIndex, GameState gameState,
					  int slainPieceTileIndex, bool kingPiece, const std::vector<size_t> &movableTileIndices,
					  size_t blackTimeRemainSec, size_t redTimeRemainSec);
	void drawDeclined();
	void drawOffered();

	void clearChat();

	// Send to server
	void reportPlayer(const std::string &chatMessages); // Creates a popup
	void sendChatMessage(const std::string &chatMessage);

	uint64_t getBoardHash() const;
	void resyncBoard(uint64_t blackTimeRemainingSeconds,
					 uint64_t redTimeRemainingSeconds,
					 uint64_t gameState,
					 uint64_t blackPiecesRemaining,
					 uint64_t redPiecesRemaining,
					 std::vector<std::string> turtlePieceNamePerTile,
					 std::vector<uint64_t> turtlePieceColorPerTile,
					 std::vector<bool> turtlePieceIsKingedPerTile);

protected:
	void mouseMoveEvent(QMouseEvent *event) override;
	void mousePressEvent(QMouseEvent *event) override;

	void paintEvent(QPaintEvent *event) override;

private:
	bool isOwnTurn();

	void clearSelections();

	void handleOfferDrawButton();		// Creates a confirmation popup
	void handleAcceptOfferDrawButton(); // Commits to the draw offer
	void handleCancelOfferDrawButton(); // Cancels the popup

	void handleAcceptDrawButton();	// Accepts an incoming draw offer
	void handleDeclineDrawButton(); // Declines a draw offer

	void handleForfeitButton();		  // Creates a confirmation popup
	void handleForfeitConfirmButton(); // Commits to the forfeit
	void handleForfeitCancelButton();  // Cancels the popup

	void handleReportPlayerConfirmButton(); // Commits to the report
	void handleReportPlayerCancelButton();	// Cancels the popup

	void handleLeaveGameButton();	 // Leaves the lobby
	void handleOfferRematchButton(); // Creates a new lobby

	void displayDialog(bool dialogDisplayed, DialogWidget *dialog = nullptr);

	CheckersPlayerWindow *m_playerWindow;

	QWidget *m_gameWidget;

	QPushButton *m_offerDrawButton;
	QPushButton *m_forfeitButton;

	bool m_showingDialog;
	DialogWidget *m_offerDrawConfirmDialog;
	DialogWidget *m_offeringDrawDialog;
	DialogWidget *m_offeredDrawDialog;
	DialogWidget *m_forfeitConfirmDialog;
	DialogWidget *m_reportPlayerConfirmDialog;
	DialogWidget *m_leaveGameDialog;

	GameState m_gameState;

	Winner m_winner;

	CheckersBoardRenderPtr m_board;
	TurtleGraveyardPtr m_blackPlayerGraveyard; // Black player's graveyard containing the slain red pieces
	TurtleGraveyardPtr m_redPlayerGraveyard;   // Red player's graveyard containing the slain black pieces
	HUDPtr m_hud;
	ChatBox *m_chatBox;

	std::string m_reportingChatMessages;

	QTimer *m_redrawTimer;
};

typedef std::unique_ptr<GameFrame> GameFrameUniPtr;