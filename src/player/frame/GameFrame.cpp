#include "player/frame/GameFrame.hpp"

#include <QWidget>
#include <QFrame>
#include <QMouseEvent>
#include <QPaintEvent>
#include <QPointF>
#include <QStackedLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QTimer>

#include <cstdlib>
#include <ctime>
#include <functional>
#include <memory>
#include <string>
#include <chrono>
#include <vector>
#include <iostream>

#include "player/CheckersPlayerWindow.hpp"
#include "shared/CheckersConsts.hpp"
#include "shared/Hasher.hpp"
#include "player/StringLibrary.hpp"
#include "player/LanguageSelectorWidget.hpp"
#include "player/CheckersBoardRender.hpp"
#include "player/TileRender.hpp"
#include "player/TurtlePieceRender.hpp"
#include "player/TurtleGraveyard.hpp"
#include "player/HUD.hpp"
#include "player/DialogWidget.hpp"
#include "player/ChatBox.hpp"
#include "player/Parameters.hpp"

GameFrame::GameFrame(CheckersPlayerWindow *parentWindow)
	: QFrame(parentWindow, Qt::WindowFlags())
{
	m_playerWindow = parentWindow;

	setMouseTracking(true);

	m_redrawTimer = new QTimer(this);
	connect(m_redrawTimer, &QTimer::timeout, this, [this]()
			{ this->update(); });

	m_gameState = GameState::Connecting;
	m_winner = Winner::None;

	m_showingDialog = false;

	m_board = std::make_shared<CheckersBoardRender>();

	m_languageSelector = new LanguageSelectorWidget(this);

	m_hud = std::make_shared<HUD>();
	m_hud->setPiecesRemaining(m_board->getBlackTurtlesRemaining(), m_board->getRedTurtlesRemaining());
	m_hud->setGameState(GameState::Connecting);

	m_chatBox = new ChatBox(this, CHAT_IN_GAME_WIDTH, CHAT_IN_GAME_HEIGHT, [this](const std::string &chatMessages)
							{ this->reportPlayer(chatMessages); }, [this](const std::string &chatMessage)
							{ this->sendChatMessage(chatMessage); });
	m_chatBox->setGeometry(CHAT_BOX_IN_GAME_X, CHAT_BOX_IN_GAME_Y,
						   CHAT_BOX_IN_GAME_WIDTH, CHAT_BOX_IN_GAME_HEIGHT);

	auto buttonLayout = new QHBoxLayout(this);
	buttonLayout->setAlignment(Qt::AlignCenter);
	buttonLayout->setContentsMargins(GRAVEYARD_WIDTH,
									 HUD_HEIGHT + BOARD_HEIGHT,
									 GRAVEYARD_WIDTH + CHAT_BOX_IN_GAME_WIDTH,
									 0);

	m_offerDrawButton = new QPushButton();
	m_offerDrawButton->setText(StringLibrary::getTranslatedString("Offer Draw"));
	connect(m_offerDrawButton, &QPushButton::released, this, &GameFrame::handleOfferDrawButton);
	buttonLayout->addWidget(m_offerDrawButton);

	m_forfeitButton = new QPushButton();
	m_forfeitButton->setText(StringLibrary::getTranslatedString("Forfeit"));
	connect(m_forfeitButton, &QPushButton::released, this, &GameFrame::handleForfeitButton);
	buttonLayout->addWidget(m_forfeitButton);

	// Offer draw confirm dialog
	m_offerDrawConfirmDialog = new DialogWidget(this, BOARD_CENTER_X, BOARD_CENTER_Y,
												"",
												"Offer Draw", {[this]()
															   { this->handleAcceptOfferDrawButton(); }},
												"Cancel", {[this]()
														   { this->handleCancelOfferDrawButton(); }});

	// Offering draw waiting dialog
	m_offeringDrawDialog = new DialogWidget(this, BOARD_CENTER_X, BOARD_CENTER_Y,
											"Offering draw...",
											"", {[]() {}},
											"", {[]() {}});

	// Draw offered accept or decline dialog
	m_offeredDrawDialog = new DialogWidget(this, BOARD_CENTER_X, BOARD_CENTER_Y,
										   "",
										   "Accept Draw", {[this]()
														   { this->handleAcceptDrawButton(); }},
										   "Decline Draw", {[this]()
															{ this->handleDeclineDrawButton(); }});

	// Forfeit confirm dialog
	m_forfeitConfirmDialog = new DialogWidget(this, BOARD_CENTER_X, BOARD_CENTER_Y,
											  "",
											  "Forfeit", {[this]()
														  { this->handleForfeitConfirmButton(); }},
											  "Cancel", {[this]()
														 { this->handleForfeitCancelButton(); }});

	// Report player confirm dialog
	m_reportPlayerConfirmDialog = new DialogWidget(this, BOARD_CENTER_X, BOARD_CENTER_Y,
												   "Report player and leave lobby?",
												   "Report", {[this]()
															  { this->handleReportPlayerConfirmButton(); }},
												   "Cancel", {[this]()
															  { this->handleReportPlayerCancelButton(); }});

	// Leave game dialog
	m_leaveGameDialog = new DialogWidget(this, BOARD_CENTER_X, VICTORY_BUTTONS_Y,
										 "",
										 "Leave Game", {[this]()
														{ this->handleLeaveGameButton(); }},
										 "", {[]() {}});

    // Needs to be in front of everything
    m_languageSelector->raise();
}

GameFrame::~GameFrame()
{
}

void GameFrame::showEvent(QShowEvent *event)
{
	(void)event; // NO LINT

	m_redrawTimer->start(1000);

	m_languageSelector->setCurrentIndex(static_cast<int>(Parameters::getLanguage()));
	reloadStrings();
}

void GameFrame::hideEvent(QHideEvent *event)
{
	(void)event; // NO LINT

	m_redrawTimer->stop();
	displayDialog(false);
}

void GameFrame::connectedToGame()
{
	if (m_gameState == GameState::Connecting)
	{
		m_gameState = GameState::Connected;
	}

	displayDialog(false);

	auto playerColor = Parameters::getPlayerColor();

	// Create the board with all the tiles and pieces
	m_board = std::make_shared<CheckersBoardRender>();
	m_board->createBoard(playerColor);

	// Create the graveyards now that we know which side of the screen they are on
	m_blackPlayerGraveyard = std::make_shared<TurtleGraveyard>(TurtlePieceColor::Black, playerColor);
	m_redPlayerGraveyard = std::make_shared<TurtleGraveyard>(TurtlePieceColor::Red, playerColor);

	m_winner = Winner::None;
	m_hud = std::make_shared<HUD>();
	m_hud->setPiecesRemaining(m_board->getBlackTurtlesRemaining(), m_board->getRedTurtlesRemaining());
	m_hud->setPlayerColor(playerColor);
	m_hud->setGameState(m_gameState);
}

void GameFrame::requestedPieceMoveAccepted(bool moveAccepted)
{
	if (!moveAccepted) // If the move was rejected, clear everything and try again (This normally shouldn't happen)
	{
		m_board->clearSelections();
	}

	update();
}

void GameFrame::requestedReachableTiles(const std::vector<size_t> &reachableTileIndices)
{
	m_board->setReachableTiles(reachableTileIndices);

	update();
}

void GameFrame::reportPlayer(const std::string &chatMessages)
{
	m_reportingChatMessages = chatMessages;
	displayDialog(true, m_reportPlayerConfirmDialog);
}

void GameFrame::clearChat()
{
	m_chatBox->clear();
}

void GameFrame::addChatMessage(const std::string &playerName,
							   TurtlePieceColor playerColor,
							   const std::string &chatMessage,
							   std::chrono::time_point<std::chrono::system_clock> timeStamp)
{
	m_chatBox->addMessage(playerName, playerColor, chatMessage, timeStamp);
}

void GameFrame::sendChatMessage(const std::string &chatMessage)
{
	m_playerWindow->sendChatMessage(chatMessage);
}

void GameFrame::declaredWinner(Winner winner)
{
	m_winner = winner;
	m_hud->setWinner(m_winner);

	m_gameState = GameState::GameFinished;
	m_hud->setGameState(m_gameState);

	displayDialog(true, m_leaveGameDialog);

	update();
}

void GameFrame::gameStarted(GameState gameState, const std::vector<size_t> &movableTileIndices,
							size_t blackTimeRemainSec, size_t redTimeRemainSec)
{
	m_gameState = gameState;
	m_hud->setGameState(m_gameState);
	m_hud->setTimeRemaining(blackTimeRemainSec, redTimeRemainSec);
	m_hud->enableTimers(blackTimeRemainSec > 0u || redTimeRemainSec > 0u);

	if (isOwnTurn())
	{
		m_board->setMovablePieces(movableTileIndices);
	}

	update();
}

void GameFrame::updatedBoard(size_t sourceTileIndex, size_t destinationTileIndex, GameState gameState,
							 int slainPieceTileIndex, bool kingPiece, const std::vector<size_t> &movableTileIndices,
							 size_t blackTimeRemainSec, size_t redTimeRemainSec)
{
	m_board->clearSelections();
	m_board->clearMovedTiles();

	m_hud->setTimeRemaining(blackTimeRemainSec, redTimeRemainSec);

	m_board->moveTurtlePiece(sourceTileIndex, destinationTileIndex);

	GameState nextGameState = gameState;
	if (slainPieceTileIndex > -1)
	{
		m_board->slayTurtle(slainPieceTileIndex);
		// if (m_gameState != nextGameState) // If the game state switches on this update, the last turn finished
		{
			m_board->moveTurtlePiecesToGraveyard(m_blackPlayerGraveyard, m_redPlayerGraveyard);
		}
	}

	if (kingPiece)
	{
		m_board->kingPiece(destinationTileIndex);
	}

	m_gameState = nextGameState;
	if (isOwnTurn())
	{
		m_board->setMovablePieces(movableTileIndices);
	}

	m_hud->setGameState(m_gameState);
	m_hud->setPiecesRemaining(m_board->getBlackTurtlesRemaining(), m_board->getRedTurtlesRemaining());

	update();
}

void GameFrame::drawDeclined()
{
	displayDialog(false);
}

void GameFrame::drawOffered()
{
	if (m_gameState != GameState::GameFinished)
	{
		displayDialog(false);
		m_offeredDrawDialog->show();
	}
}

bool GameFrame::isOwnTurn()
{
	auto playerColor = Parameters::getPlayerColor();
	return ((playerColor == TurtlePieceColor::Black && m_gameState == GameState::BlackMove) ||
			(playerColor == TurtlePieceColor::Red && m_gameState == GameState::RedMove));
}

void GameFrame::mouseMoveEvent(QMouseEvent *event)
{
	if (!m_showingDialog)
	{
		auto playerColor = Parameters::getPlayerColor();
		switch (m_gameState)
		{
		case GameState::Connecting:
		{
			// Do nothing
		}
		break;
		case GameState::Connected:
		{
			// Do nothing
		}
		break;
		case GameState::BlackMove:
		{
			if (playerColor == TurtlePieceColor::Black)
			{
				if (m_board)
				{
					m_board->handleMouseMove(event);
				}
			}
			else
			{
				// Do nothing
			}
		}
		break;
		case GameState::RedMove:
		{
			if (playerColor == TurtlePieceColor::Red)
			{
				if (m_board)
				{
					m_board->handleMouseMove(event);
				}
			}
			else
			{
				// Do nothing
			}
		}
		case GameState::GameFinished:
		{
			// Do nothing
		}
		break;
		}
	}

	update();

	QFrame::mouseMoveEvent(event); // Ensure base class event handling
}

void GameFrame::mousePressEvent(QMouseEvent *event)
{
	if (!m_showingDialog)
	{
		bool clicked = false;
		auto playerColor = Parameters::getPlayerColor();
		if (event->button() == Qt::LeftButton)
		{
			switch (m_gameState)
			{
			case GameState::Connecting:
			{
				// Do nothing
			}
			break;
			case GameState::Connected:
			{
				// Do nothing
			}
			break;
			case GameState::BlackMove:
			{
				if (playerColor == TurtlePieceColor::Black)
				{
					if (m_board)
					{
						m_board->handleMouseClick(event);
						clicked = true;
					}
				}
				else
				{
					// Do nothing
				}
			}
			break;
			case GameState::RedMove:
			{
				if (playerColor == TurtlePieceColor::Red)
				{
					if (m_board)
					{
						m_board->handleMouseClick(event);
						clicked = true;
					}
				}
				else
				{
					// Do nothing
				}
			}
			case GameState::GameFinished:
			{
				// Do nothing
			}
			break;
			}
		}

		if (m_board && clicked)
		{
			if (m_board->getIsMoveSelected())
			{
				m_playerWindow->requestPieceMove(m_board->getSourceTileIndex(), m_board->getDestinationTileIndex());
			}
			else
			{
				int selectedPieceTileIndex = m_board->getSelectedPieceTileIndex();
				if (selectedPieceTileIndex > -1)
				{
					m_playerWindow->requestReachableTiles(selectedPieceTileIndex);
				}
			}
		}
	}

	update();

	QFrame::mousePressEvent(event); // Ensure base class event handling
}

void GameFrame::handleOfferDrawButton()
{
	displayDialog(true, m_offerDrawConfirmDialog);
}

void GameFrame::handleAcceptOfferDrawButton()
{
	displayDialog(true, m_offeringDrawDialog);
	m_playerWindow->offerDraw();
}

void GameFrame::handleCancelOfferDrawButton()
{
	displayDialog(false);
}

void GameFrame::handleAcceptDrawButton()
{
	displayDialog(false);
	m_playerWindow->offerDraw();
}

void GameFrame::handleDeclineDrawButton()
{
	displayDialog(false);
	m_playerWindow->declineDraw();
}

void GameFrame::handleForfeitButton()
{
	displayDialog(true, m_forfeitConfirmDialog);
}

void GameFrame::handleForfeitConfirmButton()
{
	displayDialog(false);
	m_playerWindow->forfeit();
}

void GameFrame::handleForfeitCancelButton()
{
	displayDialog(false);
}

void GameFrame::handleReportPlayerConfirmButton()
{
	displayDialog(false);
	m_playerWindow->reportPlayer(m_reportingChatMessages);
	m_reportingChatMessages.clear();
	m_playerWindow->leaveLobby();
	m_playerWindow->moveToMainMenuFrame();
}

void GameFrame::handleReportPlayerCancelButton()
{
	displayDialog(false);
	m_reportingChatMessages.clear();
}

void GameFrame::handleLeaveGameButton()
{
	displayDialog(false);
	m_playerWindow->leaveLobby();
	m_playerWindow->moveToMainMenuFrame();
}

void GameFrame::handleOfferRematchButton()
{
	// TODO: Add button to end game dialog that creates a lobby and holds the position open for the other player
}

uint64_t GameFrame::getBoardHash() const
{
	return std::hash<CheckersBoardRenderPtr>{}(m_board);
}

void GameFrame::resyncBoard(uint64_t blackTimeRemainingSeconds,
							uint64_t redTimeRemainingSeconds,
							uint64_t gameState,
							uint64_t blackPiecesRemaining,
							uint64_t redPiecesRemaining,
							std::vector<std::string> turtlePieceNamePerTile,
							std::vector<uint64_t> turtlePieceColorPerTile,
							std::vector<bool> turtlePieceIsKingedPerTile)
{
	// Update the HUD
	m_hud->setTimeRemaining(blackTimeRemainingSeconds, redTimeRemainingSeconds);
	m_hud->setPiecesRemaining(blackPiecesRemaining, redPiecesRemaining);
	m_gameState = static_cast<GameState>(gameState);
	m_hud->setGameState(m_gameState);

	// Update the board
	m_board->resyncBoard(turtlePieceNamePerTile,
						 turtlePieceColorPerTile,
						 turtlePieceIsKingedPerTile,
						 m_blackPlayerGraveyard,
						 m_redPlayerGraveyard);
}

void GameFrame::displayDialog(bool dialogDisplayed, DialogWidget *dialog)
{
	// Enable or disable everything as needed
	m_showingDialog = dialogDisplayed;
	m_chatBox->setEnabled(!m_showingDialog);
	m_offerDrawButton->setEnabled(!m_showingDialog);
	m_forfeitButton->setEnabled(!m_showingDialog);
	m_offerDrawConfirmDialog->hide();
	m_offeringDrawDialog->hide();
	m_offeredDrawDialog->hide();
	m_forfeitConfirmDialog->hide();
	m_leaveGameDialog->hide();
	m_reportPlayerConfirmDialog->hide();
	if (dialogDisplayed && dialog)
	{
		dialog->show();
	}
}

void GameFrame::reloadStrings()
{
	// HUD reloads strings when it redraws itself every frame

	m_chatBox->reloadStrings();

	m_offerDrawButton->setText(StringLibrary::getTranslatedString("Offer Draw"));
	m_forfeitButton->setText(StringLibrary::getTranslatedString("Forfeit"));

	m_offerDrawConfirmDialog->reloadStrings();
	m_offeringDrawDialog->reloadStrings();
	m_offeredDrawDialog->reloadStrings();
	m_forfeitConfirmDialog->reloadStrings();
	m_leaveGameDialog->reloadStrings();
	m_reportPlayerConfirmDialog->reloadStrings();
}

void GameFrame::paintEvent(QPaintEvent *event)
{
	(void)event; // NO LINT
	QPainter painter(this);

	// Fill the background
	QRgb backgroundColor = qRgb(BG_RGB[0], BG_RGB[1], BG_RGB[2]);
	painter.fillRect(0, 0, width(), height(), backgroundColor);

	if (Parameters::getPlayerColor() == TurtlePieceColor::None)
	{
		return; // If we haven't connected yet, don't draw a board
	}

	m_board->paint(painter);

	// Draw the graveyards and any turtles they contain
	m_blackPlayerGraveyard->paint(painter);
	m_redPlayerGraveyard->paint(painter);

	// Draw the HUD
	m_hud->paint(painter);
}