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

#include "shared/CheckersConsts.hpp"
#include "player/Parameters.hpp"
#include "player/CheckersPlayerWindow.hpp"

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

	m_hud = std::make_shared<HUD>();
	m_hud->setPiecesRemaining(m_board->getBlackTurtlesRemaining(), m_board->getRedTurtlesRemaining());
	m_hud->setGameState(GameState::Connecting);

	auto buttonLayout = new QHBoxLayout(this);
	buttonLayout->setAlignment(Qt::AlignCenter);
	buttonLayout->setContentsMargins(GRAVEYARD_WIDTH,
									 HUD_HEIGHT + BOARD_HEIGHT,
									 GRAVEYARD_WIDTH + CHAT_WIDTH,
									 0);

	m_offerDrawButton = new QPushButton(this);
	m_offerDrawButton->setText("Offer Draw");
	connect(m_offerDrawButton, &QPushButton::released, this, &GameFrame::handleOfferDrawButton);
	buttonLayout->addWidget(m_offerDrawButton);

	m_forfitButton = new QPushButton(this);
	m_forfitButton->setText("Forfit");
	connect(m_forfitButton, &QPushButton::released, this, &GameFrame::handleForfitButton);
	buttonLayout->addWidget(m_forfitButton);

	// Offer draw confirm dialog

	m_offerDrawConfirmLayoutWidget = new QWidget(this);
	auto offerDrawConfirmLayout = new QHBoxLayout(m_offerDrawConfirmLayoutWidget);

	auto offerDrawConfirmButton = new QPushButton(this);
	offerDrawConfirmButton->setText("Offer Draw");
	connect(offerDrawConfirmButton, &QPushButton::released, this, &GameFrame::handleOfferDrawConfirmButton);
	offerDrawConfirmLayout->addWidget(offerDrawConfirmButton);

	auto offerDrawCancelButton = new QPushButton(this);
	offerDrawCancelButton->setText("Cancel");
	connect(offerDrawCancelButton, &QPushButton::released, this, &GameFrame::handleOfferDrawCancelButton);
	offerDrawConfirmLayout->addWidget(offerDrawCancelButton);

	offerDrawConfirmLayout->invalidate();
	offerDrawConfirmLayout->activate();
	m_offerDrawConfirmLayoutWidget->move(BOARD_CENTER_X - (m_offerDrawConfirmLayoutWidget->width() / 2),
										 BOARD_CENTER_Y - (m_offerDrawConfirmLayoutWidget->height() / 2));
	m_offerDrawConfirmLayoutWidget->hide();

	// Offering draw waiting dialog

	m_offeringDrawLayoutWidget = new QWidget(this);
	auto offeringDrawLayout = new QHBoxLayout(m_offeringDrawLayoutWidget);

	auto offeringDrawLabel = new QLabel(this);
	offeringDrawLabel->setText("Offering draw...");
	offeringDrawLayout->addWidget(offeringDrawLabel);

	offeringDrawLayout->invalidate();
	offeringDrawLayout->activate();
	m_offeringDrawLayoutWidget->move(BOARD_CENTER_X - (m_offeringDrawLayoutWidget->width() / 2),
									 BOARD_CENTER_Y - (m_offeringDrawLayoutWidget->height() / 2));
	m_offeringDrawLayoutWidget->hide();

	// Draw offered accept or decline dialog

	m_drawOfferedLayoutWidget = new QWidget(this);
	auto drawOfferedLayout = new QHBoxLayout(m_drawOfferedLayoutWidget);

	auto drawOfferedAcceptButton = new QPushButton(this);
	drawOfferedAcceptButton->setText("Accept Draw");
	connect(drawOfferedAcceptButton, &QPushButton::released, this, &GameFrame::handleOfferDrawConfirmButton);
	drawOfferedLayout->addWidget(drawOfferedAcceptButton);

	auto drawOfferedDeclineButton = new QPushButton(this);
	drawOfferedDeclineButton->setText("Decline Draw");
	connect(drawOfferedDeclineButton, &QPushButton::released, this, &GameFrame::handleDeclineDrawButton);
	drawOfferedLayout->addWidget(drawOfferedDeclineButton);

	drawOfferedLayout->invalidate();
	drawOfferedLayout->activate();
	m_drawOfferedLayoutWidget->move(BOARD_CENTER_X - (m_offerDrawConfirmLayoutWidget->width() / 2),
									BOARD_CENTER_Y - (m_offerDrawConfirmLayoutWidget->height() / 2));
	m_drawOfferedLayoutWidget->hide();

	// Forfit confirm dialog

	m_forfitConfirmLayoutWidget = new QWidget(this);
	auto forfitConfirmLayout = new QHBoxLayout(m_forfitConfirmLayoutWidget);

	auto forfitConfirmButton = new QPushButton(this);
	forfitConfirmButton->setText("Forfit");
	connect(forfitConfirmButton, &QPushButton::released, this, &GameFrame::handleForfitConfirmButton);
	forfitConfirmLayout->addWidget(forfitConfirmButton);

	auto forfitCancelButton = new QPushButton(this);
	forfitCancelButton->setText("Cancel");
	connect(forfitCancelButton, &QPushButton::released, this, &GameFrame::handleForfitCancelButton);
	forfitConfirmLayout->addWidget(forfitCancelButton);

	forfitConfirmLayout->invalidate();
	forfitConfirmLayout->activate();
	m_forfitConfirmLayoutWidget->move(BOARD_CENTER_X - (m_offerDrawConfirmLayoutWidget->width() / 2),
									  BOARD_CENTER_Y - (m_offerDrawConfirmLayoutWidget->height() / 2));
	m_forfitConfirmLayoutWidget->hide();

	// Leave game dialog

	m_leaveGameLayoutWidget = new QWidget(this);
	auto leaveGameLayout = new QHBoxLayout(m_leaveGameLayoutWidget);

	m_leaveGameButton = new QPushButton(this);
	m_leaveGameButton->setText("Leave Game");
	connect(m_leaveGameButton, &QPushButton::released, this, &GameFrame::handleLeaveGameButton);
	leaveGameLayout->addWidget(m_leaveGameButton);

	leaveGameLayout->invalidate();
	leaveGameLayout->activate();
	m_leaveGameLayoutWidget->move(BOARD_CENTER_X - (m_leaveGameLayoutWidget->width() / 2), VICTORY_BUTTONS_Y);
	m_leaveGameLayoutWidget->hide();
}

GameFrame::~GameFrame()
{
}

void GameFrame::showEvent(QShowEvent *event)
{
	(void)event; // NO LINT

	m_redrawTimer->start(1000);
}

void GameFrame::hideEvent(QHideEvent *event)
{
	(void)event; // NO LINT

	m_redrawTimer->stop();
}

void GameFrame::connectedToGame()
{
	if (m_gameState == GameState::Connecting)
	{
		m_gameState = GameState::Connected;
	}

	m_showingDialog = false;
	m_offeringDrawLayoutWidget->hide();
	m_drawOfferedLayoutWidget->hide();
	m_offerDrawConfirmLayoutWidget->hide();
	m_forfitConfirmLayoutWidget->hide();
	m_leaveGameLayoutWidget->hide();
	m_offerDrawButton->setEnabled(true);
	m_forfitButton->setEnabled(true);

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

void GameFrame::declaredWinner(Winner winner)
{
	m_winner = winner;
	m_hud->setWinner(m_winner);

	m_gameState = GameState::GameFinished;
	m_hud->setGameState(m_gameState);

	m_showingDialog = false;

	m_offeringDrawLayoutWidget->hide();
	m_drawOfferedLayoutWidget->hide();
	m_offerDrawConfirmLayoutWidget->hide();
	m_offerDrawButton->setEnabled(false);

	m_forfitConfirmLayoutWidget->hide();
	m_forfitButton->setEnabled(false);

	m_leaveGameLayoutWidget->show();

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
	m_showingDialog = false;
	m_offeringDrawLayoutWidget->hide();
	m_drawOfferedLayoutWidget->hide();
	m_offerDrawButton->setEnabled(true);
	m_forfitButton->setEnabled(true);

	update();
}

void GameFrame::drawOffered()
{
	if (m_gameState != GameState::GameFinished)
	{
		m_showingDialog = true;
		m_offerDrawConfirmLayoutWidget->hide();
		m_drawOfferedLayoutWidget->show();
		m_offerDrawButton->setEnabled(false);
		m_forfitButton->setEnabled(false);

		update();
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

void GameFrame::handleOfferRematchButton()
{
}

void GameFrame::handleOfferDrawButton()
{
	m_showingDialog = true;
	m_offerDrawConfirmLayoutWidget->show();
	m_offerDrawButton->setEnabled(false);
	m_forfitButton->setEnabled(false);

	update();
}

void GameFrame::handleDeclineDrawButton()
{
	m_showingDialog = false;
	m_offerDrawConfirmLayoutWidget->hide();
	m_offerDrawButton->setEnabled(true);
	m_forfitButton->setEnabled(true);
	m_playerWindow->declineDraw();

	update();
}

void GameFrame::handleForfitButton()
{
	m_showingDialog = true;
	m_forfitConfirmLayoutWidget->show();
	m_offerDrawButton->setEnabled(false);
	m_forfitButton->setEnabled(false);

	update();
}

void GameFrame::handleOfferDrawConfirmButton()
{
	m_playerWindow->offerDraw();
	m_offerDrawConfirmLayoutWidget->hide();
	m_offeringDrawLayoutWidget->show();

	update();
}

void GameFrame::handleOfferDrawCancelButton()
{
	m_showingDialog = false;
	m_offerDrawConfirmLayoutWidget->hide();
	m_offerDrawButton->setEnabled(true);
	m_forfitButton->setEnabled(true);

	update();
}

void GameFrame::handleForfitConfirmButton()
{
	m_playerWindow->forfit();
}

void GameFrame::handleForfitCancelButton()
{
	m_showingDialog = false;
	m_forfitConfirmLayoutWidget->hide();
	m_offerDrawButton->setEnabled(true);
	m_forfitButton->setEnabled(true);

	update();
}

void GameFrame::handleLeaveGameButton()
{
	m_playerWindow->leaveLobby();
	m_playerWindow->moveToMainMenuFrame();
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