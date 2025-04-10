#include "player/CheckersGameFrame.hpp"

#include <QFrame>
#include <QMouseEvent>
#include <QPaintEvent>
#include <QPointF>

#include <cstdlib>
#include <ctime>
#include <functional>
#include <memory>
#include <string>
#include <chrono>
#include <vector>
#include <iostream>

#include "player/CheckersPlayerWindow.hpp"

CheckersGameFrame::CheckersGameFrame(
	CheckersPlayerWindow *parentWindow,
	const std::string &playerName)
	: QFrame(parentWindow, Qt::WindowFlags())
{
	m_playerWindow = parentWindow;
	m_playerName = playerName;

	m_gameState = GameState::Connecting;
	m_playerColor = TurtlePieceColor::None;
	m_winner = Winner::None;

	setMouseTracking(true);

	m_board = std::make_shared<CheckersBoardRender>();

	m_hud = std::make_shared<HUD>();
	m_hud->setPiecesRemaining(m_board->getBlackTurtlesRemaining(), m_board->getRedTurtlesRemaining());
	m_hud->setGameState(GameState::Connecting);
}

CheckersGameFrame::~CheckersGameFrame()
{
}

const std::string &CheckersGameFrame::getLobbyName() const
{
	return m_lobbyName;
}

void CheckersGameFrame::setLobbyName(const std::string &lobbyName)
{
	m_lobbyName = lobbyName;
}

void CheckersGameFrame::connectedToGame(const std::string &lobbyName, TurtlePieceColor playerColor)
{
	m_lobbyName = lobbyName;
	m_playerColor = playerColor;

	if (m_gameState == GameState::Connecting)
	{
		m_gameState = GameState::Connected;
	}

	// Create the board with all the tiles and pieces
	m_board->createBoard(m_playerColor);

	// Create the graveyards now that we know which side of the screen they are on
	m_blackPlayerGraveyard = std::make_shared<TurtleGraveyard>(TurtlePieceColor::Black, m_playerColor);
	m_redPlayerGraveyard = std::make_shared<TurtleGraveyard>(TurtlePieceColor::Red, m_playerColor);

	m_hud->setPlayerColor(m_playerColor);
	m_hud->setGameState(m_gameState);
}

void CheckersGameFrame::requestedPieceMoveAccepted(bool moveAccepted)
{
	if (!moveAccepted) // If the move was rejected, clear everything and try again (This normally shouldn't happen)
	{
		m_board->clearSelections();
	}

	update();
}

void CheckersGameFrame::requestedReachableTiles(const std::vector<size_t> &reachableTileIndices)
{
	m_board->setReachableTiles(reachableTileIndices);

	update();
}

void CheckersGameFrame::declaredWinner(Winner winner)
{
	m_winner = winner;
	m_hud->setWinner(m_winner);

	update();
}

void CheckersGameFrame::gameStarted(GameState gameState, const std::vector<size_t> &movableTileIndices)
{
	m_gameState = gameState;
	m_hud->setGameState(m_gameState);

	if (isOwnTurn())
	{
		m_board->setMovablePieces(movableTileIndices);
	}

	update();
}

void CheckersGameFrame::updatedBoard(size_t sourceTileIndex, size_t destinationTileIndex, GameState gameState,
									 int slainPieceTileIndex, bool kingPiece, const std::vector<size_t> &movableTileIndices)
{
	m_board->clearSelections();
	m_board->clearMovedTiles();

	m_board->moveTurtlePiece(sourceTileIndex, destinationTileIndex);

	GameState nextGameState = gameState;
	if (slainPieceTileIndex > -1)
	{
		m_board->slayTurtle(slainPieceTileIndex);
		if (m_gameState != nextGameState) // If the game state switches on this update, the last turn finished
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

bool CheckersGameFrame::isOwnTurn()
{
	return ((m_playerColor == TurtlePieceColor::Black && m_gameState == GameState::BlackMove) ||
			(m_playerColor == TurtlePieceColor::Red && m_gameState == GameState::RedMove));
}

void CheckersGameFrame::mouseMoveEvent(QMouseEvent *event)
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
		if (m_playerColor == TurtlePieceColor::Black)
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
		if (m_playerColor == TurtlePieceColor::Red)
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

	update();

	QFrame::mouseMoveEvent(event); // Ensure base class event handling
}

void CheckersGameFrame::mousePressEvent(QMouseEvent *event)
{
	bool clicked = false;
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
			if (m_playerColor == TurtlePieceColor::Black)
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
			if (m_playerColor == TurtlePieceColor::Red)
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

	update();

	QFrame::mousePressEvent(event); // Ensure base class event handling
}

void CheckersGameFrame::paintEvent(QPaintEvent *event)
{
	(void)event; // NO LINT
	QPainter painter(this);

	// Fill the background
	QRgb backgroundColor = qRgb(BG_RGB[0], BG_RGB[1], BG_RGB[2]);
	painter.fillRect(0, 0, width(), height(), backgroundColor);

	if (m_playerColor == TurtlePieceColor::None)
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