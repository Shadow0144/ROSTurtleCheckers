#include "CheckersGameWindow.hpp"

#include <QPointF>
#include <QTimer>

#include <cstdlib>
#include <ctime>
#include <functional>
#include <string>
#include <chrono>
#include <vector>

#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rclcpp/rclcpp.hpp"

#include "turtle_checkers_interfaces/srv/connect_to_game.hpp"
#include "turtle_checkers_interfaces/srv/request_piece_move.hpp"
#include "turtle_checkers_interfaces/srv/request_reachable_tiles.hpp"
#include "turtle_checkers_interfaces/msg/declare_winner.hpp"
#include "turtle_checkers_interfaces/msg/game_start.hpp"
#include "turtle_checkers_interfaces/msg/player_ready.hpp"
#include "turtle_checkers_interfaces/msg/update_board.hpp"

using std::placeholders::_1;

CheckersGameWindow::CheckersGameWindow(
	rclcpp::Node::SharedPtr &nodeHandle,
	const std::string &playerName,
	QWidget *parent,
	Qt::WindowFlags windowFlags)
	: QFrame(parent, windowFlags),
	  m_playerName(playerName)
{
	m_nodeHandle = nodeHandle;

	m_gameState = GameState::Connecting;
	m_playerColor = TurtlePieceColor::None;
	m_winner = Winner::None;

	setFixedSize(WINDOW_WIDTH, WINDOW_HEIGHT);
	setWindowTitle("TurtleCheckers");

	srand(time(NULL));

	m_updateTimer = new QTimer(this);
	m_updateTimer->setInterval(16);
	m_updateTimer->start();
	connect(m_updateTimer, SIGNAL(timeout()), this, SLOT(onUpdate()));

	setMouseTracking(true);

	m_board = std::make_shared<CheckersBoardRender>();

	m_hud = std::make_shared<HUD>();
	m_hud->setPiecesRemaining(m_board->getBlackTurtlesRemaining(), m_board->getRedTurtlesRemaining());
	m_hud->setGameState(GameState::Connecting);

	rcl_interfaces::msg::FloatingPointRange range;
	range.from_value = 0.01f;
	range.to_value = 255.0f;
	rcl_interfaces::msg::ParameterDescriptor boardScaleDescriptor;
	boardScaleDescriptor.description = "Scale of the checkers board render";
	boardScaleDescriptor.floating_point_range.push_back(range);
	m_nodeHandle->declare_parameter(
		"board_scale", rclcpp::ParameterValue(DEFAULT_BOARD_SCALE), boardScaleDescriptor);

	rcl_interfaces::msg::ParameterDescriptor holonomicDescriptor;
	holonomicDescriptor.description = "If true, then turtles will be holonomic";
	m_nodeHandle->declare_parameter("holonomic", rclcpp::ParameterValue(false), holonomicDescriptor);

	RCLCPP_INFO(
		m_nodeHandle->get_logger(), "Starting turtle checkers board with node name %s", m_nodeHandle->get_fully_qualified_name());

	m_connectToGameClient = m_nodeHandle->create_client<turtle_checkers_interfaces::srv::ConnectToGame>("ConnectToGame");
	while (!m_connectToGameClient->wait_for_service(std::chrono::seconds(10)))
	{
		if (!rclcpp::ok())
		{
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
			return;
		}
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
	}
	auto request = std::make_shared<turtle_checkers_interfaces::srv::ConnectToGame::Request>();
	request->player_name = m_playerName;
	m_connectToGameClient->async_send_request(request,
											  std::bind(&CheckersGameWindow::connectToGameResponse, this, std::placeholders::_1));
}

CheckersGameWindow::~CheckersGameWindow()
{
	delete m_updateTimer;
}

void CheckersGameWindow::parameterEventCallback(
	const rcl_interfaces::msg::ParameterEvent::ConstSharedPtr event)
{
	// Only consider events from this node
	if (event->node == m_nodeHandle->get_fully_qualified_name())
	{
		// Since parameter events for this event aren't expected frequently just always call update()
		update();
	}
}

void CheckersGameWindow::connectToGameResponse(rclcpp::Client<turtle_checkers_interfaces::srv::ConnectToGame>::SharedFuture future)
{
	auto result = future.get();

	m_lobbyName = result->lobby_name;

	// Create the services, subscriptions, and publishers now that we have the full topic names
	m_requestPieceMoveClient = m_nodeHandle->create_client<turtle_checkers_interfaces::srv::RequestPieceMove>(
		m_lobbyName + "/RequestPieceMove");
	m_requestReachableTilesClient = m_nodeHandle->create_client<turtle_checkers_interfaces::srv::RequestReachableTiles>(
		m_lobbyName + "/RequestReachableTiles");

	m_declareWinnerSubscription = m_nodeHandle->create_subscription<turtle_checkers_interfaces::msg::DeclareWinner>(
		m_lobbyName + "/DeclareWinner", 10, std::bind(&CheckersGameWindow::declareWinnerCallback, this, _1));
	m_gameStartSubscription = m_nodeHandle->create_subscription<turtle_checkers_interfaces::msg::GameStart>(
		m_lobbyName + "/GameStart", 10, std::bind(&CheckersGameWindow::gameStartCallback, this, _1));
	m_updateBoardSubscription = m_nodeHandle->create_subscription<turtle_checkers_interfaces::msg::UpdateBoard>(
		m_lobbyName + "/UpdateBoard", 10, std::bind(&CheckersGameWindow::updateBoardCallback, this, _1));

	m_playerReadyPublisher = m_nodeHandle->create_publisher<turtle_checkers_interfaces::msg::PlayerReady>(
		m_lobbyName + "/PlayerReady", 10);

	if (result->player_color == 1) // Black
	{
		m_playerColor = TurtlePieceColor::Black;
		RCLCPP_INFO(m_nodeHandle->get_logger(), "Connected as black player.");
	}
	else if (result->player_color == 2) // Red
	{
		m_playerColor = TurtlePieceColor::Red;
		RCLCPP_INFO(m_nodeHandle->get_logger(), "Connected as red player.");
	}
	else // Failed to connect (0)
	{
		m_playerColor = TurtlePieceColor::None;
		RCLCPP_WARN(m_nodeHandle->get_logger(), "Failed to connect to a game.");
	}

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

	// Tell the game master this player is ready
	auto message = turtle_checkers_interfaces::msg::PlayerReady();
	message.lobby_name = m_lobbyName;
	message.player_name = m_playerName;
	m_playerReadyPublisher->publish(message);
}

void CheckersGameWindow::requestPieceMoveResponse(rclcpp::Client<turtle_checkers_interfaces::srv::RequestPieceMove>::SharedFuture future)
{
	auto result = future.get();
	if (!result->move_accepted) // If the move was rejected, clear everything and try again (This normally shouldn't happen)
	{
		m_board->clearSelections();
	}

	update();
}

void CheckersGameWindow::requestReachableTilesResponse(rclcpp::Client<turtle_checkers_interfaces::srv::RequestReachableTiles>::SharedFuture future)
{
	auto result = future.get();
	const auto &reachableTileIndices = result->reachable_tile_indices;

	m_board->setReachableTiles(reachableTileIndices);

	update();
}

void CheckersGameWindow::declareWinnerCallback(const turtle_checkers_interfaces::msg::DeclareWinner::SharedPtr message)
{
	if (message->lobby_name != m_lobbyName)
	{
		return;
	}

	m_winner = static_cast<Winner>(message->winner);
	m_hud->setWinner(m_winner);

	update();
}

void CheckersGameWindow::gameStartCallback(const turtle_checkers_interfaces::msg::GameStart::SharedPtr message)
{
	if (message->lobby_name != m_lobbyName)
	{
		return;
	}

	m_gameState = static_cast<GameState>(message->game_state);
	m_hud->setGameState(m_gameState);

	if (isOwnTurn())
	{
		m_board->setMovablePieces(message->movable_tile_indices);
	}

	update();
}

void CheckersGameWindow::updateBoardCallback(const turtle_checkers_interfaces::msg::UpdateBoard::SharedPtr message)
{
	if (message->lobby_name != m_lobbyName)
	{
		return;
	}

	m_board->clearSelections();
	m_board->clearMovedTiles();

	m_board->moveTurtlePiece(message->source_tile_index, message->destination_tile_index);

	GameState nextGameState = static_cast<GameState>(message->game_state);
	if (message->slain_piece_tile_index > -1)
	{
		m_board->slayTurtle(message->slain_piece_tile_index);
		if (m_gameState != nextGameState) // If the game state switches on this update, the last turn finished
		{
			m_board->moveTurtlePiecesToGraveyard(m_blackPlayerGraveyard, m_redPlayerGraveyard);
		}
	}

	if (message->king_piece)
	{
		m_board->kingPiece(message->destination_tile_index);
	}

	m_gameState = nextGameState;
	if (isOwnTurn())
	{
		m_board->setMovablePieces(message->movable_tile_indices);
	}

	m_hud->setGameState(m_gameState);
	m_hud->setPiecesRemaining(m_board->getBlackTurtlesRemaining(), m_board->getRedTurtlesRemaining());

	update();
}

bool CheckersGameWindow::isOwnTurn()
{
	return ((m_playerColor == TurtlePieceColor::Black && m_gameState == GameState::BlackMove) ||
			(m_playerColor == TurtlePieceColor::Red && m_gameState == GameState::RedMove));
}

void CheckersGameWindow::mouseMoveEvent(QMouseEvent *event)
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

void CheckersGameWindow::mousePressEvent(QMouseEvent *event)
{
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
				m_board->handleMouseClick(event);
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
				m_board->handleMouseClick(event);
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

	if (m_board)
	{
		if (m_board->getIsMoveSelected())
		{
			auto request = std::make_shared<turtle_checkers_interfaces::srv::RequestPieceMove::Request>();
			request->lobby_name = m_lobbyName;
			request->source_tile_index = m_board->getSourceTileIndex();
			request->destination_tile_index = m_board->getDestinationTileIndex();
			m_requestPieceMoveClient->async_send_request(request,
														 std::bind(&CheckersGameWindow::requestPieceMoveResponse, this, std::placeholders::_1));
		}
		else
		{
			int selectedPieceTileIndex = m_board->getSelectedPieceTileIndex();
			if (selectedPieceTileIndex > -1)
			{
				auto request = std::make_shared<turtle_checkers_interfaces::srv::RequestReachableTiles::Request>();
				request->lobby_name = m_lobbyName;
				request->tile_index = selectedPieceTileIndex;
				m_requestReachableTilesClient->async_send_request(request,
																  std::bind(&CheckersGameWindow::requestReachableTilesResponse, this, std::placeholders::_1));
			}
		}
	}

	update();

	QFrame::mousePressEvent(event); // Ensure base class event handling
}

void CheckersGameWindow::onUpdate()
{
	if (!rclcpp::ok())
	{
		close();
		return;
	}

	rclcpp::spin_some(m_nodeHandle);
}

void CheckersGameWindow::paintEvent(QPaintEvent *event)
{
	(void)event; // NO LINT
	QPainter painter(this);

	if (m_playerColor == TurtlePieceColor::None)
	{
		return; // If we haven't connected yet, don't draw a board
	}

	// Fill the background
	QRgb backgroundColor = qRgb(BG_RGB[0], BG_RGB[1], BG_RGB[2]);
	painter.fillRect(0, 0, width(), height(), backgroundColor);

	m_board->paint(painter);

	// Draw the graveyards and any turtles they contain
	m_blackPlayerGraveyard->paint(painter);
	m_redPlayerGraveyard->paint(painter);

	// Draw the HUD
	m_hud->paint(painter);
}