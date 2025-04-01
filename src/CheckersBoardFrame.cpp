#include "CheckersBoardFrame.hpp"

#include <QPointF>

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

#include "CheckersConsts.hpp"
#include "TileRenderFactory.hpp"
#include "TurtlePieceRenderFactory.hpp"

using std::placeholders::_1;

CheckersBoardFrame::CheckersBoardFrame(
	rclcpp::Node::SharedPtr &nodeHandle,
	const std::string &playerName,
	QWidget *parent,
	Qt::WindowFlags windowFlags)
	: QFrame(parent, windowFlags),
	  m_playerName(playerName)
{
	m_nodeHandle = nodeHandle;

	m_selectedPieceName = "";
	m_sourceTileIndex = -1;
	m_destinationTileIndex = -1;
	m_moveSelected = false;
	m_gameState = GameState::Connecting;
	m_playerColor = TurtlePieceColor::None;
	m_winner = Winner::None;

	setFixedSize(WINDOW_WIDTH, WINDOW_HEIGHT);
	setWindowTitle("TurtleCheckers");

	srand(time(NULL));

	setMouseTracking(true);

	m_hud = std::make_shared<HUD>();
	m_hud->setPiecesRemaining(m_blackTurtlesRemaining, m_redTurtlesRemaining);
	m_hud->setGameState(GameState::Connecting);

	m_updateTimer = new QTimer(this);
	m_updateTimer->setInterval(16);
	m_updateTimer->start();
	connect(m_updateTimer, SIGNAL(timeout()), this, SLOT(onUpdate()));

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

	m_requestPieceMoveClient = m_nodeHandle->create_client<turtle_checkers_interfaces::srv::RequestPieceMove>("RequestPieceMove");
	m_requestReachableTilesClient = m_nodeHandle->create_client<turtle_checkers_interfaces::srv::RequestReachableTiles>("RequestReachableTiles");

	m_declareWinnerSubscription = m_nodeHandle->create_subscription<turtle_checkers_interfaces::msg::DeclareWinner>("DeclareWinner", 10, std::bind(&CheckersBoardFrame::declareWinnerCallback, this, _1));
	m_gameStartSubscription = m_nodeHandle->create_subscription<turtle_checkers_interfaces::msg::GameStart>("GameStart", 10, std::bind(&CheckersBoardFrame::gameStartCallback, this, _1));
	m_updateBoardSubscription = m_nodeHandle->create_subscription<turtle_checkers_interfaces::msg::UpdateBoard>("UpdateBoard", 10, std::bind(&CheckersBoardFrame::updateBoardCallback, this, _1));

	m_playerReadyPublisher = m_nodeHandle->create_publisher<turtle_checkers_interfaces::msg::PlayerReady>("PlayerReady", 10);

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
											  std::bind(&CheckersBoardFrame::connectToGameResponse, this, std::placeholders::_1));
}

CheckersBoardFrame::~CheckersBoardFrame()
{
	delete m_updateTimer;
}

void CheckersBoardFrame::parameterEventCallback(
	const rcl_interfaces::msg::ParameterEvent::ConstSharedPtr event)
{
	// Only consider events from this node
	if (event->node == m_nodeHandle->get_fully_qualified_name())
	{
		// Since parameter events for this event aren't expected frequently just always call update()
		update();
	}
}

void CheckersBoardFrame::connectToGameResponse(rclcpp::Client<turtle_checkers_interfaces::srv::ConnectToGame>::SharedFuture future)
{
	auto result = future.get();

	m_lobbyName = result->lobby_name;

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

	// Create the graveyards now that we know which side of the screen they are on
	m_blackPlayerGraveyard = std::make_shared<TurtleGraveyard>(TurtlePieceColor::Black, m_playerColor);
	m_redPlayerGraveyard = std::make_shared<TurtleGraveyard>(TurtlePieceColor::Red, m_playerColor);

	m_hud->setPlayerColor(m_playerColor);
	m_hud->setGameState(m_gameState);

	m_tileRenders = TileRenderFactory::createTileRenders(NUM_PLAYABLE_ROWS, NUM_PLAYABLE_COLS, m_playerColor);
	m_turtlePieceRenders = TurtlePieceRenderFactory::createTurtlePieceRenders(NUM_PIECES_PER_PLAYER, m_tileRenders, m_playerColor);

	m_blackTurtlesRemaining = NUM_PIECES_PER_PLAYER;
	m_redTurtlesRemaining = NUM_PIECES_PER_PLAYER;

	// Tell the game master this player is ready
	auto message = turtle_checkers_interfaces::msg::PlayerReady();
	message.lobby_name = m_lobbyName;
	message.player_name = m_playerName;
	m_playerReadyPublisher->publish(message);
}

void CheckersBoardFrame::requestPieceMoveResponse(rclcpp::Client<turtle_checkers_interfaces::srv::RequestPieceMove>::SharedFuture future)
{
	auto result = future.get();
	if (!result->move_accepted) // If the move was rejected, clear everything and try again (This normally shouldn't happen)
	{
		clearSelections();
	}

	update();
}

void CheckersBoardFrame::requestReachableTilesResponse(rclcpp::Client<turtle_checkers_interfaces::srv::RequestReachableTiles>::SharedFuture future)
{
	for (size_t i = 0u; i < NUM_PLAYABLE_TILES; i++)
	{
		m_tileRenders[i]->setIsTileReachable(false);
	}
	auto result = future.get();
	const auto &highlightedTileIndices = result->reachable_tile_indices;
	for (size_t i = 0u; i < highlightedTileIndices.size(); i++)
	{
		m_tileRenders[highlightedTileIndices[i]]->setIsTileReachable(true);
	}

	update();
}

void CheckersBoardFrame::declareWinnerCallback(const turtle_checkers_interfaces::msg::DeclareWinner::SharedPtr message)
{
	if (message->lobby_name != m_lobbyName)
	{
		return;
	}

	m_winner = static_cast<Winner>(message->winner);
	m_hud->setWinner(m_winner);

	update();
}

void CheckersBoardFrame::gameStartCallback(const turtle_checkers_interfaces::msg::GameStart::SharedPtr message)
{
	if (message->lobby_name != m_lobbyName)
	{
		return;
	}

	m_gameState = static_cast<GameState>(message->game_state);
	m_hud->setGameState(m_gameState);

	for (auto &tileRender : m_tileRenders)
	{
		tileRender->setIsTurtlePieceMovable(false);
	}
	if (isOwnTurn())
	{
		for (auto movableTileIndex : message->movable_tile_indices)
		{
			if (movableTileIndex < NUM_PLAYABLE_TILES)
			{
				m_tileRenders[movableTileIndex]->setIsTurtlePieceMovable(true);
			}
		}
	}

	update();
}

void CheckersBoardFrame::updateBoardCallback(const turtle_checkers_interfaces::msg::UpdateBoard::SharedPtr message)
{
	if (message->lobby_name != m_lobbyName)
	{
		return;
	}

	clearSelections();
	for (auto &tileRender : m_tileRenders)
	{
		tileRender->setIsTileLastMovedFrom(false);
		tileRender->setIsTileLastMovedTo(false);
		tileRender->setIsTileLastJumpedOver(false);
		tileRender->setIsTurtlePieceMovable(false);
	}

	if (message->source_tile_index < NUM_PLAYABLE_TILES &&
		message->destination_tile_index < NUM_PLAYABLE_TILES)
	{
		m_tileRenders[message->source_tile_index]->moveTurtlePiece(m_tileRenders[message->destination_tile_index]);
		m_tileRenders[message->source_tile_index]->setIsTileLastMovedFrom(true);
		m_tileRenders[message->destination_tile_index]->setIsTileLastMovedTo(true);
	}

	GameState nextGameState = static_cast<GameState>(message->game_state);
	if (message->slain_piece_tile_index > -1)
	{
		auto slainPieceTileIndex = message->slain_piece_tile_index;
		if (slainPieceTileIndex < static_cast<int>(NUM_PLAYABLE_TILES))
		{
			m_tileRenders[slainPieceTileIndex]->setIsTileLastJumpedOver(true);
			m_tileRenders[slainPieceTileIndex]->setIsTurtlePieceDead(true);
		}
		m_tileIndicesOfSlainTurtles.push_back(slainPieceTileIndex);
		if (m_gameState != nextGameState) // If the game state switches on this update, the last turn finished
		{
			for (auto tileIndex : m_tileIndicesOfSlainTurtles)
			{
				switch (m_tileRenders[tileIndex]->getTurtlePieceColor())
				{
				case TurtlePieceColor::Black:
				{
					m_tileRenders[tileIndex]->moveTurtlePiece(m_redPlayerGraveyard);
					m_blackTurtlesRemaining--;
				}
				break;
				case TurtlePieceColor::Red:
				{
					m_tileRenders[tileIndex]->moveTurtlePiece(m_blackPlayerGraveyard);
					m_redTurtlesRemaining--;
				}
				break;
				case TurtlePieceColor::None:
				{
					// Do nothing
				}
				break;
				}
			}
			m_tileIndicesOfSlainTurtles.clear();
		}
	}

	if (message->king_piece)
	{
		m_tileRenders[message->destination_tile_index]->setIsTurtlePieceKinged(true);
	}

	m_gameState = nextGameState;
	if (isOwnTurn())
	{
		for (auto movableTileIndex : message->movable_tile_indices)
		{
			if (movableTileIndex < NUM_PLAYABLE_TILES)
			{
				m_tileRenders[movableTileIndex]->setIsTurtlePieceMovable(true);
			}
		}
	}

	m_hud->setGameState(m_gameState);
	m_hud->setPiecesRemaining(m_blackTurtlesRemaining, m_redTurtlesRemaining);

	update();
}

bool CheckersBoardFrame::isOwnTurn()
{
	return ((m_playerColor == TurtlePieceColor::Black && m_gameState == GameState::BlackMove) ||
			(m_playerColor == TurtlePieceColor::Red && m_gameState == GameState::RedMove));
}

void CheckersBoardFrame::mouseMoveEvent(QMouseEvent *event)
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
			handleMouseMove(event);
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
			handleMouseMove(event);
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

void CheckersBoardFrame::handleMouseMove(QMouseEvent *event)
{
	for (size_t i = 0u; i < NUM_PLAYABLE_TILES; i++)
	{
		m_tileRenders[i]->setIsTurtlePieceHighlighted(false);
	}

	if (!m_moveSelected && !m_selectedPieceName.empty())
	{
		for (size_t i = 0u; i < NUM_PLAYABLE_TILES; i++)
		{
			if (m_tileRenders[i]->containsPoint(event->pos()) &&
				m_tileRenders[i]->getIsTurtlePieceMovable())
			{
				m_tileRenders[i]->setIsTurtlePieceHighlighted(true);
				break;
			}
		}
		for (size_t i = 0u; i < NUM_PLAYABLE_TILES; i++)
		{
			m_tileRenders[i]->setIsTileHighlighted(false);
		}
		for (size_t i = 0u; i < NUM_PLAYABLE_TILES; i++)
		{
			if (m_tileRenders[i]->getIsTileReachable() && m_tileRenders[i]->containsPoint(event->pos()))
			{
				m_tileRenders[i]->setIsTileHighlighted(true);
				break;
			}
		}
	}
}

void CheckersBoardFrame::mousePressEvent(QMouseEvent *event)
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
				handleMouseClick(event);
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
				handleMouseClick(event);
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

	QFrame::mousePressEvent(event); // Ensure base class event handling
}

void CheckersBoardFrame::handleMouseClick(QMouseEvent *event)
{
	if (m_moveSelected)
	{
		// Do nothing, continue to wait for a response from the game node
	}
	else if (m_selectedPieceName.empty()) // If no piece is selected
	{
		for (size_t i = 0u; i < NUM_PLAYABLE_TILES; i++)
		{
			m_tileRenders[i]->setIsTurtlePieceSelected(false);
			m_tileRenders[i]->setIsTileReachable(false);
			m_tileRenders[i]->setIsTileHighlighted(false);
			m_tileRenders[i]->setIsTileSelected(false);
		}
		bool selected = false;
		for (size_t i = 0u; i < NUM_PLAYABLE_TILES; i++)
		{
			if (m_tileRenders[i]->containsPoint(event->pos()) &&
				m_tileRenders[i]->getIsTurtlePieceMovable())
			{
				if (m_selectedPieceName != m_tileRenders[i]->getTurtlePieceName()) // If we've clicked a new piece, select it
				{
					m_sourceTileIndex = i;
					m_selectedPieceName = m_tileRenders[m_sourceTileIndex]->getTurtlePieceName();
					m_tileRenders[m_sourceTileIndex]->setIsTurtlePieceSelected(true);
					auto request = std::make_shared<turtle_checkers_interfaces::srv::RequestReachableTiles::Request>();
					request->lobby_name = m_lobbyName;
					request->tile_index = m_sourceTileIndex;
					m_requestReachableTilesClient->async_send_request(request,
																	  std::bind(&CheckersBoardFrame::requestReachableTilesResponse, this, std::placeholders::_1));
					selected = true;
				}
				else // If we've clicked the same piece again, unselect it instead
				{
					m_selectedPieceName.clear();
				}
				break; // Only 1 tile can contain the mouse at any time
			}
		}
		if (!selected) // We clicked somewhere which isn't on a valid turtle, so clear the selected piece
		{
			m_selectedPieceName.clear();
		}
	}
	else // A piece is selected, look for a tile to accept
	{
		bool selected = false;
		for (size_t i = 0u; i < NUM_PLAYABLE_TILES; i++)
		{
			if (m_tileRenders[i]->containsPoint(event->pos()))
			{
				if (m_tileRenders[i]->getIsTileReachable())
				{
					m_destinationTileIndex = i;
					m_tileRenders[i]->setIsTileSelected(true);
					auto request = std::make_shared<turtle_checkers_interfaces::srv::RequestPieceMove::Request>();
					request->lobby_name = m_lobbyName;
					request->piece_name = m_selectedPieceName;
					request->source_tile_index = m_sourceTileIndex;
					request->destination_tile_index = m_destinationTileIndex;
					m_requestPieceMoveClient->async_send_request(request,
																 std::bind(&CheckersBoardFrame::requestPieceMoveResponse, this, std::placeholders::_1));
					for (auto &tileRender : m_tileRenders)
					{
						tileRender->setIsTileReachable(false);
						tileRender->setIsTileHighlighted(false);
						tileRender->setIsTurtlePieceMovable(false);
					}
					m_moveSelected = true;
					selected = true;
				}
				break; // Only 1 tile can contain the mouse at any time
			}
		}
		if (!selected) // We clicked somewhere which isn't on a valid turtle, so clear the selected piece
		{
			clearSelections();
		}
	}
}

void CheckersBoardFrame::clearSelections()
{
	for (auto &tileRender : m_tileRenders)
	{
		tileRender->setIsTileReachable(false);
		tileRender->setIsTileHighlighted(false);
		tileRender->setIsTileSelected(false);
		tileRender->setIsTurtlePieceSelected(false);
	}
	m_selectedPieceName.clear();
	m_sourceTileIndex = -1;
	m_destinationTileIndex = -1;
	m_moveSelected = false;
}

void CheckersBoardFrame::onUpdate()
{
	if (!rclcpp::ok())
	{
		close();
		return;
	}

	rclcpp::spin_some(m_nodeHandle);
}

void CheckersBoardFrame::paintEvent(QPaintEvent *event)
{
	(void)event; // NO LINT
	QPainter painter(this);

	if (m_playerColor == TurtlePieceColor::None)
	{
		return; // If we haven't connected yet, don't draw a board
	}

	// Fill the background in red
	QRgb backgroundColor = qRgb(RED_SQUARES_BG_RGB[0], RED_SQUARES_BG_RGB[1], RED_SQUARES_BG_RGB[2]);
	painter.fillRect(0, 0, width(), height(), backgroundColor);

	// Draw the black tiles over the red background and any turtles they contain
	for (auto &tileRender : m_tileRenders)
	{
		tileRender->paint(painter);
	}

	// Draw the graveyards and any turtles they contain
	m_blackPlayerGraveyard->paint(painter);
	m_redPlayerGraveyard->paint(painter);

	// Draw the HUD
	m_hud->paint(painter);
}