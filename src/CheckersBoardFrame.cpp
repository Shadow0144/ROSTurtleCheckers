#include "CheckersBoardFrame.hpp"

#include <QPointF>

#include <cstdlib>
#include <ctime>
#include <functional>
#include <string>
#include <chrono>

#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rclcpp/rclcpp.hpp"

#include "turtle_checkers_interfaces/srv/connect_to_game.hpp"
#include "turtle_checkers_interfaces/srv/request_reachable_tiles.hpp"
#include "turtle_checkers_interfaces/srv/request_piece_move.hpp"
#include "turtle_checkers_interfaces/msg/update_board.hpp"
#include "turtle_checkers_interfaces/msg/update_game_state.hpp"

#include "CheckersConsts.hpp"

using std::placeholders::_1;

CheckersBoardFrame::CheckersBoardFrame(
	rclcpp::Node::SharedPtr &nodeHandle,
	const std::string &playerName,
	QWidget *parent,
	Qt::WindowFlags windowFlags)
	: QFrame(parent, windowFlags),
	  m_playerName(playerName)
{
	setFixedSize(WINDOW_WIDTH, WINDOW_HEIGHT);
	setWindowTitle("TurtleCheckers");

	srand(time(NULL));

	setMouseTracking(true);

	m_gameState = GameState::Connecting;
	m_playerColor = TurtlePieceColor::None;

	m_blackTurtlesRemaining = NUM_PIECES_PER_PLAYER;
	m_redTurtlesRemaining = NUM_PIECES_PER_PLAYER;

	m_hud = std::make_shared<HUD>();
	m_hud->setPiecesRemaining(m_blackTurtlesRemaining, m_redTurtlesRemaining);
	m_hud->setGameState(GameState::Connecting);

	m_updateTimer = new QTimer(this);
	m_updateTimer->setInterval(16);
	m_updateTimer->start();
	connect(m_updateTimer, SIGNAL(timeout()), this, SLOT(onUpdate()));

	m_nodeHandle = nodeHandle;

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

	QVector<QString> turtlesImageNames;
	/*turtlesImageNames.append("ardent.png");
	turtlesImageNames.append("bouncy.png");
	turtlesImageNames.append("crystal.png");
	turtlesImageNames.append("dashing.png");
	turtlesImageNames.append("eloquent.png");
	turtlesImageNames.append("foxy.png");
	turtlesImageNames.append("galactic.png");
	turtlesImageNames.append("humble.png");
	turtlesImageNames.append("iron.png");
	turtlesImageNames.append("jazzy.png");*/
	turtlesImageNames.append("rolling");

	QString imagesPath =
		(ament_index_cpp::get_package_share_directory("turtle_checkers") + "/img/").c_str();
	for (int i = 0; i < turtlesImageNames.size(); ++i)
	{
		QImage bImg;
		bImg.load(imagesPath + turtlesImageNames[i] + "_black.png");
		m_blackTurtleImages.append(bImg);
		QImage rImg;
		rImg.load(imagesPath + turtlesImageNames[i] + "_red.png");
		m_redTurtleImages.append(rImg);
		QImage kImg;
		kImg.load(imagesPath + turtlesImageNames[i] + "_king.png");
		m_kingTurtleImages.append(kImg);
		QImage hImg;
		hImg.load(imagesPath + turtlesImageNames[i] + "_highlight.png");
		m_highlightTurtleImages.append(hImg);
		QImage sImg;
		sImg.load(imagesPath + turtlesImageNames[i] + "_select.png");
		m_selectTurtleImages.append(sImg);
	}

	m_selectedPieceName = "";
	m_sourceTileIndex = -1;
	m_destinationTileIndex = -1;
	m_moveSelected = false;

	clear();

	m_blackPlayerGraveyard = std::make_shared<TurtleGraveyard>(TurtlePieceColor::Black);
	m_redPlayerGraveyard = std::make_shared<TurtleGraveyard>(TurtlePieceColor::Red);

	RCLCPP_INFO(
		m_nodeHandle->get_logger(), "Starting turtle checkers board with node name %s", m_nodeHandle->get_fully_qualified_name());

	m_requestReachableTilesClient = m_nodeHandle->create_client<turtle_checkers_interfaces::srv::RequestReachableTiles>("RequestReachableTiles");
	m_requestPieceMoveClient = m_nodeHandle->create_client<turtle_checkers_interfaces::srv::RequestPieceMove>("RequestPieceMove");

	m_updateGameStateSubscription = m_nodeHandle->create_subscription<turtle_checkers_interfaces::msg::UpdateGameState>("UpdateGameState", 10, std::bind(&CheckersBoardFrame::updateGameStateCallback, this, _1));
	m_updateBoardSubscription = m_nodeHandle->create_subscription<turtle_checkers_interfaces::msg::UpdateBoard>("UpdateBoard", 10, std::bind(&CheckersBoardFrame::updateBoardCallback, this, _1));

	m_connectToGameClient = m_nodeHandle->create_client<turtle_checkers_interfaces::srv::ConnectToGame>("ConnectToGame");
	while (!m_connectToGameClient->wait_for_service(std::chrono::seconds(1)))
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

	switch (result->game_state)
	{
	case 0: // Connecting
	{
		m_gameState = GameState::Connecting;
	}
	break;
	case 1: // Connected
	{
		m_gameState = GameState::Connected;
	}
	break;
	case 2: // Black turn
	{
		m_gameState = GameState::BlackMove;
	}
	break;
	case 3: // Red turn
	{
		m_gameState = GameState::RedMove;
	}
	break;
	case 4: // Game over
	{
		m_gameState = GameState::GameFinished;
	}
	break;
	}

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

	m_hud->setPlayerColor(m_playerColor);
	m_hud->setGameState(m_gameState);

	spawnTiles();
	spawnPieces();
}

void CheckersBoardFrame::requestReachableTilesResponse(rclcpp::Client<turtle_checkers_interfaces::srv::RequestReachableTiles>::SharedFuture future)
{
	for (size_t i = 0u; i < NUM_PLAYABLE_TILES; i++)
	{
		m_tileRenders[i]->toggleIsTileReachable(false);
	}
	auto result = future.get();
	const auto &highlightedTiles = result->reachable_tile_indices;
	for (size_t i = 0u; i < highlightedTiles.size(); i++)
	{
		m_tileRenders[highlightedTiles[i]]->toggleIsTileReachable(true);
	}
	update();
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

void CheckersBoardFrame::updateGameStateCallback(const turtle_checkers_interfaces::msg::UpdateGameState::SharedPtr message)
{
	switch (message->game_state)
	{
	case 0: // Connecting
	{
		m_gameState = GameState::Connecting;
	}
	break;
	case 1: // Connected
	{
		m_gameState = GameState::Connected;
	}
	break;
	case 2: // Black turn
	{
		m_gameState = GameState::BlackMove;
	}
	break;
	case 3: // Red turn
	{
		m_gameState = GameState::RedMove;
	}
	break;
	case 4: // Game over
	{
		m_gameState = GameState::GameFinished;
	}
	break;
	}

	m_hud->setGameState(m_gameState);

	update();
}

void CheckersBoardFrame::updateBoardCallback(const turtle_checkers_interfaces::msg::UpdateBoard::SharedPtr message)
{
	clearSelections();

	if (!message->piece_name.empty() &&
		message->source_tile_index < NUM_PLAYABLE_TILES &&
		message->destination_tile_index < NUM_PLAYABLE_TILES)
	{
		m_tileRenders[message->source_tile_index]->moveTurtlePiece(m_tileRenders[message->destination_tile_index]);
		for (size_t i = 0u; i < NUM_PLAYABLE_TILES; i++)
		{
			m_tileRenders[i]->toggleIsTileLastSelected(false);
		}
		m_tileRenders[message->destination_tile_index]->toggleIsTileLastSelected(true);
	}

	if (message->slain_piece_tile_index > -1)
	{
		auto slainPieceTileIndex = message->slain_piece_tile_index;
		if (slainPieceTileIndex < static_cast<int>(NUM_PLAYABLE_TILES) &&
			slainPieceTileIndex < static_cast<int>(NUM_PLAYABLE_TILES))
		{
			switch (m_tileRenders[slainPieceTileIndex]->getTurtlePieceColor())
			{
			case TurtlePieceColor::Black:
			{
				m_redPlayerGraveyard->addTurtlePiece(m_tileRenders[slainPieceTileIndex]);
				m_blackTurtlesRemaining--;
			}
			break;
			case TurtlePieceColor::Red:
			{
				m_blackPlayerGraveyard->addTurtlePiece(m_tileRenders[slainPieceTileIndex]);
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
	}

	switch (message->game_state)
	{
	case 0: // Connecting
	{
		m_gameState = GameState::Connecting;
	}
	break;
	case 1: // Connected
	{
		m_gameState = GameState::Connected;
	}
	break;
	case 2: // Black turn
	{
		m_gameState = GameState::BlackMove;
	}
	break;
	case 3: // Red turn
	{
		m_gameState = GameState::RedMove;
	}
	break;
	case 4: // Game over
	{
		m_gameState = GameState::GameFinished;
	}
	break;
	}

	m_hud->setGameState(m_gameState);
	m_hud->setPiecesRemaining(m_blackTurtlesRemaining, m_redTurtlesRemaining);

	update();
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
		m_tileRenders[i]->toggleIsPieceHighlighted(false);
	}
	for (size_t i = 0u; i < NUM_PLAYABLE_TILES; i++)
	{
		if (m_tileRenders[i]->containsPoint(event->pos()) && m_tileRenders[i]->containsPiece(m_playerColor))
		{
			m_tileRenders[i]->toggleIsPieceHighlighted(true);
			break;
		}
	}

	if (!m_selectedPieceName.empty())
	{
		for (size_t i = 0u; i < NUM_PLAYABLE_TILES; i++)
		{
			m_tileRenders[i]->toggleIsTileHighlighted(false);
		}
		for (size_t i = 0u; i < NUM_PLAYABLE_TILES; i++)
		{
			if (m_tileRenders[i]->getIsTileReachable() && m_tileRenders[i]->containsPoint(event->pos()))
			{
				m_tileRenders[i]->toggleIsTileHighlighted(true);
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
			m_tileRenders[i]->toggleIsPieceSelected(false);
			m_tileRenders[i]->toggleIsTileReachable(false);
			m_tileRenders[i]->toggleIsTileHighlighted(false);
			m_tileRenders[i]->toggleIsTileSelected(false);
		}
		bool selected = false;
		for (size_t i = 0u; i < NUM_PLAYABLE_TILES; i++)
		{
			if (m_tileRenders[i]->containsPoint(event->pos()) && m_tileRenders[i]->containsPiece(m_playerColor))
			{
				if (m_selectedPieceName != m_tileRenders[i]->getTurtlePiece()->getName()) // If we've clicked a new piece, select it
				{
					m_sourceTileIndex = i;
					m_selectedPieceName = m_tileRenders[m_sourceTileIndex]->getTurtlePiece()->getName();
					m_tileRenders[m_sourceTileIndex]->toggleIsPieceSelected(true);
					auto request = std::make_shared<turtle_checkers_interfaces::srv::RequestReachableTiles::Request>();
					request->piece_name = m_selectedPieceName;
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
					m_tileRenders[i]->toggleIsTileSelected(true);
					auto request = std::make_shared<turtle_checkers_interfaces::srv::RequestPieceMove::Request>();
					request->piece_name = m_selectedPieceName;
					request->source_tile_index = m_sourceTileIndex;
					request->destination_tile_index = m_destinationTileIndex;
					m_requestPieceMoveClient->async_send_request(request,
																 std::bind(&CheckersBoardFrame::requestPieceMoveResponse, this, std::placeholders::_1));
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
	for (size_t i = 0u; i < NUM_PLAYABLE_TILES; i++)
	{
		m_tileRenders[i]->toggleIsPieceSelected(false);
		m_tileRenders[i]->toggleIsTileReachable(false);
		m_tileRenders[i]->toggleIsTileHighlighted(false);
		m_tileRenders[i]->toggleIsTileSelected(false);
	}
	m_selectedPieceName.clear();
	m_sourceTileIndex = -1;
	m_destinationTileIndex = -1;
	m_moveSelected = false;
}

void CheckersBoardFrame::spawnTiles()
{
	float tileCentersX[NUM_PLAYABLE_TILES];
	float tileCentersY[NUM_PLAYABLE_TILES];
	for (size_t i = 0u; i < NUM_COLS_ROWS; i++)
	{
		if (i % 2u == 0u)
		{
			tileCentersX[(i * 4) + 0] = (1 * TILE_WIDTH) + TILE_HALF_WIDTH + GRAVEYARD_WIDTH;
			tileCentersX[(i * 4) + 1] = (3 * TILE_WIDTH) + TILE_HALF_WIDTH + GRAVEYARD_WIDTH;
			tileCentersX[(i * 4) + 2] = (5 * TILE_WIDTH) + TILE_HALF_WIDTH + GRAVEYARD_WIDTH;
			tileCentersX[(i * 4) + 3] = (7 * TILE_WIDTH) + TILE_HALF_WIDTH + GRAVEYARD_WIDTH;
		}
		else
		{
			tileCentersX[(i * 4) + 0] = (0 * TILE_WIDTH) + TILE_HALF_WIDTH + GRAVEYARD_WIDTH;
			tileCentersX[(i * 4) + 1] = (2 * TILE_WIDTH) + TILE_HALF_WIDTH + GRAVEYARD_WIDTH;
			tileCentersX[(i * 4) + 2] = (4 * TILE_WIDTH) + TILE_HALF_WIDTH + GRAVEYARD_WIDTH;
			tileCentersX[(i * 4) + 3] = (6 * TILE_WIDTH) + TILE_HALF_WIDTH + GRAVEYARD_WIDTH;
		}

		if (m_playerColor == TurtlePieceColor::Black)
		{
			tileCentersY[(i * 4) + 0] = (i * TILE_HEIGHT) + TILE_HALF_HEIGHT + HUD_HEIGHT;
			tileCentersY[(i * 4) + 1] = (i * TILE_HEIGHT) + TILE_HALF_HEIGHT + HUD_HEIGHT;
			tileCentersY[(i * 4) + 2] = (i * TILE_HEIGHT) + TILE_HALF_HEIGHT + HUD_HEIGHT;
			tileCentersY[(i * 4) + 3] = (i * TILE_HEIGHT) + TILE_HALF_HEIGHT + HUD_HEIGHT;
		}
		else // Red - The red player faces the board from the other direction, so mirror it
		{
			tileCentersY[(i * 4) + 0] = WINDOW_HEIGHT - (i * TILE_HEIGHT) - TILE_HALF_HEIGHT;
			tileCentersY[(i * 4) + 1] = WINDOW_HEIGHT - (i * TILE_HEIGHT) - TILE_HALF_HEIGHT;
			tileCentersY[(i * 4) + 2] = WINDOW_HEIGHT - (i * TILE_HEIGHT) - TILE_HALF_HEIGHT;
			tileCentersY[(i * 4) + 3] = WINDOW_HEIGHT - (i * TILE_HEIGHT) - TILE_HALF_HEIGHT;
		}
	}
	for (size_t i = 0u; i < NUM_PLAYABLE_TILES; i++)
	{
		m_tileRenders[i] = std::make_shared<TileRender>(QPointF(tileCentersX[i], tileCentersY[i]));
	}
}

void CheckersBoardFrame::clearPieces()
{
}

void CheckersBoardFrame::spawnPieces()
{
	// Spawn all the checkers pieces
	// Red
	for (size_t i = 0u; i < NUM_PIECES_PER_PLAYER; i++)
	{
		std::string name = "Red" + std::to_string(i + 1);
		auto center = m_tileRenders[i]->getCenterPosition();
		spawnTurtle(name,
					false,
					center.x(),
					center.y(),
					(m_playerColor == TurtlePieceColor::Black) ? DOWNWARD_ANGLE : UPWARD_ANGLE,
					m_redImageIndex);
		m_tileRenders[i]->setTurtlePiece(m_redTurtles[name]);
	}
	// Black
	for (size_t i = 0u; i < NUM_PIECES_PER_PLAYER; i++)
	{
		std::string name = "Black" + std::to_string(i + 1);
		auto center = m_tileRenders[i + BLACK_OFFSET]->getCenterPosition();
		spawnTurtle(name,
					true,
					center.x(),
					center.y(),
					(m_playerColor == TurtlePieceColor::Black) ? UPWARD_ANGLE : DOWNWARD_ANGLE,
					m_blackImageIndex);
		m_tileRenders[i + BLACK_OFFSET]->setTurtlePiece(m_blackTurtles[name]);
	}
}

void CheckersBoardFrame::spawnTurtle(const std::string &name,
									 bool black,
									 float x,
									 float y,
									 float angle,
									 size_t imageIndex)
{
	if (black)
	{
		m_blackTurtles[name] = std::make_shared<TurtlePiece>(
			name,
			TurtlePieceColor::Black,
			m_blackTurtleImages[static_cast<int>(imageIndex)],
			m_kingTurtleImages[static_cast<int>(imageIndex)],
			m_highlightTurtleImages[static_cast<int>(imageIndex)],
			m_selectTurtleImages[static_cast<int>(imageIndex)],
			QPointF(x, y),
			angle);
		/*RCLCPP_INFO(
			m_nodeHandle->get_logger(), "Spawning black turtle [%s] at x=[%f], y=[%f], theta=[%f]",
			name.c_str(), x, y, angle);*/
	}
	else // red
	{
		m_redTurtles[name] = std::make_shared<TurtlePiece>(
			name,
			TurtlePieceColor::Red,
			m_redTurtleImages[static_cast<int>(imageIndex)],
			m_kingTurtleImages[static_cast<int>(imageIndex)],
			m_highlightTurtleImages[static_cast<int>(imageIndex)],
			m_selectTurtleImages[static_cast<int>(imageIndex)],
			QPointF(x, y),
			angle);
		/*RCLCPP_INFO(
			m_nodeHandle->get_logger(), "Spawning red turtle [%s] at x=[%f], y=[%f], theta=[%f]",
			name.c_str(), x, y, angle);*/
	}
	update();
}

void CheckersBoardFrame::clear()
{
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

	// Draw the black tiles over the red background
	for (size_t i = 0u; i < NUM_PLAYABLE_TILES; i++)
	{
		m_tileRenders[i]->paint(painter);
	}

	// Draw the black turtles
	TurtlePiecesMap::iterator bit = m_blackTurtles.begin();
	TurtlePiecesMap::iterator bend = m_blackTurtles.end();
	for (; bit != bend; ++bit)
	{
		bit->second->paint(painter);
	}

	// Draw the red turtles
	TurtlePiecesMap::iterator rit = m_redTurtles.begin();
	TurtlePiecesMap::iterator rend = m_redTurtles.end();
	for (; rit != rend; ++rit)
	{
		rit->second->paint(painter);
	}

	// Draw the graveyards
	m_blackPlayerGraveyard->paint(painter);
	m_redPlayerGraveyard->paint(painter);

	// Draw the HUD
	m_hud->paint(painter);
}