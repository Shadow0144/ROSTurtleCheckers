#pragma once

#include <cstddef>

// Enums
enum class GameState
{
    Connecting,
    BlackMove,
    RedMove,
    GameFinished
};

enum class Winner
{
    None,
    Black,
    Red,
    Draw
};

enum class TurtlePieceColor
{
    None,
    Black,
    Red
};

// Constants
constexpr size_t NUM_PIECES_PER_PLAYER = 12u;

constexpr size_t NUM_COLS_ROWS = 8u;
constexpr static size_t NUM_PLAYABLE_TILES = 32u;
constexpr size_t NUM_PLAYABLE_ROWS = 8u;
constexpr size_t NUM_PLAYABLE_COLS = 4u;

constexpr int RED_SQUARES_BG_RGB[3] = {255u, 0u, 0u};

constexpr size_t BLACK_OFFSET = 20u;

constexpr int UPWARD_ANGLE = 0;
constexpr int DOWNWARD_ANGLE = 180;

constexpr int TILE_WIDTH = 55;
constexpr int TILE_HEIGHT = 55;
constexpr float TILE_HALF_WIDTH = 0.5f * TILE_WIDTH;
constexpr float TILE_HALF_HEIGHT = 0.5f * TILE_HEIGHT;
constexpr int BOARD_WIDTH = 8 * TILE_WIDTH;
constexpr int BOARD_HEIGHT = 8 * TILE_HEIGHT;

constexpr float DEFAULT_BOARD_SCALE = 1.0f;

constexpr size_t MAX_COL_ROW_INDEX = 7u;
constexpr size_t MAX_JUMP_INDEX = MAX_COL_ROW_INDEX - 1u;

constexpr int REACHABLE_SQUARES_BG_RGB[3] = {0u, 0u, 255u};
constexpr int HIGHLIGHTED_SQUARES_BG_RGB[3] = {0u, 150u, 255u};
constexpr int SELECTED_SQUARES_BG_RGB[3] = {0u, 255u, 0u};
constexpr int BLACK_SQUARES_BG_RGB[3] = {0u, 0u, 0u};