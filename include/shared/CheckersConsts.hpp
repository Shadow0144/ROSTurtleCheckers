#pragma once

#include <cstddef>

// Enums
enum class GameState
{
    Connecting,
    Connected,
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

enum class RematchType
{
    SameColors,
    SwapColors,
    RandomColors
};

// Constants
constexpr size_t NUM_PIECES_PER_PLAYER = 1u;
constexpr size_t NUM_PIECES = 2u * NUM_PIECES_PER_PLAYER;

constexpr size_t NUM_PLAYABLE_ROWS = 8u;
constexpr size_t NUM_PLAYABLE_COLS = 4u;
constexpr static size_t NUM_PLAYABLE_TILES = NUM_PLAYABLE_ROWS * NUM_PLAYABLE_COLS;

constexpr size_t BLACK_OFFSET = NUM_PLAYABLE_TILES - NUM_PIECES_PER_PLAYER;

constexpr int UPWARD_ANGLE = 0;
constexpr int DOWNWARD_ANGLE = 180;

constexpr int TILE_WIDTH = 55;
constexpr int TILE_HEIGHT = 55;
constexpr float TILE_HALF_WIDTH = 0.5f * TILE_WIDTH;
constexpr float TILE_HALF_HEIGHT = 0.5f * TILE_HEIGHT;

constexpr int TITLE_FONT_SIZE = 24;

constexpr int HUD_HEIGHT = 20;
constexpr int HUD_TEXT_X_OFFSET = 5;
constexpr int HUD_TEXT_Y_OFFSET = 5;
constexpr int HUD_FONT_SIZE = 10;
constexpr int VICTORY_TEXT_Y_OFFSET = -35;
constexpr int VICTORY_TEXT_HEIGHT = 24;
constexpr int VICTORY_TEXT_FONT_SIZE = 24;
constexpr int VICTORY_IMAGE_Y_OFFSET = 35;

constexpr int GRAVEYARD_WIDTH = 2u * TILE_WIDTH;

constexpr int BOARD_LEFT = GRAVEYARD_WIDTH;
constexpr int BOARD_TOP = HUD_HEIGHT;
constexpr int BOARD_WIDTH = 8 * TILE_WIDTH;
constexpr int BOARD_HEIGHT = 8 * TILE_HEIGHT;

constexpr int BUTTON_DOCK_HEIGHT = 50;

constexpr int WINDOW_WIDTH = BOARD_WIDTH + (2 * GRAVEYARD_WIDTH);
constexpr int WINDOW_HEIGHT = BOARD_HEIGHT + HUD_HEIGHT + BUTTON_DOCK_HEIGHT;

constexpr float BOARD_CENTER_X = BOARD_LEFT + (0.5f * BOARD_WIDTH);
constexpr float BOARD_CENTER_Y = BOARD_TOP + (0.5f * BOARD_HEIGHT);

constexpr float DEFAULT_BOARD_SCALE = 1.0f;

constexpr int BG_RGB[3] = {0u, 0u, 0u};
constexpr int TEXT_RGB[3] = {255u, 255u, 255u};
constexpr int TEXT_DISABLED_RGB[3] = {100u, 100u, 100u};

constexpr int RED_SQUARES_BG_RGB[3] = {255u, 0u, 0u};
constexpr int REACHABLE_SQUARES_BG_RGB[3] = {0u, 0u, 255u};
constexpr int HIGHLIGHTED_SQUARES_BG_RGB[3] = {0u, 150u, 255u};
constexpr int SELECTED_SQUARES_BG_RGB[3] = {0u, 255u, 0u};
constexpr int LAST_MOVED_FROM_SQUARES_BG_RGB[3] = {0u, 0u, 100u};
constexpr int LAST_MOVED_TO_SQUARES_BG_RGB[3] = {0u, 0u, 150u};
constexpr int LAST_JUMPED_OVER_SQUARES_BG_RGB[3] = {50u, 50u, 100u};
constexpr int BLACK_SQUARES_BG_RGB[3] = {0u, 0u, 0u};
constexpr int HUD_BG_RGB[3] = {100u, 100u, 100u};
constexpr int GRAVEYARD_BG_RGB[3] = {50u, 50u, 50u};

constexpr int NUM_BITS_RSA = 20;