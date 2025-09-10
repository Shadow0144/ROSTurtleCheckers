#pragma once

#include <cstddef>
#include <algorithm>

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

enum class VictoryCondition
{
    None,
    TimeExpired,
    EliminatedPieces,
    PlayerHasNoMoves,
    PlayersHaveNoMoves,
    DrawAccepted,
    PlayerForfitted,
    PlayerDisconnected
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

enum class Language
{
    English,
    Japanese,
    German
};

// Constants
constexpr size_t NUM_PIECES_PER_PLAYER = 12u;
constexpr size_t NUM_PIECES = 2u * NUM_PIECES_PER_PLAYER;
constexpr size_t PIECE_SIZE = 50u;

constexpr size_t NUM_PLAYABLE_ROWS = 8u;
constexpr size_t NUM_PLAYABLE_COLS = 4u;
constexpr size_t NUM_PLAYABLE_TILES = NUM_PLAYABLE_ROWS * NUM_PLAYABLE_COLS;

constexpr size_t BLACK_OFFSET = NUM_PLAYABLE_TILES - NUM_PIECES_PER_PLAYER;

constexpr int UPWARD_ANGLE = 0;
constexpr int DOWNWARD_ANGLE = 180;

constexpr int TILE_WIDTH = 55;
constexpr int TILE_HEIGHT = 55;
constexpr float TILE_HALF_WIDTH = 0.5f * TILE_WIDTH;
constexpr float TILE_HALF_HEIGHT = 0.5f * TILE_HEIGHT;

constexpr int TITLE_FONT_SIZE = 36;
constexpr int TITLE_ICON_HEIGHT_WIDTH = 50;

constexpr int PLAYER_NAME_FONT_SIZE = 16;

constexpr int LOBBY_NAME_FONT_SIZE = 16;

constexpr int MENU_LINE_EDIT_WIDTH = 550;
constexpr int MENU_BUTTON_WIDTH = 150;

constexpr int LOBBY_LIST_SCROLL_W = 700;
constexpr int LOBBY_LIST_SCROLL_H = 340;
constexpr int LOBBY_LIST_LOBBY_NAME_WIDTH = 150;
constexpr int LOBBY_LIST_PLAYER_NAME_WIDTH = 150;

constexpr int STATISTICS_SCROLL_W = 750;
constexpr int STATISTICS_SCROLL_H = 250;
constexpr int MATCH_LIST_LOBBY_NAME_WIDTH = 130;
constexpr int MATCH_LIST_PLAYER_NAME_WIDTH = 130;

constexpr int GRAVEYARD_OFFSET_X = 9;
constexpr int GRAVEYARD_WIDTH = TILE_WIDTH;
constexpr int FULL_GRAVEYARD_WIDTH = GRAVEYARD_OFFSET_X + GRAVEYARD_WIDTH;
constexpr int GRAVEYARD_PADDING = 5;

constexpr int BOARD_WIDTH = 8 * TILE_WIDTH;
constexpr int BOARD_HEIGHT = 8 * TILE_HEIGHT;
constexpr int BOARD_LEFT = GRAVEYARD_OFFSET_X + GRAVEYARD_WIDTH;
constexpr int BOARD_TOP = 40;

constexpr int BUTTON_DOCK_HEIGHT = 50;

constexpr int HUD_HEIGHT = BOARD_TOP;
constexpr int HUD_WIDTH = (2 * FULL_GRAVEYARD_WIDTH) + BOARD_WIDTH;
constexpr int HUD_TEXT_X_OFFSET = 5;
constexpr int HUD_TEXT_Y_OFFSET = 5;
constexpr int HUD_FONT_SIZE = 10;
constexpr int HUD_BLACK_REMAINING_TEXT_X_OFFSET = 205;
constexpr int HUD_RED_REMAINING_TEXT_X_OFFSET = 405;
constexpr int HUD_REMAINING_TEXT_Y_OFFSET = 2;
constexpr int HUD_REMAINING_TEXT_WIDTH = 15;
constexpr int HUD_BLACK_TURTLE_ICON_X_OFFSET = 225;
constexpr int HUD_RED_TURTLE_ICON_X_OFFSET = 425;
constexpr int HUD_TURTLE_ICON_Y_OFFSET = 1;
constexpr int HUD_TURTLE_ICON_HEIGHT_WIDTH = 18;
constexpr int HUD_BLACK_TIMER_TEXT_X_OFFSET = 250;
constexpr int HUD_RED_TIMER_TEXT_X_OFFSET = 450;

constexpr int CHAT_HEADER_FONT_SIZE = 10;
constexpr int CHAT_BOX_MARGIN_W = 20;
constexpr int CHAT_BOX_MARGIN_H = 110;
constexpr int REPORT_PLAYER_MARGINS = 5;

constexpr int CHAT_BOX_IN_LOBBY_WIDTH = BOARD_WIDTH + (2 * GRAVEYARD_WIDTH) + 215;
constexpr int CHAT_BOX_IN_LOBBY_HEIGHT = 200;

constexpr int CHAT_BOX_IN_GAME_X = (2 * GRAVEYARD_WIDTH) + BOARD_WIDTH + GRAVEYARD_OFFSET_X;
constexpr int CHAT_BOX_IN_GAME_Y = 0;
constexpr int CHAT_BOX_IN_GAME_WIDTH = 240;
constexpr int CHAT_BOX_IN_GAME_HEIGHT = BOARD_HEIGHT + HUD_HEIGHT + BUTTON_DOCK_HEIGHT + 13;

constexpr int WINDOW_WIDTH = BOARD_WIDTH + (2 * GRAVEYARD_WIDTH) + GRAVEYARD_OFFSET_X + CHAT_BOX_IN_GAME_WIDTH;
constexpr int WINDOW_HEIGHT = BOARD_HEIGHT + HUD_HEIGHT + BUTTON_DOCK_HEIGHT;

constexpr int WINDOW_CENTER_X = WINDOW_WIDTH / 2;
constexpr int WINDOW_CENTER_Y = WINDOW_HEIGHT / 2;

constexpr float BOARD_CENTER_X = BOARD_LEFT + (0.5f * BOARD_WIDTH);
constexpr float BOARD_CENTER_Y = BOARD_TOP + (0.5f * BOARD_HEIGHT);

constexpr int VICTORY_TEXT_HEIGHT = 24;
constexpr int VICTORY_TEXT_FONT_SIZE = 24;
constexpr int VICTORY_WIDGET_Y_OFFSET = -50;
constexpr int VICTORY_BUTTONS_Y = BOARD_CENTER_Y + 100;

constexpr float DEFAULT_BOARD_SCALE = 1.0f;

constexpr int ICON_HEIGHT_WIDTH = 20;

constexpr int LANGUAGE_SELECT_X = WINDOW_WIDTH - 63;
constexpr int LANGUAGE_SELECT_Y = 10;
constexpr int LANGUAGE_ICON_WIDTH = 20;
constexpr int LANGUAGE_ICON_HEIGHT = 15;

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
constexpr int GRAVEYARD_BG_RGB[3] = {35u, 35u, 35u};

constexpr int NUM_BITS_RSA = 16;
constexpr int NUM_CHARS_SALT = 6;

constexpr int MAX_CHARS_PLAYER_NAME = 24;
constexpr int MAX_CHARS_PLAYER_PASS = 3;
constexpr int MAX_CHARS_LOBBY_NAME = 24;
constexpr int MAX_CHARS_LOBBY_PASS = 24;
constexpr int MAX_CHARS_CHAT_BOX = 120;