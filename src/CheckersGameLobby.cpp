#include "CheckersGameLobby.hpp"

constexpr size_t NUM_PLAYABLE_ROWS = 8u;
constexpr size_t NUM_PLAYABLE_COLS = 4u;
constexpr size_t NUM_PIECES_PER_PLAYER = 12u;
constexpr size_t BLACK_OFFSET = 20u;
constexpr size_t NUM_PLAYABLE_TILES = 32u;

using std::placeholders::_1;

CheckersGameLobby::CheckersGameLobby(
    const std::string &lobby_name,
    const std::string &black_player_name,
    const std::string &red_player_name)
    : lobby_name_(lobby_name),
      black_player_name_(black_player_name),
      red_player_name_(red_player_name)
{
    // Create the tiles
    for (size_t r = 0u; r < NUM_PLAYABLE_ROWS; r++)
    {
        for (size_t c = 0u; c < NUM_PLAYABLE_COLS; c++)
        {
            tiles_.push_back(std::make_shared<Tile>(r, 2 * c + ((r + 1u) % 2))); // Every other column is offset by 1
        }
    }

    // Add the pieces
    for (size_t r = 0u; r < NUM_PIECES_PER_PLAYER; r++)
    {
        tiles_[r]->setTurtlePiece(Tile::TurtlePieceColor::Red, "Red" + std::to_string(r + 1));
    }
    for (size_t b = 0u; b < NUM_PIECES_PER_PLAYER; b++)
    {
        tiles_[b + BLACK_OFFSET]->setTurtlePiece(Tile::TurtlePieceColor::Black, "Black" + std::to_string(b + 1));
    }
}

std::vector<uint64_t> CheckersGameLobby::requestReachableTiles(const std::string &requestedPieceName)
{
    for (size_t i = 0u; i < NUM_PLAYABLE_TILES; i++)
    {
        if (tiles_[i]->getTurtlePieceName() == requestedPieceName)
        {
            return tiles_[i]->getCurrentlyReachableTiles(tiles_);
        }
    }
    return {};
}

std::string CheckersGameLobby::getLobbyName()
{
    return lobby_name_;
}

CheckersGameLobby::Winner CheckersGameLobby::getWinner()
{
    return winner;
}