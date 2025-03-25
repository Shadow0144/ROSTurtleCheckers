#include "CheckersGameLobby.hpp"

#include "CheckersConsts.hpp"

using std::placeholders::_1;

CheckersGameLobby::CheckersGameLobby(
    const std::string &lobbyName,
    const std::string &blackPlayerName,
    const std::string &redPlayerName)
    : m_lobbyName(lobbyName),
      m_blackPlayerName(blackPlayerName),
      m_redPlayerName(redPlayerName)
{
    // Create the tiles
    for (size_t r = 0u; r < NUM_PLAYABLE_ROWS; r++)
    {
        for (size_t c = 0u; c < NUM_PLAYABLE_COLS; c++)
        {
            m_tiles.push_back(std::make_shared<Tile>(r, 2 * c + ((r + 1u) % 2))); // Every other column is offset by 1
        }
    }

    // Add the pieces
    for (size_t r = 0u; r < NUM_PIECES_PER_PLAYER; r++)
    {
        m_tiles[r]->setTurtlePiece(TurtlePieceColor::Red, "Red" + std::to_string(r + 1), false);
    }
    for (size_t b = 0u; b < NUM_PIECES_PER_PLAYER; b++)
    {
        m_tiles[b + BLACK_OFFSET]->setTurtlePiece(TurtlePieceColor::Black, "Black" + std::to_string(b + 1), false);
    }
}

std::vector<uint64_t> CheckersGameLobby::requestReachableTiles(const std::string &requestedPieceName)
{
    for (size_t i = 0u; i < NUM_PLAYABLE_TILES; i++)
    {
        if (m_tiles[i]->getTurtlePieceName() == requestedPieceName)
        {
            return m_tiles[i]->getCurrentlyReachableTiles(m_tiles);
        }
    }
    return {};
}

std::string CheckersGameLobby::getLobbyName()
{
    return m_lobbyName;
}

Winner CheckersGameLobby::getWinner()
{
    return m_winner;
}