#include "CheckersGameLobby.hpp"

#include "CheckersConsts.hpp"

using std::placeholders::_1;

CheckersGameLobby::CheckersGameLobby(
    const std::string &lobbyName)
    : m_lobbyName(lobbyName)
{
    m_winner = Winner::None;
    m_isBlackTurn = true;
    
    m_blackPlayerName = "";
    m_redPlayerName = "";

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

bool CheckersGameLobby::playerSlotAvailable() const
{
    return (m_blackPlayerName.empty() || m_redPlayerName.empty());
}

TurtlePieceColor CheckersGameLobby::addPlayer(const std::string &playerName)
{
    if (m_blackPlayerName.empty())
    {
        m_blackPlayerName = playerName;
        return TurtlePieceColor::Black;
    }
    else if (m_redPlayerName.empty())
    {
        m_redPlayerName = playerName;
        return TurtlePieceColor::Red;
    }
    else
    {
        return TurtlePieceColor::None;
    }
}

bool CheckersGameLobby::isPieceValidForTurn(const std::string &requestedPieceName) const
{
    if (m_isBlackTurn)
    {
        return (requestedPieceName.rfind("Black", 0) == 0);
    }
    else
    {
        return (requestedPieceName.rfind("Red", 0) == 0);
    }
}

std::vector<uint64_t> CheckersGameLobby::requestReachableTiles(const std::string &requestedPieceName) const
{
    if (isPieceValidForTurn(requestedPieceName))
    {
        for (size_t i = 0u; i < NUM_PLAYABLE_TILES; i++)
        {
            if (m_tiles[i]->getTurtlePieceName() == requestedPieceName)
            {
                return m_tiles[i]->getCurrentlyReachableTiles(m_tiles);
            }
        }
    }
    return {};
}

bool CheckersGameLobby::requestPieceMove(const std::string &requestedPieceName, int sourceTileIndex, int destinationTileIndex)
{
    if (isPieceValidForTurn(requestedPieceName) &&
        sourceTileIndex > -1 &&
        destinationTileIndex > -1 &&
        sourceTileIndex < static_cast<int>(m_tiles.size()) &&
        destinationTileIndex < static_cast<int>(m_tiles.size()))
    {
        if (m_tiles[sourceTileIndex]->getTurtlePieceName() == requestedPieceName && m_tiles[destinationTileIndex]->getTurtlePieceName().empty())
        {
            m_tiles[sourceTileIndex]->moveTurtlePiece(m_tiles[destinationTileIndex]);
            return true;
        }
        else
        {
            return false;
        }
    }
    return false;
}

std::string CheckersGameLobby::getLobbyName() const
{
    return m_lobbyName;
}

Winner CheckersGameLobby::getWinner() const
{
    return m_winner;
}

void CheckersGameLobby::setIsBlackTurn(bool isBlackTurn)
{
    m_isBlackTurn = isBlackTurn;
}

bool CheckersGameLobby::getIsBlackTurn() const
{
    return m_isBlackTurn;
}


void CheckersGameLobby::togglePlayerTurn()
{
    m_isBlackTurn = !m_isBlackTurn;
}