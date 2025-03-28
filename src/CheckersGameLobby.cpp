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

    m_blackPiecesRemaining = NUM_PIECES_PER_PLAYER;
    m_redPiecesRemaining = NUM_PIECES_PER_PLAYER;

    // Create the tiles
    for (size_t r = 0u; r < NUM_PLAYABLE_ROWS; r++)
    {
        for (size_t c = 0u; c < NUM_PLAYABLE_COLS; c++)
        {
            m_tiles.push_back(std::make_shared<Tile>(static_cast<int>(r), static_cast<int>(2u * c + ((r + 1u) % 2u)))); // Every other column is offset by 1
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

TurtlePieceColor CheckersGameLobby::getColorFromPieceName(const std::string &pieceName) const
{
    if (pieceName.rfind("Black", 0) == 0)
    {
        return TurtlePieceColor::Black;
    }
    else if (pieceName.rfind("Red", 0) == 0)
    {
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
            if (wasPieceKinged(requestedPieceName, destinationTileIndex))
            {
                m_tiles[destinationTileIndex]->kingTurtlePiece();
            }
            return true;
        }
        else
        {
            return false;
        }
    }
    return false;
}

int CheckersGameLobby::getJumpedPieceTileIndex(int sourceTileIndex, int destinationTileIndex) const
{
    if (sourceTileIndex > -1 &&
        destinationTileIndex > -1 &&
        sourceTileIndex < static_cast<int>(m_tiles.size()) &&
        destinationTileIndex < static_cast<int>(m_tiles.size()) &&
        std::abs(m_tiles[sourceTileIndex]->getRow() - m_tiles[destinationTileIndex]->getRow()) > 1) // Moved more than one row
    {
        if (m_tiles[sourceTileIndex]->getRow() < m_tiles[destinationTileIndex]->getRow()) // Jumped down
        {
            if (m_tiles[sourceTileIndex]->getCol() < m_tiles[destinationTileIndex]->getCol()) // Jumped right
            {
                return (sourceTileIndex + NUM_PLAYABLE_COLS);
            }
            else // Jumped left
            {
                return (sourceTileIndex + NUM_PLAYABLE_COLS - 1);
            }
        }
        else // Jumped up
        {
            if (m_tiles[sourceTileIndex]->getCol() < m_tiles[destinationTileIndex]->getCol()) // Jumped right
            {
                return (sourceTileIndex - NUM_PLAYABLE_COLS + 1);
            }
            else // Jumped left
            {
                return (sourceTileIndex - NUM_PLAYABLE_COLS);
            }
        }
    }
    return -1;
}

bool CheckersGameLobby::wasPieceKinged(const std::string &pieceName, int destinationTileIndex) const
{
    auto pieceColor = getColorFromPieceName(pieceName);
    switch (pieceColor)
    {
    case TurtlePieceColor::Black:
    {
        return (destinationTileIndex < static_cast<int>(NUM_PLAYABLE_COLS));
    }
    break;
    case TurtlePieceColor::Red:
    {
        return (destinationTileIndex >= static_cast<int>(NUM_PLAYABLE_TILES - NUM_PLAYABLE_COLS));
    }
    break;
    case TurtlePieceColor::None:
    {
        return false;
    }
    break;
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

void CheckersGameLobby::slayTurtleAtTileIndex(int tileIndex)
{
    switch (m_tiles[tileIndex]->getTurtlePieceColor())
    {
    case TurtlePieceColor::Black:
    {

        m_blackPiecesRemaining--;
    }
    break;
    case TurtlePieceColor::Red:
    {
        m_redPiecesRemaining--;
    }
    break;
    case TurtlePieceColor::None:
    {
        // Do nothing
    }
    break;
    }
    m_tiles[tileIndex]->clearTurtlePiece();
}