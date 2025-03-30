#include "CheckersGameLobby.hpp"

#include "CheckersConsts.hpp"
#include "TileFactory.hpp"
#include "TurtlePieceFactory.hpp"

#include <iostream>

CheckersGameLobby::CheckersGameLobby(const std::string &lobbyName)
    : m_lobbyName(lobbyName)
{
    m_winner = Winner::None;
    m_isBlackTurn = true;

    m_blackPlayerName = "";
    m_redPlayerName = "";

    m_blackPiecesRemaining = NUM_PIECES_PER_PLAYER;
    m_redPiecesRemaining = NUM_PIECES_PER_PLAYER;

    // Create the tiles
    m_tiles = TileFactory::createTiles(NUM_PLAYABLE_ROWS, NUM_PLAYABLE_COLS);

    // Add the pieces
    m_turtlePieces = TurtlePieceFactory::createTurtlePieces(NUM_PIECES_PER_PLAYER, m_tiles);
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
                m_tiles[destinationTileIndex]->setIsTurtlePieceKinged(true);
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
        if (m_tiles[sourceTileIndex]->getRow() % 2 == 0) // Finding the index will depend on if the row is shifted or not
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
                    return (sourceTileIndex - NUM_PLAYABLE_COLS);
                }
                else // Jumped left
                {
                    return (sourceTileIndex - NUM_PLAYABLE_COLS - 1);
                }
            }
        }
        else // (m_tiles[sourceTileIndex]->getRow() % 2 == 1)
        {
            if (m_tiles[sourceTileIndex]->getRow() < m_tiles[destinationTileIndex]->getRow()) // Jumped down
            {
                if (m_tiles[sourceTileIndex]->getCol() < m_tiles[destinationTileIndex]->getCol()) // Jumped right
                {
                    return (sourceTileIndex + NUM_PLAYABLE_COLS + 1);
                }
                else // Jumped left
                {
                    return (sourceTileIndex + NUM_PLAYABLE_COLS);
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

    // If all the pieces of a player are slain, the other player wins
    if (m_blackPiecesRemaining == 0)
    {
        m_winner = Winner::Red;
    }
    else if (m_redPiecesRemaining == 0)
    {
        m_winner = Winner::Black;
    }
}

void CheckersGameLobby::checkPlayersCanMove()
{
    // Check if both players have valid moves
    // If they do not, check for a draw
    // If only one can't move, that one loses; if both can't, it's a draw
    bool blackPlayerHasMoves = false;
    bool redPlayerHasMoves = false;

    for (const auto &tile : m_tiles)
    {
        if (!tile->getCurrentlyReachableTiles(m_tiles).empty())
        {
            if (tile->getTurtlePieceColor() == TurtlePieceColor::Black)
            {
                blackPlayerHasMoves = true;
            }
            else if (tile->getTurtlePieceColor() == TurtlePieceColor::Red)
            {
                redPlayerHasMoves = true;
            }
        }
        if (blackPlayerHasMoves && redPlayerHasMoves)
        {
            return; // No need to update the winner
        }
    }

    if (!blackPlayerHasMoves && !redPlayerHasMoves)
    {
        m_winner = Winner::Draw;
    }
    else if (!blackPlayerHasMoves)
    {
        m_winner = Winner::Red;
    }
    else // if (!redPlayerHasMoves)
    {
        m_winner = Winner::Black;
    }
}