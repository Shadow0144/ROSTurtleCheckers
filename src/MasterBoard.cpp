#include "MasterBoard.hpp"

#include "CheckersConsts.hpp"
#include "TileFactory.hpp"
#include "TurtlePieceFactory.hpp"

#include <iostream>

MasterBoard::MasterBoard()
{
    m_winner = Winner::None;

    m_blackPiecesRemaining = NUM_PIECES_PER_PLAYER;
    m_redPiecesRemaining = NUM_PIECES_PER_PLAYER;

    // Create the tiles
    m_tiles = TileFactory::createTiles(NUM_PLAYABLE_ROWS, NUM_PLAYABLE_COLS);

    // Create and add the pieces to the tiles
    TurtlePieceFactory::createTurtlePieces(NUM_PIECES_PER_PLAYER, m_tiles);
}

std::string MasterBoard::getPieceNameAtTileIndex(int tileIndex) const
{
    if (tileIndex > -1 && tileIndex < static_cast<int>(NUM_PLAYABLE_TILES))
    {
        return m_tiles[tileIndex]->getTurtlePieceName();
    }
    return "";
}

TurtlePieceColor MasterBoard::getPieceColorAtTileIndex(int tileIndex) const
{
    if (tileIndex > -1 && tileIndex < static_cast<int>(NUM_PLAYABLE_TILES))
    {
        return m_tiles[tileIndex]->getTurtlePieceColor();
    }
    return TurtlePieceColor::None;
}

TurtlePieceColor MasterBoard::getColorFromPieceName(const std::string &pieceName) const
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

std::vector<uint64_t> MasterBoard::requestReachableTiles(int tileIndex) const
{
    if (tileIndex > -1 &&
        tileIndex < static_cast<int>(m_tiles.size()))
    {
        return m_tiles[tileIndex]->getCurrentlyReachableTiles(m_tiles, m_mustJump);
    }
    else
    {
        return {};
    }
}

std::vector<uint64_t> MasterBoard::requestJumpableTiles(int tileIndex) const
{
    if (tileIndex > -1 &&
        tileIndex < static_cast<int>(m_tiles.size()))
    {
        return m_tiles[tileIndex]->getCurrentlyReachableTiles(m_tiles, true);
    }
    else
    {
        return {};
    }
}

bool MasterBoard::requestPieceMove(int sourceTileIndex, int destinationTileIndex)
{
    if (sourceTileIndex > -1 &&
        destinationTileIndex > -1 &&
        sourceTileIndex < static_cast<int>(m_tiles.size()) &&
        destinationTileIndex < static_cast<int>(m_tiles.size()))
    {
        if (m_tiles[destinationTileIndex]->getTurtlePieceName().empty())
        {
            m_tiles[sourceTileIndex]->moveTurtlePiece(m_tiles[destinationTileIndex]);
            if (wasPieceKinged(destinationTileIndex))
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

int MasterBoard::getJumpedPieceTileIndex(int sourceTileIndex, int destinationTileIndex) const
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

bool MasterBoard::wasPieceKinged(int tileIndex) const
{
    auto pieceColor = m_tiles[tileIndex]->getTurtlePieceColor();
    switch (pieceColor)
    {
    case TurtlePieceColor::Black:
    {
        return (tileIndex < static_cast<int>(NUM_PLAYABLE_COLS));
    }
    break;
    case TurtlePieceColor::Red:
    {
        return (tileIndex >= static_cast<int>(NUM_PLAYABLE_TILES - NUM_PLAYABLE_COLS));
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

Winner MasterBoard::getWinner() const
{
    return m_winner;
}

bool MasterBoard::getMustJump() const
{
    return m_mustJump;
}

void MasterBoard::setMustJump(bool mustJump)
{
    m_mustJump = mustJump;
}

void MasterBoard::addTileToJumpedTileIndices(int tileIndex)
{
    // Slay the jumped turtle and update the piece count
    m_tiles[tileIndex]->setIsTurtlePieceDead(true);
    m_jumpedPieceTileIndices.push_back(tileIndex);
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

    // If all the pieces of a player are slain, the other player wins
    if (m_blackPiecesRemaining == 0u)
    {
        m_winner = Winner::Red;
    }
    else if (m_redPiecesRemaining == 0u)
    {
        m_winner = Winner::Black;
    }
}

void MasterBoard::slayTurtlesAtJumpedTileIndices()
{
    // After switching turns, remove the slain turtles from the battlefield
    for (auto tileIndex : m_jumpedPieceTileIndices)
    {
        m_tiles[tileIndex]->clearTurtlePiece();
    }
    m_jumpedPieceTileIndices.clear();
}

bool MasterBoard::canJumpAgainFromTileIndex(int tileIndex)
{
    return (!m_tiles[tileIndex]->getCurrentlyReachableTiles(m_tiles, true).empty());
}

void MasterBoard::checkPlayersCanMove(bool isBlackTurn, std::vector<size_t> &movableTileIndices)
{
    if (m_blackPiecesRemaining == 0 || m_redPiecesRemaining == 0)
    {
        return; // The game is over, no one can move anymore
    }

    // Check if both players have valid moves
    // If they do not, check for a draw
    // If only one can't move, that one loses; if both can't, it's a draw
    bool blackPlayerHasMoves = false;
    bool redPlayerHasMoves = false;

    for (size_t i = 0u; i < NUM_PLAYABLE_TILES; i++)
    {
        auto &tile = m_tiles[i];
        if (!tile->getCurrentlyReachableTiles(m_tiles).empty())
        {
            if (tile->getTurtlePieceColor() == TurtlePieceColor::Black)
            {
                blackPlayerHasMoves = true;
                if (isBlackTurn)
                {
                    movableTileIndices.push_back(i);
                }
            }
            else if (tile->getTurtlePieceColor() == TurtlePieceColor::Red)
            {
                redPlayerHasMoves = true;
                if (!isBlackTurn)
                {
                    movableTileIndices.push_back(i);
                }
            }
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
    else if (!redPlayerHasMoves)
    {
        m_winner = Winner::Black;
    }
}