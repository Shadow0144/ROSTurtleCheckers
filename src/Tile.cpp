#include "Tile.hpp"

#include <iostream>

#include "CheckersConsts.hpp"

Tile::Tile(size_t row, size_t col)
    : m_row(row),
      m_col(col)
{
    m_turtlePieceColor = TurtlePieceColor::None;
}

void Tile::setTurtlePiece(const TurtlePieceColor &turtlePieceColor, const std::string &turtlePieceName, bool isTurtlePieceKinged)
{
    m_turtlePieceColor = turtlePieceColor;
    m_turtlePieceName = turtlePieceName;
    m_isTurtlePieceKinged = isTurtlePieceKinged;
}

TurtlePieceColor Tile::getTurtlePieceColor() const
{
    return m_turtlePieceColor;
}

std::string Tile::getTurtlePieceName() const
{
    return m_turtlePieceName;
}

bool Tile::getIsTurtlePieceKinged() const
{
    return m_isTurtlePieceKinged;
}

size_t Tile::getRow() const
{
    return m_row;
}

size_t Tile::getCol() const
{
    return m_col;
}

bool Tile::canJumpPiece(TurtlePieceColor otherPieceColor) const
{
    return ((m_turtlePieceColor == TurtlePieceColor::Black && otherPieceColor == TurtlePieceColor::Red) ||
            (m_turtlePieceColor == TurtlePieceColor::Red && otherPieceColor == TurtlePieceColor::Black));
}

void Tile::checkTilesAbove(const std::vector<TilePtr> &tiles, std::vector<uint64_t> &reachableTiles) const
{
    // If a piece in this tile is an enemy piece or not
    bool topLeftJumpable = false;
    bool topRightJumpable = false;

    // The index of the tile past a jump (if it is open)
    // -1 means the tile doesn't exist or isn't empty
    int topLeftJumpableIndex = -1;
    int topRightJumpableIndex = -1;

    auto tileCount = tiles.size();
    for (uint64_t i = 0u; i < tileCount; i++)
    {
        // Check the tile to the top left of this one
        if (m_row > 0u && m_col > 0u &&
            m_row - 1u == tiles[i]->m_row &&
            m_col - 1u == tiles[i]->m_col)
        {
            if (tiles[i]->m_turtlePieceColor == TurtlePieceColor::None)
            {
                reachableTiles.push_back(i);
            }
            else if (canJumpPiece(tiles[i]->m_turtlePieceColor))
            {
                topLeftJumpable = true;
            }
        }
        // Check the tile to the top right of this one
        else if (m_row > 0u && m_col < MAX_COL_ROW_INDEX &&
                 m_row - 1u == tiles[i]->m_row &&
                 m_col + 1u == tiles[i]->m_col)
        {
            if (tiles[i]->m_turtlePieceColor == TurtlePieceColor::None)
            {
                reachableTiles.push_back(i);
            }
            else if (canJumpPiece(tiles[i]->m_turtlePieceColor))
            {
                topRightJumpable = true;
            }
        }
        // Check if the tile is jumpable to the top left of this one
        else if (m_row > 1u && m_col > 1u &&
                 m_row - 2u == tiles[i]->m_row &&
                 m_col - 2u == tiles[i]->m_col)
        {
            if (tiles[i]->m_turtlePieceColor == TurtlePieceColor::None)
            {
                topLeftJumpableIndex = i;
            }
        }
        // Check if the tile is jumpable to the top right of this one
        else if (m_row > 1u && m_col < MAX_JUMP_INDEX &&
                 m_row - 2u == tiles[i]->m_row &&
                 m_col + 2u == tiles[i]->m_col)
        {
            if (tiles[i]->m_turtlePieceColor == TurtlePieceColor::None)
            {
                topRightJumpableIndex = i;
            }
        }
    }
    // Add any jumpable indices
    if (topLeftJumpable && topLeftJumpableIndex >= 0)
    {
        reachableTiles.push_back(topLeftJumpableIndex);
    }
    if (topRightJumpable && topRightJumpableIndex >= 0)
    {
        reachableTiles.push_back(topRightJumpableIndex);
    }
}

void Tile::checkTilesBelow(const std::vector<TilePtr> &tiles, std::vector<uint64_t> &reachableTiles) const
{
    // If a piece in this tile is an enemy piece or not
    bool bottomLeftJumpable = false;
    bool bottomRightJumpable = false;

    // The index of the tile past a jump (if it is open)
    // -1 means the tile doesn't exist or isn't empty
    int bottomLeftJumpableIndex = -1;
    int bottomRightJumpableIndex = -1;

    auto tileCount = tiles.size();
    for (uint64_t i = 0u; i < tileCount; i++)
    {
        // Check the tile to the bottom left of this one
        if (m_row < MAX_COL_ROW_INDEX && m_col > 0u &&
            m_row + 1u == tiles[i]->m_row &&
            m_col - 1u == tiles[i]->m_col)
        {
            if (tiles[i]->m_turtlePieceColor == TurtlePieceColor::None)
            {
                reachableTiles.push_back(i);
            }
            else if (canJumpPiece(tiles[i]->m_turtlePieceColor))
            {
                bottomLeftJumpable = true;
            }
        }
        // Check the tile to the bottom right of this one
        else if (m_row < MAX_COL_ROW_INDEX && m_col < MAX_COL_ROW_INDEX &&
                 m_row + 1u == tiles[i]->m_row &&
                 m_col + 1u == tiles[i]->m_col)
        {
            if (tiles[i]->m_turtlePieceColor == TurtlePieceColor::None)
            {
                reachableTiles.push_back(i);
            }
            else if (canJumpPiece(tiles[i]->m_turtlePieceColor))
            {
                bottomRightJumpable = true;
            }
        }
        // Check if the tile is jumpable to the bottom left of this one
        else if (m_row < MAX_JUMP_INDEX && m_col > 1u &&
                 m_row + 2u == tiles[i]->m_row &&
                 m_col - 2u == tiles[i]->m_col)
        {
            if (tiles[i]->m_turtlePieceColor == TurtlePieceColor::None)
            {
                bottomLeftJumpableIndex = i;
            }
        }
        // Check if the tile is jumpable to the top right of this one
        else if (m_row < MAX_JUMP_INDEX && m_col < MAX_JUMP_INDEX &&
                 m_row + 2u == tiles[i]->m_row &&
                 m_col + 2u == tiles[i]->m_col)
        {
            if (tiles[i]->m_turtlePieceColor == TurtlePieceColor::None)
            {
                bottomRightJumpableIndex = i;
            }
        }
    }
    // Add any jumpable indices
    if (bottomLeftJumpable && bottomLeftJumpableIndex >= 0)
    {
        reachableTiles.push_back(bottomLeftJumpableIndex);
    }
    if (bottomRightJumpable && bottomRightJumpableIndex >= 0)
    {
        reachableTiles.push_back(bottomRightJumpableIndex);
    }
}

std::vector<uint64_t> Tile::getCurrentlyReachableTiles(const std::vector<TilePtr> &tiles) const
{
    std::vector<uint64_t> reachableTiles;

    switch (m_turtlePieceColor)
    {
    case TurtlePieceColor::None:
        // Nothing is reachable
        break;
    case TurtlePieceColor::Black:
        // Black moves up the board and can jump red pieces
        checkTilesAbove(tiles, reachableTiles);
        if (m_isTurtlePieceKinged) // Kings can move in the opposite direction too
        {
            checkTilesBelow(tiles, reachableTiles);
        }
        break;
    case TurtlePieceColor::Red:
        // Red moves down the board and can jump black pieces
        checkTilesBelow(tiles, reachableTiles);
        if (m_isTurtlePieceKinged) // Kings can move in the opposite direction too
        {
            checkTilesAbove(tiles, reachableTiles);
        }
        break;
    }

    return reachableTiles;
}