#include "shared/Tile.hpp"

#include <iostream>

#include "shared/CheckersConsts.hpp"
#include "shared/Hasher.hpp"

Tile::Tile(int row, int col)
    : m_row(row),
      m_col(col)
{
    m_isReachable = false;
    m_isHighlighted = false;
    m_isSelected = false;
    m_isLastMovedFrom = false;
    m_isLastMovedTo = false;
    m_isLastJumpedOver = false;
}

int Tile::getRow() const
{
    return m_row;
}

int Tile::getCol() const
{
    return m_col;
}

/// Tile statuses

bool Tile::getIsTileReachable() const
{
    return m_isReachable;
}

void Tile::setIsTileReachable(bool isReachable)
{
    m_isReachable = isReachable;
}

bool Tile::toggleIsTileReachable()
{
    m_isReachable = !m_isReachable;
    return m_isReachable;
}

bool Tile::getIsTileHighlighted() const
{
    return m_isHighlighted;
}

void Tile::setIsTileHighlighted(bool isHighlighted)
{
    m_isHighlighted = isHighlighted;
}

bool Tile::toggleIsTileHighlighted()
{
    m_isHighlighted = !m_isHighlighted;
    return m_isHighlighted;
}

bool Tile::getIsTileSelected() const
{
    return m_isSelected;
}

void Tile::setIsTileSelected(bool isSelected)
{
    m_isSelected = isSelected;
}

bool Tile::toggleIsTileSelected()
{
    m_isSelected = !m_isSelected;
    return m_isSelected;
}

bool Tile::getIsTileLastMovedFrom() const
{
    return m_isLastMovedFrom;
}

void Tile::setIsTileLastMovedFrom(bool isLastMovedFrom)
{
    m_isLastMovedFrom = isLastMovedFrom;
}

bool Tile::toggleIsTileLastMovedFrom()
{
    m_isLastMovedFrom = !m_isLastMovedFrom;
    return m_isLastMovedFrom;
}

bool Tile::getIsTileLastMovedTo() const
{
    return m_isLastMovedTo;
}

void Tile::setIsTileLastMovedTo(bool isLastMovedTo)
{
    m_isLastMovedTo = isLastMovedTo;
}

bool Tile::toggleIsTileLastMovedTo()
{
    m_isLastMovedTo = !m_isLastMovedTo;
    return m_isLastMovedTo;
}

bool Tile::getIsTileLastJumpedOver() const
{
    return m_isLastJumpedOver;
}

void Tile::setIsTileLastJumpedOver(bool isLastJumpedOver)
{
    m_isLastJumpedOver = isLastJumpedOver;
}

bool Tile::toggleIsTileLastJumpedOver()
{
    m_isLastJumpedOver = !m_isLastJumpedOver;
    return m_isLastJumpedOver;
}

/// Turtle piece statuses

void Tile::setTurtlePiece(const TurtlePiecePtr &turtle)
{
    m_containedTurtle = turtle;
}

const TurtlePiecePtr &Tile::getTurtlePiece() const
{
    return m_containedTurtle;
}

bool Tile::containsPiece(TurtlePieceColor color) const
{
    switch (color)
    {
    case TurtlePieceColor::Black:
    {
        return (m_containedTurtle && m_containedTurtle->getColor() == TurtlePieceColor::Black);
    }
    break;
    case TurtlePieceColor::Red:
    {
        return (m_containedTurtle && m_containedTurtle->getColor() == TurtlePieceColor::Red);
    }
    break;
    case TurtlePieceColor::None:
    {
        return (!m_containedTurtle || m_containedTurtle->getColor() == TurtlePieceColor::None);
    }
    break;
    }
    return false;
}

void Tile::clearTurtlePiece()
{
    m_containedTurtle.reset();
}

TurtlePieceColor Tile::getTurtlePieceColor() const
{
    return ((m_containedTurtle) ? m_containedTurtle->getColor() : TurtlePieceColor::None);
}

std::string Tile::getTurtlePieceName() const
{
    return ((m_containedTurtle) ? m_containedTurtle->getName() : "");
}

bool Tile::getIsTurtlePieceMovable() const
{
    if (m_containedTurtle)
    {
        return m_containedTurtle->getIsMovable();
    }
    return false;
}

void Tile::setIsTurtlePieceMovable(bool isMovable)
{
    if (m_containedTurtle)
    {
        m_containedTurtle->setIsMovable(isMovable);
    }
}

bool Tile::toggleIsTurtlePieceMovable()
{
    if (m_containedTurtle)
    {
        m_containedTurtle->toggleIsMovable();
        return m_containedTurtle->getIsMovable();
    }
    return false;
}

bool Tile::getIsTurtlePieceHighlighted() const
{
    if (m_containedTurtle)
    {
        return m_containedTurtle->getIsHighlighted();
    }
    return false;
}

void Tile::setIsTurtlePieceHighlighted(bool isHighlighted)
{
    if (m_containedTurtle)
    {
        m_containedTurtle->setIsHighlighted(isHighlighted);
    }
}

bool Tile::toggleIsTurtlePieceHighlighted()
{
    if (m_containedTurtle)
    {
        m_containedTurtle->toggleIsHighlighted();
        return m_containedTurtle->getIsHighlighted();
    }
    return false;
}

bool Tile::getIsTurtlePieceSelected() const
{
    if (m_containedTurtle)
    {
        return m_containedTurtle->getIsSelected();
    }
    return false;
}

void Tile::setIsTurtlePieceSelected(bool isSelected)
{
    if (m_containedTurtle)
    {
        m_containedTurtle->setIsSelected(isSelected);
    }
}

bool Tile::toggleIsTurtlePieceSelected()
{
    if (m_containedTurtle)
    {
        m_containedTurtle->toggleIsSelected();
        return m_containedTurtle->getIsSelected();
    }
    return false;
}

bool Tile::getIsTurtlePieceKinged() const
{
    if (m_containedTurtle)
    {
        return m_containedTurtle->getIsKinged();
    }
    return false;
}

void Tile::setIsTurtlePieceKinged(bool isKinged)
{
    if (m_containedTurtle)
    {
        m_containedTurtle->setIsKinged(isKinged);
    }
}

bool Tile::toggleIsTurtlePieceKinged()
{
    if (m_containedTurtle)
    {
        m_containedTurtle->toggleIsKinged();
        return m_containedTurtle->getIsKinged();
    }
    return false;
}

bool Tile::getIsTurtlePieceDead() const
{
    if (m_containedTurtle)
    {
        return m_containedTurtle->getIsDead();
    }
    return false;
}

void Tile::setIsTurtlePieceDead(bool isDead)
{
    if (m_containedTurtle)
    {
        m_containedTurtle->setIsDead(isDead);
    }
}

bool Tile::toggleIsTurtlePieceDead()
{
    if (m_containedTurtle)
    {
        m_containedTurtle->toggleIsDead();
        return m_containedTurtle->getIsDead();
    }
    return false;
}

void Tile::moveTurtlePiece(const TilePtr &destinationTile)
{
    destinationTile->setTurtlePiece(m_containedTurtle);
    clearTurtlePiece();
}

bool Tile::canJumpPiece(TurtlePieceColor otherPieceColor) const
{
    if (m_containedTurtle)
    {
        return ((m_containedTurtle->getColor() == TurtlePieceColor::Black && otherPieceColor == TurtlePieceColor::Red) ||
                (m_containedTurtle->getColor() == TurtlePieceColor::Red && otherPieceColor == TurtlePieceColor::Black));
    }
    else
    {
        return false;
    }
}

void Tile::checkTilesAbove(const std::vector<TilePtr> &tiles,
                           std::vector<uint64_t> &reachableTiles,
                           std::vector<uint64_t> &jumpableTiles) const
{
    // If a piece in this tile is an enemy piece or not
    bool topLeftJumpable = false;
    bool topRightJumpable = false;

    // The index of the tile past a jump (if it is open)
    // -1 means the tile doesn't exist or isn't empty
    int topLeftJumpableIndex = -1;
    int topRightJumpableIndex = -1;

    auto tileCount = static_cast<int>(tiles.size());
    for (int i = 0; i < tileCount; i++)
    {
        auto tileIColor = tiles[i]->getTurtlePieceColor();
        bool tileITurtleDead = tiles[i]->getIsTurtlePieceDead();

        // Check the tile to the top left of this one
        if (m_row - 1 == tiles[i]->m_row &&
            m_col - 1 == tiles[i]->m_col)
        {
            if (tileIColor == TurtlePieceColor::None)
            {
                reachableTiles.push_back(i);
            }
            else if (!tileITurtleDead && canJumpPiece(tileIColor))
            {
                topLeftJumpable = true;
            }
        }
        // Check the tile to the top right of this one
        else if (m_row - 1 == tiles[i]->m_row &&
                 m_col + 1 == tiles[i]->m_col)
        {
            if (tileIColor == TurtlePieceColor::None)
            {
                reachableTiles.push_back(i);
            }
            else if (!tileITurtleDead && canJumpPiece(tileIColor))
            {
                topRightJumpable = true;
            }
        }
        // Check if the tile is jumpable to the top left of this one
        else if (m_row - 2 == tiles[i]->m_row &&
                 m_col - 2 == tiles[i]->m_col)
        {
            if (tileIColor == TurtlePieceColor::None)
            {
                topLeftJumpableIndex = i;
            }
        }
        // Check if the tile is jumpable to the top right of this one
        else if (m_row - 2 == tiles[i]->m_row &&
                 m_col + 2 == tiles[i]->m_col)
        {
            if (tileIColor == TurtlePieceColor::None)
            {
                topRightJumpableIndex = i;
            }
        }
    }
    // Add any jumpable indices
    if (topLeftJumpable && topLeftJumpableIndex >= 0)
    {
        reachableTiles.push_back(topLeftJumpableIndex);
        jumpableTiles.push_back(topLeftJumpableIndex);
    }
    if (topRightJumpable && topRightJumpableIndex >= 0)
    {
        reachableTiles.push_back(topRightJumpableIndex);
        jumpableTiles.push_back(topRightJumpableIndex);
    }
}

void Tile::checkTilesBelow(const std::vector<TilePtr> &tiles,
                           std::vector<uint64_t> &reachableTiles,
                           std::vector<uint64_t> &jumpableTiles) const
{
    // If a piece in this tile is an enemy piece or not
    bool bottomLeftJumpable = false;
    bool bottomRightJumpable = false;

    // The index of the tile past a jump (if it is open)
    // -1 means the tile doesn't exist or isn't empty
    int bottomLeftJumpableIndex = -1;
    int bottomRightJumpableIndex = -1;

    auto tileCount = static_cast<int>(tiles.size());
    for (int i = 0; i < tileCount; i++)
    {
        auto tileIColor = tiles[i]->getTurtlePieceColor();
        bool tileITurtleDead = tiles[i]->getIsTurtlePieceDead();

        // Check the tile to the bottom left of this one
        if (m_row + 1 == tiles[i]->m_row &&
            m_col - 1 == tiles[i]->m_col)
        {
            if (tileIColor == TurtlePieceColor::None)
            {
                reachableTiles.push_back(i);
            }
            else if (!tileITurtleDead && canJumpPiece(tileIColor))
            {
                bottomLeftJumpable = true;
            }
        }
        // Check the tile to the bottom right of this one
        else if (m_row + 1 == tiles[i]->m_row &&
                 m_col + 1 == tiles[i]->m_col)
        {
            if (tileIColor == TurtlePieceColor::None)
            {
                reachableTiles.push_back(i);
            }
            else if (!tileITurtleDead && canJumpPiece(tileIColor))
            {
                bottomRightJumpable = true;
            }
        }
        // Check if the tile is jumpable to the bottom left of this one
        else if (m_row + 2 == tiles[i]->m_row &&
                 m_col - 2 == tiles[i]->m_col)
        {
            if (tileIColor == TurtlePieceColor::None)
            {
                bottomLeftJumpableIndex = i;
            }
        }
        // Check if the tile is jumpable to the top right of this one
        else if (m_row + 2 == tiles[i]->m_row &&
                 m_col + 2 == tiles[i]->m_col)
        {
            if (tileIColor == TurtlePieceColor::None)
            {
                bottomRightJumpableIndex = i;
            }
        }
    }
    // Add any jumpable indices
    if (bottomLeftJumpable && bottomLeftJumpableIndex >= 0)
    {
        reachableTiles.push_back(bottomLeftJumpableIndex);
        jumpableTiles.push_back(bottomLeftJumpableIndex);
    }
    if (bottomRightJumpable && bottomRightJumpableIndex >= 0)
    {
        reachableTiles.push_back(bottomRightJumpableIndex);
        jumpableTiles.push_back(bottomRightJumpableIndex);
    }
}

std::vector<uint64_t> Tile::getCurrentlyReachableTiles(const std::vector<TilePtr> &tiles, bool jumpsOnly) const
{
    std::vector<uint64_t> reachableTiles;
    std::vector<uint64_t> jumpableTiles;

    if (m_containedTurtle)
    {
        switch (m_containedTurtle->getColor())
        {
        case TurtlePieceColor::None:
            // Nothing is reachable
            break;
        case TurtlePieceColor::Black:
            // Black moves up the board and can jump red pieces
            checkTilesAbove(tiles, reachableTiles, jumpableTiles);
            if (m_containedTurtle->getIsKinged()) // Kings can move in the opposite direction too
            {
                checkTilesBelow(tiles, reachableTiles, jumpableTiles);
            }
            break;
        case TurtlePieceColor::Red:
            // Red moves down the board and can jump black pieces
            checkTilesBelow(tiles, reachableTiles, jumpableTiles);
            if (m_containedTurtle->getIsKinged()) // Kings can move in the opposite direction too
            {
                checkTilesAbove(tiles, reachableTiles, jumpableTiles);
            }
            break;
        }
    }

    if (jumpsOnly)
    {
        return jumpableTiles;
    }
    else
    {
        return reachableTiles;
    }
}

size_t std::hash<TilePtr>::operator()(const TilePtr &tilePtr) const noexcept
{
    size_t combinedHash = 0u;
    hashCombine(combinedHash, std::hash<int>{}(tilePtr->m_row));
    hashCombine(combinedHash, std::hash<int>{}(tilePtr->m_col));
    if (tilePtr->m_containedTurtle)
    {
        hashCombine(combinedHash, std::hash<TurtlePiecePtr>{}(tilePtr->m_containedTurtle));
    }
    else // No piece in tile, use default values
    {
        hashCombine(combinedHash, std::hash<std::string>{}(""));
        hashCombine(combinedHash, std::hash<uint64_t>{}(static_cast<uint64_t>(TurtlePieceColor::None)));
        hashCombine(combinedHash, std::hash<bool>{}(static_cast<uint64_t>(false)));
    }
    return combinedHash;
}