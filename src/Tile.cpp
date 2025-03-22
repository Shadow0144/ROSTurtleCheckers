#include "Tile.hpp"

constexpr size_t MAX_COLS_ROW_INDEX = 7u;
constexpr size_t MAX_JUMP_INDEX = MAX_COLS_ROW_INDEX - 1u;

Tile::Tile(size_t row, size_t col)
    : row_(row),
      col_(col)
{
}

void Tile::setTurtlePieceColor(const TurtlePieceColor &turtlePieceColor)
{
    _turtlePieceColor = turtlePieceColor;
}

Tile::TurtlePieceColor Tile::getTurtlePieceColor() const
{
    return _turtlePieceColor;
}

size_t Tile::getRow() const
{
    return row_;
}

size_t Tile::getCol() const
{
    return col_;
}

bool Tile::canJumpPiece(TurtlePieceColor otherPieceColor) const
{
    return (((_turtlePieceColor == TurtlePieceColor::Black || _turtlePieceColor == TurtlePieceColor::BlackKing) &&
            (otherPieceColor == TurtlePieceColor::Red || otherPieceColor == TurtlePieceColor::RedKing)) || 
            ((_turtlePieceColor == TurtlePieceColor::Red || _turtlePieceColor == TurtlePieceColor::RedKing) &&
            (otherPieceColor == TurtlePieceColor::Black || otherPieceColor == TurtlePieceColor::BlackKing)));
}

void Tile::checkTilesAbove(const std::vector<Tile> &tiles, std::vector<uint32_t> &reachableTiles) const
{
    // If a piece in this tile is an enemy piece or not
    bool topLeftJumpable = false;
    bool topRightJumpable = false;

    // The index of the tile past a jump (if it is open)
    // -1 means the tile doesn't exist or isn't empty
    int topLeftJumpableIndex = -1;
    int topRightJumpableIndex = -1;

    for (int i = 0; i < tiles.size(); i++)
    {
        // Check the tile to the top left of this one
        if (row_ > 0u && col_ > 0u &&
            row_ - 1u == tiles[i].row_ &&
            col_ - 1u == tiles[i].col_)
        {
            if (tiles[i]._turtlePieceColor == TurtlePieceColor::None)
            {
                reachableTiles.push_back(i);
            }
            else if (canJumpPiece(tiles[i]._turtlePieceColor))
            {
                topLeftJumpable = true;
            }
        }
        // Check the tile to the top right of this one
        else if (row_ > 0u && col_ < MAX_COLS_ROW_INDEX &&
                 row_ - 1u == tiles[i].row_ &&
                 col_ + 1u == tiles[i].col_)
        {
            if (tiles[i]._turtlePieceColor == TurtlePieceColor::None)
            {
                reachableTiles.push_back(i);
            }
            else if (tiles[i]._turtlePieceColor == TurtlePieceColor::Red ||
                     tiles[i]._turtlePieceColor == TurtlePieceColor::RedKing)
            {
                topRightJumpable = true;
            }
        }
        // Check if the tile is jumpable to the top left of this one
        else if (row_ > 1u && col_ > 1u &&
                 row_ - 2u == tiles[i].row_ &&
                 col_ - 2u == tiles[i].col_)
        {
            if (tiles[i]._turtlePieceColor == TurtlePieceColor::None)
            {
                topLeftJumpableIndex = i;
            }
        }
        // Check if the tile is jumpable to the top right of this one
        else if (row_ > 1u && col_ < MAX_JUMP_INDEX &&
                 row_ - 2u == tiles[i].row_ &&
                 col_ + 2u == tiles[i].col_)
        {
            if (tiles[i]._turtlePieceColor == TurtlePieceColor::None)
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

void Tile::checkTilesBelow(const std::vector<Tile> &tiles, std::vector<uint32_t> &reachableTiles) const
{
    // If a piece in this tile is an enemy piece or not
    bool bottomLeftJumpable = false;
    bool bottomRightJumpable = false;

    // The index of the tile past a jump (if it is open)
    // -1 means the tile doesn't exist or isn't empty
    int bottomLeftJumpableIndex = -1;
    int bottomRightJumpableIndex = -1;

    for (int i = 0; i < tiles.size(); i++)
    {
        // Check the tile to the bottom left of this one
        if (row_ < MAX_COLS_ROW_INDEX && col_ > 0u &&
            row_ + 1u == tiles[i].row_ &&
            col_ - 1u == tiles[i].col_)
        {
            if (tiles[i]._turtlePieceColor == TurtlePieceColor::None)
            {
                reachableTiles.push_back(i);
            }
            else if (canJumpPiece(tiles[i]._turtlePieceColor))
            {
                bottomLeftJumpable = true;
            }
        }
        // Check the tile to the bottom right of this one
        else if (row_ < MAX_COLS_ROW_INDEX && col_ < MAX_COLS_ROW_INDEX &&
                 row_ + 1u == tiles[i].row_ &&
                 col_ + 1u == tiles[i].col_)
        {
            if (tiles[i]._turtlePieceColor == TurtlePieceColor::None)
            {
                reachableTiles.push_back(i);
            }
            else if (tiles[i]._turtlePieceColor == TurtlePieceColor::Red ||
                     tiles[i]._turtlePieceColor == TurtlePieceColor::RedKing)
            {
                bottomRightJumpable = true;
            }
        }
        // Check if the tile is jumpable to the bottom left of this one
        else if (row_ < MAX_JUMP_INDEX && col_ > 1u &&
                 row_ + 2u == tiles[i].row_ &&
                 col_ - 2u == tiles[i].col_)
        {
            if (tiles[i]._turtlePieceColor == TurtlePieceColor::None)
            {
                bottomLeftJumpableIndex = i;
            }
        }
        // Check if the tile is jumpable to the top right of this one
        else if (row_ < MAX_JUMP_INDEX && col_ < MAX_JUMP_INDEX &&
                 row_ + 2u == tiles[i].row_ &&
                 col_ + 2u == tiles[i].col_)
        {
            if (tiles[i]._turtlePieceColor == TurtlePieceColor::None)
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

std::vector<uint32_t> Tile::getCurrentlyReachableTiles(const std::vector<Tile> &tiles) const
{
    std::vector<uint32_t> reachableTiles;

    switch (_turtlePieceColor)
    {
    case TurtlePieceColor::None:
        // Nothing is reachable
        break;
    case TurtlePieceColor::Black:
        // Black moves up the board and can jump red (and red king) pieces
        checkTilesAbove(tiles, reachableTiles);
        break;
    case TurtlePieceColor::BlackKing:
        // Black kings can move in either direction and can jump red (and red king) pieces
        checkTilesAbove(tiles, reachableTiles);
        checkTilesBelow(tiles, reachableTiles);
        break;
    case TurtlePieceColor::Red:
        // Red moves down the board and can jump black (and black king) pieces
        checkTilesBelow(tiles, reachableTiles);
        break;
    case TurtlePieceColor::RedKing:
        // Red kings can move in either direction and can jump black (and black king) pieces
        checkTilesAbove(tiles, reachableTiles);
        checkTilesBelow(tiles, reachableTiles);
        break;
    }

    return reachableTiles;
}