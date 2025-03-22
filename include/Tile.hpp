#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

class Tile
{
public:
    enum class TurtlePieceColor
    {
        None,
        Black,
        Red,
        BlackKing,
        RedKing
    };

    Tile(size_t row, size_t col);

    size_t getRow() const;
    size_t getCol() const;

    void setTurtlePieceColor(const TurtlePieceColor &turtlePieceColor);
    TurtlePieceColor getTurtlePieceColor() const;

    std::vector<uint32_t> getCurrentlyReachableTiles(const std::vector<Tile> &tiles) const; // Get the list of tiles the piece currently on this tile can reach (including by jumping)

private:
    bool canJumpPiece(TurtlePieceColor otherPieceColor) const;
    void checkTilesAbove(const std::vector<Tile> &tiles, std::vector<uint32_t> &reachableTiles) const;
    void checkTilesBelow(const std::vector<Tile> &tiles, std::vector<uint32_t> &reachableTiles) const;

    size_t row_;
    size_t col_;

    TurtlePieceColor _turtlePieceColor; // The color of the current piece (or None)
};

typedef std::shared_ptr<Tile> TilePtr;