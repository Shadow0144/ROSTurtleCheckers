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

    void setTurtlePiece(const TurtlePieceColor &turtlePieceColor, const std::string &turtlePieceName);
    TurtlePieceColor getTurtlePieceColor() const;
    std::string getTurtlePieceName() const;

    std::vector<uint64_t> getCurrentlyReachableTiles(const std::vector<std::shared_ptr<Tile>> &tiles) const; // Get the list of tiles the piece currently on this tile can reach (including by jumping)

private:
    bool canJumpPiece(TurtlePieceColor otherPieceColor) const;
    void checkTilesAbove(const std::vector<std::shared_ptr<Tile>> &tiles, std::vector<uint64_t> &reachableTiles) const;
    void checkTilesBelow(const std::vector<std::shared_ptr<Tile>> &tiles, std::vector<uint64_t> &reachableTiles) const;

    size_t row_;
    size_t col_;

    TurtlePieceColor _turtlePieceColor; // The color of the current piece (or None)
    std::string _turtlePieceName;
};

typedef std::shared_ptr<Tile> TilePtr;