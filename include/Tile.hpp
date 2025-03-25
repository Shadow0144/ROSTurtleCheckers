#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "CheckersConsts.hpp"

class Tile
{
public:
    Tile(size_t row, size_t col);

    size_t getRow() const;
    size_t getCol() const;

    void setTurtlePiece(const TurtlePieceColor &turtlePieceColor, const std::string &turtlePieceName, bool isTurtlePieceKinged);
    void moveTurtlePiece(std::shared_ptr<Tile> tile);

    TurtlePieceColor getTurtlePieceColor() const;
    std::string getTurtlePieceName() const;
    bool getIsTurtlePieceKinged() const;

    std::vector<uint64_t> getCurrentlyReachableTiles(const std::vector<std::shared_ptr<Tile>> &tiles) const; // Get the list of tiles the piece currently on this tile can reach (including by jumping)

private:
    void clearTurtlePiece();

    bool canJumpPiece(TurtlePieceColor otherPieceColor) const;
    void checkTilesAbove(const std::vector<std::shared_ptr<Tile>> &tiles, std::vector<uint64_t> &reachableTiles) const;
    void checkTilesBelow(const std::vector<std::shared_ptr<Tile>> &tiles, std::vector<uint64_t> &reachableTiles) const;

    size_t m_row;
    size_t m_col;

    TurtlePieceColor m_turtlePieceColor; // The color of the current piece (or None)
    std::string m_turtlePieceName;
    bool m_isTurtlePieceKinged;
};

typedef std::shared_ptr<Tile> TilePtr;