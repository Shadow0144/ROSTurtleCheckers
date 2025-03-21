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
        King
    };

    Tile(
        const std::vector<uint32_t> &reachableTilesBlack,
        const std::vector<uint32_t> &reachableTilesRed,
        const std::unordered_map<uint32_t, std::vector<uint32_t>> &reachableTilesBlackJump,
        const std::unordered_map<uint32_t, std::vector<uint32_t>> &reachableTilesRedJump,
        TurtlePieceColor turtlePieceColor);

    void setTurtlePieceColor(TurtlePieceColor turtlePieceColor);
    TurtlePieceColor getTurtlePieceColor();

    std::vector<uint32_t> getCurrentlyReachableTiles(std::vector<Tile> tiles); // Get the list of tiles the piece currently on this tile can reach (including by jumping)

private:
    std::vector<uint32_t> _reachableTilesBlack;
    std::vector<uint32_t> _reachableTilesRed;
    std::unordered_map<uint32_t, std::vector<uint32_t>> _reachableTilesBlackJump;
    std::unordered_map<uint32_t, std::vector<uint32_t>> _reachableTilesRedJump;
    TurtlePieceColor _turtlePieceColor;
};

typedef std::shared_ptr<Tile> TilePtr;