#include "Tile.hpp"

Tile::Tile(
    const std::vector<uint32_t> &reachableTilesBlack,
    const std::vector<uint32_t> &reachableTilesRed,
    const std::unordered_map<uint32_t, std::vector<uint32_t>> &reachableTilesBlackJump,
    const std::unordered_map<uint32_t, std::vector<uint32_t>> &reachableTilesRedJump,
    Tile::TurtlePieceColor turtlePieceColor,
    Tile::TurtlePieceColor turtlePieceKinging)
    : _reachableTilesBlack(reachableTilesBlack),
      _reachableTilesRed(reachableTilesRed),
      _reachableTilesBlackJump(reachableTilesBlackJump),
      _reachableTilesRedJump(reachableTilesRedJump),
      _turtlePieceColor(turtlePieceColor),
      _turtlePieceKinging(turtlePieceKinging)
{
}

void Tile::setTurtlePieceColor(const TurtlePieceColor &turtlePieceColor)
{
    _turtlePieceColor = turtlePieceColor;
}

Tile::TurtlePieceColor Tile::getTurtlePieceColor()
{
    return _turtlePieceColor;
}

Tile::TurtlePieceColor Tile::getTurtlePieceKinging()
{
    return _turtlePieceKinging;
}

std::vector<uint32_t> getCurrentlyReachableTiles(const std::vector<Tile> &tiles)
{
    return std::vector<uint32_t>{};
}