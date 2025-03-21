#include "Tile.hpp"

Tile::Tile(
    const std::vector<uint32_t> &reachableTilesBlack,
    const std::vector<uint32_t> &reachableTilesRed,
    const std::unordered_map<uint32_t, std::vector<uint32_t>> &reachableTilesBlackJump,
    const std::unordered_map<uint32_t, std::vector<uint32_t>> &reachableTilesRedJump,
    TurtlePieceColor turtlePieceColor)
    : _reachableTilesBlack(reachableTilesBlack),
      _reachableTilesRed(reachableTilesRed),
      _reachableTilesBlackJump(reachableTilesBlackJump),
      _reachableTilesRedJump(reachableTilesRedJump),
      _turtlePieceColor(turtlePieceColor)
{
}