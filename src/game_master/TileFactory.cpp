#include "game_master/TileFactory.hpp"

#include <vector>

#include "shared/CheckersConsts.hpp"

std::vector<TilePtr> TileFactory::createTiles(size_t numRows, size_t numCols)
{
    std::vector<TilePtr> tiles;
    for (size_t r = 0u; r < numRows; r++)
    {
        for (size_t c = 0u; c < numCols; c++)
        {
            // Rows and columns start at 1
            tiles.push_back(std::make_shared<Tile>((r + 1u), (r % 2u == 0u) ? ((2u * c) + 2u) : ((2u * c)) + 1u));
        }
    }
    return tiles;
}