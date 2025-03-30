#include "TileFactory.hpp"

#include <vector>

#include "CheckersConsts.hpp"

std::vector<TilePtr> TileFactory::createTiles(size_t numRows, size_t numCols)
{
    std::vector<TilePtr> tiles;
    for (size_t i = 0u; i < numRows; i++)
    {
        for (size_t j = 0u; j < numCols; j++)
        {
            tiles.push_back(std::make_shared<Tile>(i, j));
        }
    }
    return tiles;
}