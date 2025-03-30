#include "TileRenderFactory.hpp"

#include <vector>

#include "CheckersConsts.hpp"

std::vector<TileRenderPtr> TileRenderFactory::createTileRenders(
    size_t numRows, size_t numCols, TurtlePieceColor playerColor)
{
    std::vector<TileRenderPtr> tileRenders;

    float tileCenterX;
    float tileCenterY;
    for (size_t r = 0u; r < numRows; r++)
    {
        for (size_t c = 0u; c < numCols; c++)
        {
            if (r % 2u == 0u)
            {
                tileCenterX = ((1u + (2u * c)) * TILE_WIDTH) + TILE_HALF_WIDTH + GRAVEYARD_WIDTH;
            }
            else
            {
                tileCenterX = ((0u + (2u * c)) * TILE_WIDTH) + TILE_HALF_WIDTH + GRAVEYARD_WIDTH;
            }
            if (playerColor == TurtlePieceColor::Black)
            {
                tileCenterY = (r * TILE_HEIGHT) + TILE_HALF_HEIGHT + HUD_HEIGHT;
            }
            else // Red - The red player faces the board from the other direction, so mirror it
            {
                tileCenterY = WINDOW_HEIGHT - (r * TILE_HEIGHT) - TILE_HALF_HEIGHT;
            }
            // Rows and columns start at 1
            tileRenders.push_back(std::make_shared<TileRender>(
                (r + 1u), (r % 2u == 0u) ? ((2u * c) + 2u) : ((2u * c) + 1u), QPointF(tileCenterX, tileCenterY)));
        }
    }
    return tileRenders;
}