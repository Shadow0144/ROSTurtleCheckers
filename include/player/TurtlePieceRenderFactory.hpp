#pragma once

#include <vector>

#include "shared/CheckersConsts.hpp"
#include "player/TurtlePieceRender.hpp"
#include "player/TileRender.hpp"

class TurtlePieceRenderFactory
{
public:
    static std::vector<TurtlePieceRenderPtr> createTurtlePieceRenders(
        size_t numberOfPiecesPerPlayer,
        const std::vector<TileRenderPtr> &tileRenders,
        TurtlePieceColor playerColor);
};