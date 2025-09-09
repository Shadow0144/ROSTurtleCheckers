#include "player/TurtlePieceRenderFactory.hpp"

#include <QVector>
#include <QString>

#include <string>
#include <memory>
#include <vector>

#include "shared/CheckersConsts.hpp"

std::vector<TurtlePieceRenderPtr> TurtlePieceRenderFactory::createTurtlePieceRenders(
    size_t numberOfPiecesPerPlayer,
    const std::vector<TileRenderPtr> &tileRenders,
    TurtlePieceColor playerColor)
{
    std::vector<TurtlePieceRenderPtr> turtlePieceRenders;

    // Black
    for (size_t i = 0u; i < numberOfPiecesPerPlayer; i++)
    {
        std::string name = "Black" + std::to_string(i + 1);
        const auto &centerPosition = tileRenders[BLACK_OFFSET + i]->getCenterPosition();
        int angleDegrees = (playerColor == TurtlePieceColor::Black) ? UPWARD_ANGLE : DOWNWARD_ANGLE;
        auto blackTurtle = std::make_shared<TurtlePieceRender>(
            name, TurtlePieceColor::Black, centerPosition, angleDegrees);
        tileRenders[BLACK_OFFSET + i]->setTurtlePiece(blackTurtle);
        turtlePieceRenders.push_back(blackTurtle);
    }

    // Red
    for (size_t i = 0u; i < numberOfPiecesPerPlayer; i++)
    {
        std::string name = "Red" + std::to_string(i + 1);
        const auto &centerPosition = tileRenders[i]->getCenterPosition();
        int angleDegrees = (playerColor == TurtlePieceColor::Red) ? UPWARD_ANGLE : DOWNWARD_ANGLE;
        auto redTurtle = std::make_shared<TurtlePieceRender>(
            name, TurtlePieceColor::Red, centerPosition, angleDegrees);
            tileRenders[i]->setTurtlePiece(redTurtle);
        turtlePieceRenders.push_back(redTurtle);
    }

    return turtlePieceRenders;
}