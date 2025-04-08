#pragma once

#include <QMainWindow>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "shared/CheckersConsts.hpp"

class CheckersPlayerNode;
class CheckersGameFrame;

class CheckersPlayerWindow : public QMainWindow, public std::enable_shared_from_this<CheckersPlayerWindow>
{
    Q_OBJECT
public:
    CheckersPlayerWindow(const std::weak_ptr<CheckersPlayerNode> &playerNode,
                         const std::string &playerName);

    const std::string &getLobbyName() const;

    void connectedToGame(const std::string &lobbyName, TurtlePieceColor playerColor);
    void requestedPieceMoveAccepted(bool moveAccepted);
    void requestedReachableTiles(const std::vector<size_t> &reachableTileIndices);
    void declaredWinner(Winner winner);
    void gameStarted(GameState gameState, const std::vector<size_t> &movableTileIndices);
    void updatedBoard(size_t sourceTileIndex, size_t destinationTileIndex, GameState gameState,
                      int slainPieceTileIndex, bool kingPiece, const std::vector<size_t> &movableTileIndices);

    void requestPieceMove(size_t sourceTileIndex, size_t destinationTileIndex);
    void requestReachableTiles(size_t selectedPieceTileIndex);

    std::shared_ptr<CheckersGameFrame> &getGameWindow();

    void update();

private:
    std::weak_ptr<CheckersPlayerNode> m_playerNode;

    std::shared_ptr<CheckersGameFrame> m_checkersGameFrame;
};

typedef std::unique_ptr<CheckersPlayerWindow> CheckersPlayerWindowUniPtr;
typedef std::weak_ptr<CheckersPlayerWindow> CheckersPlayerWindowWkPtr;