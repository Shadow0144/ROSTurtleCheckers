#include "CheckersPlayerWindow.hpp"

#include <QMainWindow>
#include <QVBoxLayout>

#include "CheckersPlayerNode.hpp"
#include "CheckersGameFrame.hpp"

CheckersPlayerWindow::CheckersPlayerWindow(const CheckersPlayerNodeWkPtr &playerNode,
                                           const std::string &playerName)
    : QMainWindow()
{
    m_playerNode = playerNode;

    setFixedSize(WINDOW_WIDTH, WINDOW_HEIGHT);
    setWindowTitle("TurtleCheckers");

    setMouseTracking(true);

    m_checkersGameFrame = std::make_shared<CheckersGameFrame>(weak_from_this(), playerName, this);
    setCentralWidget(m_checkersGameFrame.get());
}

const std::string &CheckersPlayerWindow::getLobbyName() const
{
    return m_checkersGameFrame->getLobbyName();
}

void CheckersPlayerWindow::connectedToGame(const std::string &lobbyName, TurtlePieceColor playerColor)
{
    m_checkersGameFrame->connectedToGame(lobbyName, playerColor);
}

void CheckersPlayerWindow::requestedPieceMoveAccepted(bool moveAccepted)
{
    m_checkersGameFrame->requestedPieceMoveAccepted(moveAccepted);
}

void CheckersPlayerWindow::requestedReachableTiles(const std::vector<size_t> &reachableTileIndices)
{
    m_checkersGameFrame->requestedReachableTiles(reachableTileIndices);
}

void CheckersPlayerWindow::declaredWinner(Winner winner)
{
    m_checkersGameFrame->declaredWinner(winner);
}

void CheckersPlayerWindow::gameStarted(GameState gameState, const std::vector<size_t> &movableTileIndices)
{
    m_checkersGameFrame->gameStarted(gameState, movableTileIndices);
}

void CheckersPlayerWindow::updatedBoard(size_t sourceTileIndex, size_t destinationTileIndex, GameState gameState,
                                        int slainPieceTileIndex, bool kingPiece, const std::vector<size_t> &movableTileIndices)
{
    m_checkersGameFrame->updatedBoard(sourceTileIndex, destinationTileIndex, gameState,
                                      slainPieceTileIndex, kingPiece, movableTileIndices);
}

void CheckersPlayerWindow::requestPieceMove(size_t sourceTileIndex, size_t destinationTileIndex)
{
    if (auto playerNode = m_playerNode.lock())
    {
        playerNode->requestPieceMove(sourceTileIndex, destinationTileIndex);
    }
}

void CheckersPlayerWindow::requestReachableTiles(size_t selectedPieceTileIndex)
{
    if (auto playerNode = m_playerNode.lock())
    {
        playerNode->requestReachableTiles(selectedPieceTileIndex);
    }
}

void CheckersPlayerWindow::update()
{
    m_checkersGameFrame->update();
}

// void CheckersPlayerWindow::onUpdate()
// {
//     // m_checkersGameFrame->update();
//     if (auto playerNode = m_playerNode.lock())
//     {
//         playerNode->onUpdate();
//     }
// }

std::shared_ptr<CheckersGameFrame> &CheckersPlayerWindow::getGameWindow()
{
    return m_checkersGameFrame;
}