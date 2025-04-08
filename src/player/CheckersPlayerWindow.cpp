#include "player/CheckersPlayerWindow.hpp"

#include <QMainWindow>

#include "shared/CheckersConsts.hpp"
#include "player/CheckersPlayerNode.hpp"
#include "player/CheckersMainMenuFrame.hpp"
#include "player/CheckersGameFrame.hpp"

CheckersPlayerWindow::CheckersPlayerWindow(const CheckersPlayerNodeWkPtr &playerNode)
    : QMainWindow()
{
    m_playerNode = playerNode;

    m_windowState = WindowState::MainMenu;

    setFixedSize(WINDOW_WIDTH, WINDOW_HEIGHT);
    setWindowTitle("TurtleCheckers");

    setMouseTracking(true);

    m_checkersMainMenuFrame = std::make_unique<CheckersMainMenuFrame>(this);
    setCentralWidget(m_checkersMainMenuFrame.get());
}

void CheckersPlayerWindow::closeEvent(QCloseEvent *event)
{
    if (auto playerNode = m_playerNode.lock())
    {
        playerNode->shutdown();
    }

    QMainWindow::closeEvent(event);
}

void CheckersPlayerWindow::setConnectedToServer(bool connected)
{
    m_connectedToServer = connected;
    if (m_checkersMainMenuFrame)
    {
        m_checkersMainMenuFrame->setConnectedToServer(connected);
    }
}

const std::string &CheckersPlayerWindow::getLobbyName() const
{
    return m_checkersGameFrame->getLobbyName();
}

void CheckersPlayerWindow::createLobby(const std::string &playerName,
                                       const std::string &lobbyName,
                                       TurtlePieceColor playerColor)
{
    if (auto playerNode = m_playerNode.lock())
    {
        playerNode->createLobby(playerName, lobbyName, playerColor);
    }
}

void CheckersPlayerWindow::joinLobby(const std::string &playerName,
                                     const std::string &lobbyName,
                                     TurtlePieceColor playerColor)
{
    if (auto playerNode = m_playerNode.lock())
    {
        playerNode->joinLobby(playerName, lobbyName, playerColor);
    }
}

void CheckersPlayerWindow::connectedToGame(
    const std::string &playerName,
    const std::string &lobbyName,
    TurtlePieceColor playerColor)
{
    m_checkersGameFrame = std::make_unique<CheckersGameFrame>(this, playerName);
    setCentralWidget(m_checkersGameFrame.get());
    m_checkersGameFrame->connectedToGame(lobbyName, playerColor);

    update();
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