#include "player/CheckersPlayerWindow.hpp"

#include <QMainWindow>

#include <iostream>

#include "shared/CheckersConsts.hpp"
#include "player/CheckersPlayerNode.hpp"
#include "player/frame/CheckersMainMenuFrame.hpp"
#include "player/frame/CheckersGameFrame.hpp"

CheckersPlayerWindow::CheckersPlayerWindow(const CheckersPlayerNodeWkPtr &playerNode)
    : QMainWindow()
{
    m_playerNode = playerNode;

    m_windowState = WindowState::MainMenu;

    setFixedSize(WINDOW_WIDTH, WINDOW_HEIGHT);
    setWindowTitle("TurtleCheckers");

    setMouseTracking(true);

    m_checkersMainMenuFrame = new CheckersMainMenuFrame(this);
    setCentralWidget(m_checkersMainMenuFrame);

    m_checkersGameFrame = nullptr;
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

void CheckersPlayerWindow::createLobby(const std::string &playerName,
                                       const std::string &lobbyName,
                                       const std::string &lobbyPassword,
                                       TurtlePieceColor playerColor)
{
    if (auto playerNode = m_playerNode.lock())
    {
        playerNode->createLobby(playerName, lobbyName, lobbyPassword, playerColor);
    }
}

void CheckersPlayerWindow::joinLobby(const std::string &playerName,
                                     const std::string &lobbyName,
                                     const std::string &lobbyId,
                                     const std::string &lobbyPassword,
                                     TurtlePieceColor playerColor)
{
    if (auto playerNode = m_playerNode.lock())
    {
        playerNode->joinLobby(playerName, lobbyName, lobbyId, lobbyPassword, playerColor);
    }
}

void CheckersPlayerWindow::getLobbyList()
{
    if (auto playerNode = m_playerNode.lock())
    {
        playerNode->getLobbyList();
    }
}

void CheckersPlayerWindow::updateLobbyList(const std::vector<std::string> &lobbyNames,
                                           const std::vector<std::string> &lobbyIds,
                                           const std::vector<bool> &hasPasswords,
                                           const std::vector<std::string> &blackPlayerNames,
                                           const std::vector<std::string> &redPlayerNames)
{
    if (m_checkersMainMenuFrame)
    {
        m_checkersMainMenuFrame->displayLobbyList(lobbyNames,
                                                  lobbyIds,
                                                  hasPasswords,
                                                  blackPlayerNames,
                                                  redPlayerNames);
    }
}

void CheckersPlayerWindow::setPasswordIncorrect()
{
    if (m_checkersMainMenuFrame)
    {
        m_checkersMainMenuFrame->setPasswordIncorrect();
    }
}

void CheckersPlayerWindow::leaveLobby()
{
    if (auto playerNode = m_playerNode.lock())
    {
        playerNode->leaveLobby();
    }
}

void CheckersPlayerWindow::setReady(bool ready)
{
    if (auto playerNode = m_playerNode.lock())
    {
        playerNode->setReady(ready);
    }
}

void CheckersPlayerWindow::connectedToLobby(const std::string &lobbyName,
                                            const std::string &lobbyId,
                                            const std::string &blackPlayerName,
                                            const std::string &redPlayerName,
                                            bool blackPlayerReady,
                                            bool redPlayerReady)
{

    if (m_checkersMainMenuFrame)
    {
        m_checkersMainMenuFrame->connectedToLobby(lobbyName,
                                                  lobbyId,
                                                  blackPlayerName,
                                                  redPlayerName,
                                                  blackPlayerReady,
                                                  redPlayerReady);
    }
}

void CheckersPlayerWindow::playerJoinedLobby(const std::string &playerName, TurtlePieceColor playerColor)
{
    if (m_checkersMainMenuFrame)
    {
        m_checkersMainMenuFrame->playerJoinedLobby(playerName, playerColor);
    }
}

void CheckersPlayerWindow::playerLeftLobby(const std::string &playerName)
{
    if (m_checkersMainMenuFrame)
    {
        m_checkersMainMenuFrame->playerLeftLobby(playerName);
    }
}

void CheckersPlayerWindow::setPlayerReady(const std::string &playerName, bool ready)
{
    if (m_checkersMainMenuFrame)
    {
        m_checkersMainMenuFrame->setPlayerReady(playerName, ready);
    }
}

void CheckersPlayerWindow::requestedPieceMoveAccepted(bool moveAccepted)
{
    if (m_checkersGameFrame)
    {
        m_checkersGameFrame->requestedPieceMoveAccepted(moveAccepted);
    }
}

void CheckersPlayerWindow::requestedReachableTiles(const std::vector<size_t> &reachableTileIndices)
{
    if (m_checkersGameFrame)
    {
        m_checkersGameFrame->requestedReachableTiles(reachableTileIndices);
    }
}

void CheckersPlayerWindow::declaredWinner(Winner winner)
{
    if (m_checkersGameFrame)
    {
        m_checkersGameFrame->declaredWinner(winner);
    }
}

void CheckersPlayerWindow::gameStarted(GameState gameState,
                                       const std::string &lobbyName,
                                       const std::string &lobbyId,
                                       const std::string &playerName,
                                       TurtlePieceColor playerColor,
                                       const std::vector<size_t> &movableTileIndices)
{
    m_checkersGameFrame = new CheckersGameFrame(this, playerName);
    setCentralWidget(m_checkersGameFrame); // This will delete the main menu frame
    m_checkersMainMenuFrame = nullptr;

    m_checkersGameFrame->connectedToGame(lobbyName, lobbyId, playerColor);
    m_checkersGameFrame->gameStarted(gameState, movableTileIndices);

    update();
}

void CheckersPlayerWindow::updatedBoard(size_t sourceTileIndex, size_t destinationTileIndex,
                                        GameState gameState, int slainPieceTileIndex, bool kingPiece,
                                        const std::vector<size_t> &movableTileIndices)
{
    if (m_checkersGameFrame)
    {
        m_checkersGameFrame->updatedBoard(sourceTileIndex, destinationTileIndex, gameState,
                                          slainPieceTileIndex, kingPiece, movableTileIndices);
    }
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

void CheckersPlayerWindow::offerDraw()
{
    if (auto playerNode = m_playerNode.lock())
    {
        playerNode->offerDraw();
    }
}

void CheckersPlayerWindow::declineDraw()
{
    if (auto playerNode = m_playerNode.lock())
    {
        playerNode->declineDraw();
    }
}

void CheckersPlayerWindow::forfit()
{
    if (auto playerNode = m_playerNode.lock())
    {
        playerNode->forfit();
    }
}

void CheckersPlayerWindow::drawDeclined()
{
    if (m_checkersGameFrame)
    {
        m_checkersGameFrame->drawDeclined();
    }
}

void CheckersPlayerWindow::drawOffered()
{
    if (m_checkersGameFrame)
    {
        m_checkersGameFrame->drawOffered();
    }
}

void CheckersPlayerWindow::returnToMainMenu(const std::string &playerName)
{
    m_checkersMainMenuFrame = new CheckersMainMenuFrame(this);
    m_checkersMainMenuFrame->setPlayerName(playerName);
    setCentralWidget(m_checkersMainMenuFrame); // This will delete the game frame
    m_checkersGameFrame = nullptr;

    update();
}