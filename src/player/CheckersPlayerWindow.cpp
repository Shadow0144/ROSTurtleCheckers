#include "player/CheckersPlayerWindow.hpp"

#include <QMainWindow>

#include <chrono>
#include <iostream>

#include "shared/CheckersConsts.hpp"
#include "player/Parameters.hpp"
#include "player/CheckersPlayerNode.hpp"
#include "player/frame/MainMenuFrame.hpp"
#include "player/frame/GameFrame.hpp"

CheckersPlayerWindow::CheckersPlayerWindow(const CheckersPlayerNodeWkPtr &playerNode)
    : QMainWindow()
{
    m_playerNode = playerNode;

    m_windowState = WindowState::MainMenu;

    setFixedSize(WINDOW_WIDTH, WINDOW_HEIGHT);
    setWindowTitle("TurtleCheckers");

    setMouseTracking(true);

    QWidget *windowLayoutWidget = new QWidget(this);
    m_windowLayout = new QStackedLayout(windowLayoutWidget);

    // Create and add the screens to the layout
    m_createAccountFrame = new CreateAccountFrame(this);
    m_createLobbyFrame = new CreateLobbyFrame(this);
    m_gameFrame = new GameFrame(this);
    m_inLobbyFrame = new InLobbyFrame(this);
    m_lobbyListFrame = new LobbyListFrame(this);
    m_lobbyPasswordFrame = new LobbyPasswordFrame(this);
    m_logInAccountFrame = new LogInAccountFrame(this);
    m_mainMenuFrame = new MainMenuFrame(this);
    m_titleFrame = new TitleFrame(this);

    m_windowLayout->insertWidget(CREATE_ACCOUNT_INDEX, m_createAccountFrame);
    m_windowLayout->insertWidget(CREATE_LOBBY_INDEX, m_createLobbyFrame);
    m_windowLayout->insertWidget(GAME_INDEX, m_gameFrame);
    m_windowLayout->insertWidget(IN_LOBBY_INDEX, m_inLobbyFrame);
    m_windowLayout->insertWidget(LOBBY_LIST_INDEX, m_lobbyListFrame);
    m_windowLayout->insertWidget(LOBBY_PASSWORD_INDEX, m_lobbyPasswordFrame);
    m_windowLayout->insertWidget(LOG_IN_ACCOUNT_INDEX, m_logInAccountFrame);
    m_windowLayout->insertWidget(MAIN_MENU_INDEX, m_mainMenuFrame);
    m_windowLayout->insertWidget(TITLE_INDEX, m_titleFrame);

    m_windowLayout->setCurrentIndex(TITLE_INDEX);

    setCentralWidget(windowLayoutWidget);
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
    m_titleFrame->setConnectedToServer(connected);
    if (!m_connectedToServer)
    {
        moveToTitleFrame();
    }
}

void CheckersPlayerWindow::moveToTitleFrame()
{
    m_windowLayout->setCurrentIndex(TITLE_INDEX);
}

void CheckersPlayerWindow::moveToCreateAccountFrame()
{
    m_windowLayout->setCurrentIndex(CREATE_ACCOUNT_INDEX);
}

void CheckersPlayerWindow::moveToLogInAccountFrame()
{
    m_windowLayout->setCurrentIndex(LOG_IN_ACCOUNT_INDEX);
}

void CheckersPlayerWindow::moveToMainMenuFrame()
{
    m_windowLayout->setCurrentIndex(MAIN_MENU_INDEX);
}

void CheckersPlayerWindow::moveToCreateLobbyFrame()
{
    m_windowLayout->setCurrentIndex(CREATE_LOBBY_INDEX);
}

void CheckersPlayerWindow::moveToLobbyListFrame()
{
    m_windowLayout->setCurrentIndex(LOBBY_LIST_INDEX);
}

void CheckersPlayerWindow::moveToLobbyPasswordFrame()
{
    m_windowLayout->setCurrentIndex(LOBBY_PASSWORD_INDEX);
}

void CheckersPlayerWindow::moveToInLobbyFrame()
{
    m_windowLayout->setCurrentIndex(IN_LOBBY_INDEX);
}

void CheckersPlayerWindow::moveToGameFrame()
{
    m_windowLayout->setCurrentIndex(GAME_INDEX);
}

void CheckersPlayerWindow::createAccount(const std::string &playerName, const std::string &playerPassword)
{
    if (auto playerNode = m_playerNode.lock())
    {
        playerNode->createAccount(playerName, playerPassword);
    }
}

void CheckersPlayerWindow::logInAccount(const std::string &playerName, const std::string &playerPassword)
{
    if (auto playerNode = m_playerNode.lock())
    {
        playerNode->logInAccount(playerName, playerPassword);
    }
}

void CheckersPlayerWindow::logOutAccount()
{
    if (auto playerNode = m_playerNode.lock())
    {
        playerNode->logOutAccount();
    }
    Parameters::setPlayerName(""); // Clear out the player name
    moveToTitleFrame();
}

void CheckersPlayerWindow::createLobby(const std::string &lobbyPassword)
{
    if (auto playerNode = m_playerNode.lock())
    {
        playerNode->createLobby(Parameters::getPlayerName(),
                                Parameters::getLobbyName(),
                                lobbyPassword,
                                Parameters::getPlayerColor());
    }
}

void CheckersPlayerWindow::joinLobby(const std::string &lobbyPassword)
{
    if (auto playerNode = m_playerNode.lock())
    {
        playerNode->joinLobby(Parameters::getPlayerName(),
                              Parameters::getLobbyName(),
                              Parameters::getLobbyId(),
                              lobbyPassword,
                              Parameters::getPlayerColor());
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
    m_lobbyListFrame->displayLobbyList(lobbyNames,
                                       lobbyIds,
                                       hasPasswords,
                                       blackPlayerNames,
                                       redPlayerNames);
}

void CheckersPlayerWindow::setPasswordIncorrect()
{
    m_lobbyPasswordFrame->setPasswordIncorrect();
}

void CheckersPlayerWindow::leaveLobby()
{
    if (auto playerNode = m_playerNode.lock())
    {
        playerNode->leaveLobby();
    }
    Parameters::setLobbyName(""); // Clear out the lobby name
    Parameters::setLobbyId("");   // Clear out the lobby ID
}

void CheckersPlayerWindow::updateLobbyOwner(const std::string &playerName)
{
    m_inLobbyFrame->updateLobbyOwner(playerName);
}

void CheckersPlayerWindow::kickPlayer(const std::string &playerName)
{
    if (auto playerNode = m_playerNode.lock())
    {
        playerNode->kickPlayer(playerName);
    }
}

void CheckersPlayerWindow::setReady(bool ready)
{
    if (auto playerNode = m_playerNode.lock())
    {
        playerNode->setReady(ready);
    }
}

void CheckersPlayerWindow::setTimer(uint64_t timerSeconds)
{
    if (auto playerNode = m_playerNode.lock())
    {
        playerNode->setTimer(timerSeconds);
    }
}

void CheckersPlayerWindow::loggedIn(const std::string &playerName)
{
    Parameters::setPlayerName(playerName);
    moveToMainMenuFrame();
}

void CheckersPlayerWindow::failedLogIn(const std::string &errorMessage)
{
    m_logInAccountFrame->failedLogIn(errorMessage);
}

void CheckersPlayerWindow::accountCreated(const std::string &playerName)
{
    Parameters::setPlayerName(playerName);
    moveToMainMenuFrame();
}

void CheckersPlayerWindow::failedCreate(const std::string &errorMessage)
{
    m_createAccountFrame->failedCreate(errorMessage);
}

void CheckersPlayerWindow::connectedToLobby(const std::string &lobbyName,
                                            const std::string &lobbyId,
                                            const std::string &blackPlayerName,
                                            const std::string &redPlayerName,
                                            TurtlePieceColor lobbyOwnerColor,
                                            bool blackPlayerReady,
                                            bool redPlayerReady,
                                            uint64_t timerSeconds)
{
    Parameters::setLobbyName(lobbyName);
    Parameters::setLobbyId(lobbyId);
    m_inLobbyFrame->setLobbyInfo(blackPlayerName,
                                 redPlayerName,
                                 lobbyOwnerColor,
                                 blackPlayerReady,
                                 redPlayerReady,
                                 timerSeconds);
    moveToInLobbyFrame();
}

void CheckersPlayerWindow::playerJoinedLobby(const std::string &playerName, TurtlePieceColor playerColor)
{
    m_inLobbyFrame->playerJoinedLobby(playerName, playerColor);
}

void CheckersPlayerWindow::playerLeftLobby(const std::string &playerName)
{
    m_inLobbyFrame->playerLeftLobby(playerName);
}

void CheckersPlayerWindow::setPlayerReady(const std::string &playerName, bool ready)
{
    m_inLobbyFrame->setPlayerReady(playerName, ready);
}

void CheckersPlayerWindow::updateTimer(uint64_t timerSeconds)
{
    m_inLobbyFrame->setTimer(timerSeconds);
}

void CheckersPlayerWindow::requestedPieceMoveAccepted(bool moveAccepted)
{
    m_gameFrame->requestedPieceMoveAccepted(moveAccepted);
}

void CheckersPlayerWindow::requestedReachableTiles(const std::vector<size_t> &reachableTileIndices)
{
    m_gameFrame->requestedReachableTiles(reachableTileIndices);
}

void CheckersPlayerWindow::declaredWinner(Winner winner)
{
    m_gameFrame->declaredWinner(winner);
}

void CheckersPlayerWindow::gameStarted(GameState gameState,
                                       const std::vector<size_t> &movableTileIndices,
                                       size_t blackTimeRemainSec, size_t redTimeRemainSec)
{
    m_gameFrame->connectedToGame();
    m_gameFrame->gameStarted(gameState, movableTileIndices, blackTimeRemainSec, redTimeRemainSec);

    moveToGameFrame();

    update();
}

void CheckersPlayerWindow::addChatMessage(const std::string &playerName,
                                          TurtlePieceColor playerColor,
                                          const std::string &chatMessage,
                                          std::chrono::time_point<std::chrono::system_clock> timeStamp)
{
    m_gameFrame->addChatMessage(playerName, playerColor, chatMessage, timeStamp);
}

void CheckersPlayerWindow::sendChatMessage(const std::string &chatMessage)
{
    if (auto playerNode = m_playerNode.lock())
    {
        playerNode->sendChatMessage(chatMessage);
    }
}

void CheckersPlayerWindow::updatedBoard(size_t sourceTileIndex, size_t destinationTileIndex,
                                        GameState gameState, int slainPieceTileIndex, bool kingPiece,
                                        const std::vector<size_t> &movableTileIndices,
                                        size_t blackTimeRemainSec, size_t redTimeRemainSec)
{
    m_gameFrame->updatedBoard(sourceTileIndex, destinationTileIndex, gameState,
                              slainPieceTileIndex, kingPiece, movableTileIndices,
                              blackTimeRemainSec, redTimeRemainSec);
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
    m_gameFrame->drawDeclined();
}

void CheckersPlayerWindow::drawOffered()
{
    m_gameFrame->drawOffered();
}