#include "player/CheckersPlayerWindow.hpp"

#include <QMainWindow>

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
    m_loginAccountFrame = new LoginAccountFrame(this);
    m_mainMenuFrame = new MainMenuFrame(this);
    m_titleFrame = new TitleFrame(this);

    m_windowLayout->insertWidget(CREATE_ACCOUNT_INDEX, m_createAccountFrame);
    m_windowLayout->insertWidget(CREATE_LOBBY_INDEX, m_createLobbyFrame);
    m_windowLayout->insertWidget(GAME_INDEX, m_gameFrame);
    m_windowLayout->insertWidget(IN_LOBBY_INDEX, m_inLobbyFrame);
    m_windowLayout->insertWidget(LOBBY_LIST_INDEX, m_lobbyListFrame);
    m_windowLayout->insertWidget(LOBBY_PASSWORD_INDEX, m_lobbyPasswordFrame);
    m_windowLayout->insertWidget(LOGIN_ACCOUNT_INDEX, m_loginAccountFrame);
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

void CheckersPlayerWindow::moveToLoginAccountFrame()
{
    m_windowLayout->setCurrentIndex(LOGIN_ACCOUNT_INDEX);
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

void CheckersPlayerWindow::loginAccount(const std::string &playerName, const std::string &playerPassword)
{
    if (auto playerNode = m_playerNode.lock())
    {
        playerNode->loginAccount(playerName, playerPassword);
    }
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
}

void CheckersPlayerWindow::setReady(bool ready)
{
    if (auto playerNode = m_playerNode.lock())
    {
        playerNode->setReady(ready);
    }
}

void CheckersPlayerWindow::loggedIn(const std::string &playerName)
{
    Parameters::setPlayerName(playerName);
    moveToMainMenuFrame();
}

void CheckersPlayerWindow::failedLogin(const std::string &errorMessage)
{
    m_createAccountFrame->failedLogin(errorMessage);
    m_loginAccountFrame->failedLogin(errorMessage);
}

void CheckersPlayerWindow::connectedToLobby(const std::string &lobbyName,
                                            const std::string &lobbyId,
                                            const std::string &blackPlayerName,
                                            const std::string &redPlayerName,
                                            bool blackPlayerReady,
                                            bool redPlayerReady)
{
    Parameters::setLobbyName(lobbyName);
    Parameters::setLobbyId(lobbyId);
    m_inLobbyFrame->setLobbyInfo(blackPlayerName,
                                 redPlayerName,
                                 blackPlayerReady,
                                 redPlayerReady);
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
                                       const std::vector<size_t> &movableTileIndices)
{
    m_gameFrame->connectedToGame();
    m_gameFrame->gameStarted(gameState, movableTileIndices);

    moveToGameFrame();

    update();
}

void CheckersPlayerWindow::updatedBoard(size_t sourceTileIndex, size_t destinationTileIndex,
                                        GameState gameState, int slainPieceTileIndex, bool kingPiece,
                                        const std::vector<size_t> &movableTileIndices)
{
    m_gameFrame->updatedBoard(sourceTileIndex, destinationTileIndex, gameState,
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