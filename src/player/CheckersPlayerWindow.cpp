#include "player/CheckersPlayerWindow.hpp"

#include <QMainWindow>

#include <chrono>
#include <iostream>

#include "shared/CheckersConsts.hpp"
#include "player/Parameters.hpp"
#include "player/CheckersPlayerNode.hpp"
#include "player/frame/ChangeAccountPasswordFrame.hpp"
#include "player/frame/CreateAccountFrame.hpp"
#include "player/frame/CreateLobbyFrame.hpp"
#include "player/frame/GameFrame.hpp"
#include "player/frame/InLobbyFrame.hpp"
#include "player/frame/LobbyListFrame.hpp"
#include "player/frame/LobbyPasswordFrame.hpp"
#include "player/frame/LogInAccountFrame.hpp"
#include "player/frame/MainMenuFrame.hpp"
#include "player/frame/StatisticsFrame.hpp"
#include "player/frame/TitleFrame.hpp"
#include "player/DialogWidget.hpp"

#include <QWidget>
#include <QStackedLayout>
#include <QString>

static constexpr int CHANGE_ACCOUNT_PASSWORD_INDEX = 0;
static constexpr int CREATE_ACCOUNT_INDEX = 1;
static constexpr int CREATE_LOBBY_INDEX = 2;
static constexpr int GAME_INDEX = 3;
static constexpr int IN_LOBBY_INDEX = 4;
static constexpr int LOBBY_LIST_INDEX = 5;
static constexpr int LOBBY_PASSWORD_INDEX = 6;
static constexpr int LOG_IN_ACCOUNT_INDEX = 7;
static constexpr int MAIN_MENU_INDEX = 8;
static constexpr int STATISTICS_INDEX = 9;
static constexpr int TITLE_INDEX = 10;

CheckersPlayerWindow::CheckersPlayerWindow(const CheckersPlayerNodeWkPtr &playerNode)
    : QMainWindow()
{
    m_playerNode = playerNode;

    m_windowState = WindowState::MainMenu;

    setFixedSize(WINDOW_WIDTH, WINDOW_HEIGHT);
    setWindowTitle(QString::fromStdString("Turtle Checkers"));

    setMouseTracking(true);

    Parameters::setLanguageChangedCallback([this]()
                                           { this->reloadStrings(); });

    m_windowLayoutWidget = new QWidget(this);
    m_windowLayout = new QStackedLayout(m_windowLayoutWidget);

    // Create and add the screens to the layout
    m_changeAccountPasswordFrame = new ChangeAccountPasswordFrame(this);
    m_createAccountFrame = new CreateAccountFrame(this);
    m_createLobbyFrame = new CreateLobbyFrame(this);
    m_gameFrame = new GameFrame(this);
    m_inLobbyFrame = new InLobbyFrame(this);
    m_lobbyListFrame = new LobbyListFrame(this);
    m_lobbyPasswordFrame = new LobbyPasswordFrame(this);
    m_logInAccountFrame = new LogInAccountFrame(this);
    m_mainMenuFrame = new MainMenuFrame(this);
    m_statisticsFrame = new StatisticsFrame(this);
    m_titleFrame = new TitleFrame(this);

    m_windowLayout->insertWidget(CHANGE_ACCOUNT_PASSWORD_INDEX, m_changeAccountPasswordFrame);
    m_windowLayout->insertWidget(CREATE_ACCOUNT_INDEX, m_createAccountFrame);
    m_windowLayout->insertWidget(CREATE_LOBBY_INDEX, m_createLobbyFrame);
    m_windowLayout->insertWidget(GAME_INDEX, m_gameFrame);
    m_windowLayout->insertWidget(IN_LOBBY_INDEX, m_inLobbyFrame);
    m_windowLayout->insertWidget(LOBBY_LIST_INDEX, m_lobbyListFrame);
    m_windowLayout->insertWidget(LOBBY_PASSWORD_INDEX, m_lobbyPasswordFrame);
    m_windowLayout->insertWidget(LOG_IN_ACCOUNT_INDEX, m_logInAccountFrame);
    m_windowLayout->insertWidget(MAIN_MENU_INDEX, m_mainMenuFrame);
    m_windowLayout->insertWidget(STATISTICS_INDEX, m_statisticsFrame);
    m_windowLayout->insertWidget(TITLE_INDEX, m_titleFrame);

    m_windowLayout->setCurrentIndex(TITLE_INDEX);

    setCentralWidget(m_windowLayoutWidget);

    // Disconnected dialog
    m_disconnectedDialog = new DialogWidget(this, WINDOW_CENTER_X, WINDOW_CENTER_Y,
                                            "Lost connection to server",
                                            "Return to title", {[this]()
                                                                { this->handleReturnToTitle(); }},
                                            "", {[]() {}});

    // Logged out dialog
    m_loggedOutDialog = new DialogWidget(this, WINDOW_CENTER_X, WINDOW_CENTER_Y,
                                         "You have been logged out by the server",
                                         "Return to title", {[this]()
                                                             { this->handleReturnToTitle(); }},
                                         "", {[]() {}});

    // Kicked dialog
    m_kickedDialog = new DialogWidget(this, WINDOW_CENTER_X, WINDOW_CENTER_Y,
                                      "You have been kicked",
                                      "Return to lobby list", {[this]()
                                                               { this->handleReturnToLobbyList(); }},
                                      "", {[]() {}});

    // Banned dialog
    m_bannedDialog = new DialogWidget(this, WINDOW_CENTER_X, WINDOW_CENTER_Y,
                                      "You have been banned",
                                      "Return to title", {[this]()
                                                          { this->handleReturnToTitle(); }},
                                      "", {[]() {}});
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

void CheckersPlayerWindow::moveToChangeAccountPasswordFrame()
{
    m_windowLayout->setCurrentIndex(CHANGE_ACCOUNT_PASSWORD_INDEX);
}

void CheckersPlayerWindow::moveToStatisticsFrame()
{
    m_windowLayout->setCurrentIndex(STATISTICS_INDEX);
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
    Parameters::setPlayerName("");   // Clear out the player name
    Parameters::setOpponentName(""); // Clear out the opponent name too
    moveToTitleFrame();
}

void CheckersPlayerWindow::requestStatistics(const std::string &playerName)
{
    if (auto playerNode = m_playerNode.lock())
    {
        playerNode->requestStatistics(playerName);
    }
}

void CheckersPlayerWindow::displayStatistics(const std::string &playerName,
                                             const std::vector<std::string> &lobbyNameIds,
                                             const std::vector<std::string> &blackPlayerNames,
                                             const std::vector<std::string> &redPlayerNames,
                                             const std::vector<uint64_t> &winners,
                                             uint64_t matchesPlayed,
                                             uint64_t matchesWon,
                                             uint64_t matchesLost,
                                             uint64_t matchesDrawn)
{
    m_statisticsFrame->displayStatistics(playerName, lobbyNameIds, blackPlayerNames, redPlayerNames,
                                         winners, matchesPlayed, matchesWon, matchesLost, matchesDrawn);
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
                              lobbyPassword);
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

void CheckersPlayerWindow::changeAccountPassword(const std::string &previousPlayerPassword,
                                                 const std::string &newPlayerPassword)
{
    if (auto playerNode = m_playerNode.lock())
    {
        playerNode->changeAccountPassword(previousPlayerPassword, newPlayerPassword);
    }
}

void CheckersPlayerWindow::accountPasswordChanged()
{
    m_changeAccountPasswordFrame->succeededChange();
}

void CheckersPlayerWindow::failedAccountPasswordChange(const std::string &errorMessage)
{
    m_changeAccountPasswordFrame->failedChange(errorMessage);
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
    m_gameFrame->clearChat();
    m_inLobbyFrame->clearChat();
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

void CheckersPlayerWindow::reportPlayer(const std::string &chatMessages)
{
    if (auto playerNode = m_playerNode.lock())
    {
        playerNode->reportPlayer(chatMessages);
    }
}

void CheckersPlayerWindow::addChatMessage(const std::string &playerName,
                                          TurtlePieceColor playerColor,
                                          const std::string &chatMessage,
                                          std::chrono::time_point<std::chrono::system_clock> timeStamp)
{
    m_inLobbyFrame->addChatMessage(playerName, playerColor, chatMessage, timeStamp);
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

void CheckersPlayerWindow::resyncBoard(uint64_t blackTimeRemainingSeconds,
                                       uint64_t redTimeRemainingSeconds,
                                       uint64_t gameState,
                                       uint64_t blackPiecesRemaining,
                                       uint64_t redPiecesRemaining,
                                       std::vector<std::string> turtlePieceNamePerTile,
                                       std::vector<uint64_t> turtlePieceColorPerTile,
                                       std::vector<bool> turtlePieceIsKingedPerTile)
{
    m_gameFrame->resyncBoard(blackTimeRemainingSeconds,
                             redTimeRemainingSeconds,
                             gameState,
                             blackPiecesRemaining,
                             redPiecesRemaining,
                             turtlePieceNamePerTile,
                             turtlePieceColorPerTile,
                             turtlePieceIsKingedPerTile);
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

void CheckersPlayerWindow::forfeit()
{
    if (auto playerNode = m_playerNode.lock())
    {
        playerNode->forfeit();
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

void CheckersPlayerWindow::disconnected()
{
    setConnectedToServer(false);
    displayDialog(true, m_disconnectedDialog);
}

void CheckersPlayerWindow::loggedOut()
{
    displayDialog(true, m_loggedOutDialog);
}

void CheckersPlayerWindow::kicked()
{
    displayDialog(true, m_kickedDialog);
}

void CheckersPlayerWindow::banned()
{
    displayDialog(true, m_bannedDialog);
}

void CheckersPlayerWindow::displayDialog(bool dialogDisplayed, DialogWidget *dialog)
{
    // Enable or disable everything as needed
    m_showingDialog = dialogDisplayed;
    m_windowLayoutWidget->setEnabled(!dialogDisplayed);
    m_disconnectedDialog->hide();
    m_loggedOutDialog->hide();
    m_kickedDialog->hide();
    m_bannedDialog->hide();
    if (dialogDisplayed && dialog)
    {
        dialog->show();
    }
}

void CheckersPlayerWindow::handleReturnToTitle()
{
    displayDialog(false, nullptr);
    moveToTitleFrame();
}

void CheckersPlayerWindow::handleReturnToLobbyList()
{
    displayDialog(false, nullptr);
    moveToLobbyListFrame();
}

uint64_t CheckersPlayerWindow::getBoardHash() const
{
    return m_gameFrame->getBoardHash();
}

void CheckersPlayerWindow::reloadStrings()
{
    m_changeAccountPasswordFrame->reloadStrings();
    m_createAccountFrame->reloadStrings();
    m_createLobbyFrame->reloadStrings();
    m_gameFrame->reloadStrings();
    m_inLobbyFrame->reloadStrings();
    m_lobbyListFrame->reloadStrings();
    m_lobbyPasswordFrame->reloadStrings();
    m_logInAccountFrame->reloadStrings();
    m_mainMenuFrame->reloadStrings();
    m_statisticsFrame->reloadStrings();
    m_titleFrame->reloadStrings();
}