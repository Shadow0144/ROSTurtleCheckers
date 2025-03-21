#include "CheckersGameLobby.hpp"

CheckersGameLobby::CheckersGameLobby(
    const std::string &lobby_name,
    const std::string &black_player_name,
    const std::string &red_player_name)
    : lobby_name_(lobby_name),
      black_player_name_(black_player_name),
      red_player_name_(red_player_name)
{
}

std::string CheckersGameLobby::getLobbyName()
{
    return lobby_name_;
}

CheckersGameLobby::Winner CheckersGameLobby::getWinner()
{
    return winner;
}