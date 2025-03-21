#pragma once

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

class CheckersGameLobby
{
public:
    enum class Winner
    {
        None,
        Black,
        Red,
        Draw
    };

    CheckersGameLobby(
        const std::string &lobby_name,
        const std::string &black_player_name,
        const std::string &red_player_name);

    std::string getLobbyName();

    Winner getWinner();

private:
    std::string lobby_name_;
    std::string black_player_name_;
    std::string red_player_name_;

    bool nextMoveBlack_;
    std::vector<uint32_t> validPiecesPerPlayer_[2]; // Black and red
    std::unordered_map<uint32_t, std::vector<uint32_t>> validMovesPerPiece_;
    Winner winner;
};

typedef std::shared_ptr<CheckersGameLobby> CheckersGameLobbyPtr;