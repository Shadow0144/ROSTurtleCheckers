#pragma once

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include "Tile.hpp"

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

    std::vector<uint64_t> requestReachableTiles(const std::string &requestedPieceName);

private:
    std::string lobby_name_;
    std::string black_player_name_;
    std::string red_player_name_;

    bool nextMoveBlack_;                            // True if it is black's turn, false for red
    std::vector<uint32_t> validPiecesPerPlayer_[2]; // Black and red
    std::unordered_map<uint32_t, std::vector<uint32_t>> validMovesPerPiece_;
    std::vector<TilePtr> tiles_;
    Winner winner;
};

typedef std::shared_ptr<CheckersGameLobby> CheckersGameLobbyPtr;