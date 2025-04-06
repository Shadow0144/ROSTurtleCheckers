#pragma once

#include "turtle_checkers_interfaces/srv/request_piece_move.hpp"
#include "turtle_checkers_interfaces/srv/request_reachable_tiles.hpp"
#include "turtle_checkers_interfaces/msg/declare_winner.hpp"
#include "turtle_checkers_interfaces/msg/game_start.hpp"
#include "turtle_checkers_interfaces/msg/player_ready.hpp"
#include "turtle_checkers_interfaces/msg/update_board.hpp"

#include <string>
#include <vector>

// Borrowed from Boost
size_t hashCombine(size_t lhs, size_t rhs)
{
    lhs ^= rhs + 0x9e3779b9 + (lhs << 6) + (lhs >> 2);
    return lhs;
}

template <>
struct std::hash<turtle_checkers_interfaces::msg::DeclareWinner>
{
    size_t operator()(const turtle_checkers_interfaces::msg::DeclareWinner &message) const noexcept
    {
        size_t hash1 = std::hash<std::string>{}(message.lobby_name);
        size_t hash2 = std::hash<size_t>{}(request.winner);
        size_t combinedHash = hashCombine(hash1, hash2);
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::msg::GameStart>
{
    size_t operator()(const turtle_checkers_interfaces::msg::GameStart &message) const noexcept
    {
        size_t hash1 = std::hash<std::string>{}(message.lobby_name);
        size_t hash2 = std::hash<size_t>{}(request.game_state);
        size_t hash3 = std::hash<std::vector<size_t>>{}(response.movable_tile_indices);
        size_t combinedHash = hashCombine(hashCombine(hash1, hash2), hash3);
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::msg::PlayerReady>
{
    size_t operator()(const turtle_checkers_interfaces::msg::PlayerReady &message) const noexcept
    {
        size_t hash1 = std::hash<std::string>{}(message.lobby_name);
        size_t hash2 = std::hash<std::string>{}(message.player_name);
        size_t combinedHash = hashCombine(hash1, hash2);
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::msg::UpdateBoard>
{
    size_t operator()(const turtle_checkers_interfaces::msg::UpdateBoard &message) const noexcept
    {
        size_t hash1 = std::hash<std::string>{}(message.lobby_name);
        size_t hash2 = std::hash<size_t>{}(request.source_tile_index);
        size_t hash3 = std::hash<size_t>{}(request.destination_tile_index);
        size_t hash4 = std::hash<bool>{}(request.king_piece);
        size_t hash5 = std::hash<int64_t>{}(request.slain_piece_tile_index);
        size_t hash6 = std::hash<std::vector<size_t>>{}(response.movable_tile_indices);
        size_t hash7 = std::hash<size_t>{}(request.game_state);
        size_t combinedHash = hashCombine(hashCombine(hashCombine(hashCombine(hashCombine(hashCombine(
            hash1, hash2), hash3), hash4), hash5), hash6), hash7);
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::srv::RequestPieceMove::Request>
{
    size_t operator()(const turtle_checkers_interfaces::srv::RequestPieceMove::Request &request) const noexcept
    {
        size_t hash1 = std::hash<std::string>{}(request.lobby_name);
        size_t hash2 = std::hash<size_t>{}(request.source_tile_index);
        size_t hash3 = std::hash<size_t>{}(request.destination_tile_index);
        size_t combinedHash = hashCombine(hashCombine(hash1, hash2), hash3);
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::srv::RequestPieceMove::Response>
{
    size_t operator()(const turtle_checkers_interfaces::srv::RequestPieceMove::Response &response) const noexcept
    {
        size_t hash = std::hash<bool>{}(response.move_accepted);
        return hash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::srv::RequestReachableTiles::Request>
{
    size_t operator()(const turtle_checkers_interfaces::srv::RequestReachableTiles::Request &request) const noexcept
    {
        size_t hash1 = std::hash<std::string>{}(request.lobby_name);
        size_t hash2 = std::hash<size_t>{}(request.tile_index);
        size_t combinedHash = hashCombine(hash1, hash2);
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::srv::RequestReachableTiles::Request>
{
    size_t operator()(const turtle_checkers_interfaces::srv::RequestReachableTiles::Response &response) const noexcept
    {
        size_t hash = std::hash<std::vector<size_t>>{}(response.reachable_tile_indices);
        return hash;
    }
};