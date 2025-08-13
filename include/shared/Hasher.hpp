#pragma once

#include "turtle_checkers_interfaces/msg/chat_message.hpp"
#include "turtle_checkers_interfaces/msg/client_heartbeat.hpp"
#include "turtle_checkers_interfaces/msg/declare_winner.hpp"
#include "turtle_checkers_interfaces/msg/draw_declined.hpp"
#include "turtle_checkers_interfaces/msg/draw_offered.hpp"
#include "turtle_checkers_interfaces/msg/forfit.hpp"
#include "turtle_checkers_interfaces/msg/game_start.hpp"
#include "turtle_checkers_interfaces/msg/kick_player.hpp"
#include "turtle_checkers_interfaces/msg/leave_lobby.hpp"
#include "turtle_checkers_interfaces/msg/log_out_account.hpp"
#include "turtle_checkers_interfaces/msg/offer_draw.hpp"
#include "turtle_checkers_interfaces/msg/player_banned.hpp"
#include "turtle_checkers_interfaces/msg/player_joined_lobby.hpp"
#include "turtle_checkers_interfaces/msg/player_kicked.hpp"
#include "turtle_checkers_interfaces/msg/player_left_lobby.hpp"
#include "turtle_checkers_interfaces/msg/player_logged_out.hpp"
#include "turtle_checkers_interfaces/msg/player_readied.hpp"
#include "turtle_checkers_interfaces/msg/player_ready.hpp"
#include "turtle_checkers_interfaces/msg/report_player.hpp"
#include "turtle_checkers_interfaces/msg/server_heartbeat.hpp"
#include "turtle_checkers_interfaces/msg/timer_changed.hpp"
#include "turtle_checkers_interfaces/msg/update_board.hpp"
#include "turtle_checkers_interfaces/msg/update_chat.hpp"
#include "turtle_checkers_interfaces/msg/update_lobby_owner.hpp"
#include "turtle_checkers_interfaces/msg/update_timer.hpp"

#include "turtle_checkers_interfaces/srv/change_account_password.hpp"
#include "turtle_checkers_interfaces/srv/create_account.hpp"
#include "turtle_checkers_interfaces/srv/create_lobby.hpp"
#include "turtle_checkers_interfaces/srv/get_lobby_list.hpp"
#include "turtle_checkers_interfaces/srv/join_lobby.hpp"
#include "turtle_checkers_interfaces/srv/log_in_account.hpp"
#include "turtle_checkers_interfaces/srv/request_board_state.hpp"
#include "turtle_checkers_interfaces/srv/request_piece_move.hpp"
#include "turtle_checkers_interfaces/srv/request_reachable_tiles.hpp"
#include "turtle_checkers_interfaces/srv/resync_board.hpp"

#include <memory>
#include <string>
#include <vector>

// Borrowed from Boost
inline void hashCombine(size_t &hash, size_t nextHash)
{
    hash ^= nextHash + 0x9e3779b9 + (hash << 6) + (hash >> 2);
}

template <class T>
struct std::hash<std::vector<T>>
{
    size_t operator()(const std::vector<T> &vec) const noexcept
    {
        size_t combinedHash = 0u;
        for (const auto &elem : vec)
        {
            hashCombine(combinedHash, std::hash<T>{}(elem));
        }
        return combinedHash;
    }
};

// Prevents ambiguity for special case of std::vector<bool>
template <>
struct std::hash<std::vector<bool>>
{
    size_t operator()(const std::vector<bool> &vec) const noexcept
    {
        size_t combinedHash = 0u;
        for (const auto &elem : vec)
        {
            hashCombine(combinedHash, std::hash<bool>{}(elem));
        }
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::msg::ChatMessage>
{
    size_t operator()(const turtle_checkers_interfaces::msg::ChatMessage &message) const noexcept
    {
        size_t combinedHash = 0u;
        hashCombine(combinedHash, std::hash<std::string>{}(message.lobby_name));
        hashCombine(combinedHash, std::hash<std::string>{}(message.lobby_id));
        hashCombine(combinedHash, std::hash<std::string>{}(message.player_name));
        hashCombine(combinedHash, std::hash<uint64_t>{}(message.player_color));
        hashCombine(combinedHash, std::hash<std::string>{}(message.msg));
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::msg::ClientHeartbeat>
{
    size_t operator()(const turtle_checkers_interfaces::msg::ClientHeartbeat &message) const noexcept
    {
        size_t combinedHash = 0u;
        hashCombine(combinedHash, std::hash<std::string>{}(message.player_name));
        hashCombine(combinedHash, std::hash<uint64_t>{}(message.timestamp));
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::msg::DeclareWinner>
{
    size_t operator()(const turtle_checkers_interfaces::msg::DeclareWinner &message) const noexcept
    {
        size_t combinedHash = 0u;
        hashCombine(combinedHash, std::hash<std::string>{}(message.lobby_name));
        hashCombine(combinedHash, std::hash<std::string>{}(message.lobby_id));
        hashCombine(combinedHash, std::hash<size_t>{}(message.winner));
        hashCombine(combinedHash, std::hash<size_t>{}(message.victory_condition));
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::msg::DrawDeclined>
{
    size_t operator()(const turtle_checkers_interfaces::msg::DrawDeclined &message) const noexcept
    {
        size_t combinedHash = 0u;
        hashCombine(combinedHash, std::hash<std::string>{}(message.lobby_name));
        hashCombine(combinedHash, std::hash<std::string>{}(message.lobby_id));
        hashCombine(combinedHash, std::hash<std::string>{}(message.player_name));
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::msg::DrawOffered>
{
    size_t operator()(const turtle_checkers_interfaces::msg::DrawOffered &message) const noexcept
    {
        size_t combinedHash = 0u;
        hashCombine(combinedHash, std::hash<std::string>{}(message.lobby_name));
        hashCombine(combinedHash, std::hash<std::string>{}(message.lobby_id));
        hashCombine(combinedHash, std::hash<std::string>{}(message.player_name));
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::msg::Forfit>
{
    size_t operator()(const turtle_checkers_interfaces::msg::Forfit &message) const noexcept
    {
        size_t combinedHash = 0u;
        hashCombine(combinedHash, std::hash<std::string>{}(message.lobby_name));
        hashCombine(combinedHash, std::hash<std::string>{}(message.lobby_id));
        hashCombine(combinedHash, std::hash<std::string>{}(message.player_name));
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::msg::GameStart>
{
    size_t operator()(const turtle_checkers_interfaces::msg::GameStart &message) const noexcept
    {
        size_t combinedHash = 0u;
        hashCombine(combinedHash, std::hash<std::string>{}(message.lobby_name));
        hashCombine(combinedHash, std::hash<std::string>{}(message.lobby_id));
        hashCombine(combinedHash, std::hash<std::string>{}(message.black_player_name));
        hashCombine(combinedHash, std::hash<std::string>{}(message.red_player_name));
        hashCombine(combinedHash, std::hash<uint64_t>{}(message.black_time_remaining_seconds));
        hashCombine(combinedHash, std::hash<uint64_t>{}(message.red_time_remaining_seconds));
        hashCombine(combinedHash, std::hash<uint64_t>{}(message.game_state));
        hashCombine(combinedHash, std::hash<std::vector<uint64_t>>{}(message.movable_tile_indices));
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::msg::KickPlayer>
{
    size_t operator()(const turtle_checkers_interfaces::msg::KickPlayer &message) const noexcept
    {
        size_t combinedHash = 0u;
        hashCombine(combinedHash, std::hash<std::string>{}(message.lobby_name));
        hashCombine(combinedHash, std::hash<std::string>{}(message.lobby_id));
        hashCombine(combinedHash, std::hash<std::string>{}(message.requesting_player_name));
        hashCombine(combinedHash, std::hash<std::string>{}(message.kick_player_name));
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::msg::LeaveLobby>
{
    size_t operator()(const turtle_checkers_interfaces::msg::LeaveLobby &message) const noexcept
    {
        size_t combinedHash = 0u;
        hashCombine(combinedHash, std::hash<std::string>{}(message.lobby_name));
        hashCombine(combinedHash, std::hash<std::string>{}(message.lobby_id));
        hashCombine(combinedHash, std::hash<std::string>{}(message.player_name));
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::msg::LogOutAccount>
{
    size_t operator()(const turtle_checkers_interfaces::msg::LogOutAccount &message) const noexcept
    {
        size_t combinedHash = 0u;
        hashCombine(combinedHash, std::hash<std::string>{}(message.player_name));
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::msg::OfferDraw>
{
    size_t operator()(const turtle_checkers_interfaces::msg::OfferDraw &message) const noexcept
    {
        size_t combinedHash = 0u;
        hashCombine(combinedHash, std::hash<std::string>{}(message.lobby_name));
        hashCombine(combinedHash, std::hash<std::string>{}(message.lobby_id));
        hashCombine(combinedHash, std::hash<std::string>{}(message.player_name));
        hashCombine(combinedHash, std::hash<bool>{}(message.accept_draw));
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::msg::PlayerBanned>
{
    size_t operator()(const turtle_checkers_interfaces::msg::PlayerBanned &message) const noexcept
    {
        size_t combinedHash = 0u;
        hashCombine(combinedHash, std::hash<std::string>{}(message.player_name));
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::msg::PlayerJoinedLobby>
{
    size_t operator()(const turtle_checkers_interfaces::msg::PlayerJoinedLobby &message) const noexcept
    {
        size_t combinedHash = 0u;
        hashCombine(combinedHash, std::hash<std::string>{}(message.lobby_name));
        hashCombine(combinedHash, std::hash<std::string>{}(message.lobby_id));
        hashCombine(combinedHash, std::hash<std::string>{}(message.player_name));
        hashCombine(combinedHash, std::hash<uint64_t>{}(message.player_color));
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::msg::PlayerKicked>
{
    size_t operator()(const turtle_checkers_interfaces::msg::PlayerKicked &message) const noexcept
    {
        size_t combinedHash = 0u;
        hashCombine(combinedHash, std::hash<std::string>{}(message.lobby_name));
        hashCombine(combinedHash, std::hash<std::string>{}(message.lobby_id));
        hashCombine(combinedHash, std::hash<std::string>{}(message.player_name));
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::msg::PlayerLeftLobby>
{
    size_t operator()(const turtle_checkers_interfaces::msg::PlayerLeftLobby &message) const noexcept
    {
        size_t combinedHash = 0u;
        hashCombine(combinedHash, std::hash<std::string>{}(message.lobby_name));
        hashCombine(combinedHash, std::hash<std::string>{}(message.lobby_id));
        hashCombine(combinedHash, std::hash<std::string>{}(message.player_name));
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::msg::PlayerLoggedOut>
{
    size_t operator()(const turtle_checkers_interfaces::msg::PlayerLoggedOut &message) const noexcept
    {
        size_t combinedHash = 0u;
        hashCombine(combinedHash, std::hash<std::string>{}(message.player_name));
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::msg::PlayerReadied>
{
    size_t operator()(const turtle_checkers_interfaces::msg::PlayerReadied &message) const noexcept
    {
        size_t combinedHash = 0u;
        hashCombine(combinedHash, std::hash<std::string>{}(message.lobby_name));
        hashCombine(combinedHash, std::hash<std::string>{}(message.lobby_id));
        hashCombine(combinedHash, std::hash<std::string>{}(message.player_name));
        hashCombine(combinedHash, std::hash<bool>{}(message.ready));
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::msg::PlayerReady>
{
    size_t operator()(const turtle_checkers_interfaces::msg::PlayerReady &message) const noexcept
    {
        size_t combinedHash = 0u;
        hashCombine(combinedHash, std::hash<std::string>{}(message.lobby_name));
        hashCombine(combinedHash, std::hash<std::string>{}(message.lobby_id));
        hashCombine(combinedHash, std::hash<std::string>{}(message.player_name));
        hashCombine(combinedHash, std::hash<bool>{}(message.ready));
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::msg::ReportPlayer>
{
    size_t operator()(const turtle_checkers_interfaces::msg::ReportPlayer &message) const noexcept
    {
        size_t combinedHash = 0u;
        hashCombine(combinedHash, std::hash<std::string>{}(message.reporting_player_name));
        hashCombine(combinedHash, std::hash<std::string>{}(message.reported_player_name));
        hashCombine(combinedHash, std::hash<std::string>{}(message.chat_messages));
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::msg::ServerHeartbeat>
{
    size_t operator()(const turtle_checkers_interfaces::msg::ServerHeartbeat &message) const noexcept
    {
        size_t combinedHash = 0u;
        hashCombine(combinedHash, std::hash<uint64_t>{}(message.timestamp));
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::msg::TimerChanged>
{
    size_t operator()(const turtle_checkers_interfaces::msg::TimerChanged &message) const noexcept
    {
        size_t combinedHash = 0u;
        hashCombine(combinedHash, std::hash<std::string>{}(message.lobby_name));
        hashCombine(combinedHash, std::hash<std::string>{}(message.lobby_id));
        hashCombine(combinedHash, std::hash<std::string>{}(message.player_name));
        hashCombine(combinedHash, std::hash<uint64_t>{}(message.timer_seconds));
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::msg::UpdateBoard>
{
    size_t operator()(const turtle_checkers_interfaces::msg::UpdateBoard &message) const noexcept
    {
        size_t combinedHash = 0u;
        hashCombine(combinedHash, std::hash<std::string>{}(message.lobby_name));
        hashCombine(combinedHash, std::hash<std::string>{}(message.lobby_id));
        hashCombine(combinedHash, std::hash<uint64_t>{}(message.black_time_remaining_seconds));
        hashCombine(combinedHash, std::hash<uint64_t>{}(message.red_time_remaining_seconds));
        hashCombine(combinedHash, std::hash<uint64_t>{}(message.source_tile_index));
        hashCombine(combinedHash, std::hash<uint64_t>{}(message.destination_tile_index));
        hashCombine(combinedHash, std::hash<bool>{}(message.king_piece));
        hashCombine(combinedHash, std::hash<int64_t>{}(message.slain_piece_tile_index));
        hashCombine(combinedHash, std::hash<std::vector<uint64_t>>{}(message.movable_tile_indices));
        hashCombine(combinedHash, std::hash<uint64_t>{}(message.game_state));
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::msg::UpdateChat>
{
    size_t operator()(const turtle_checkers_interfaces::msg::UpdateChat &message) const noexcept
    {
        size_t combinedHash = 0u;
        hashCombine(combinedHash, std::hash<std::string>{}(message.lobby_name));
        hashCombine(combinedHash, std::hash<std::string>{}(message.lobby_id));
        hashCombine(combinedHash, std::hash<std::string>{}(message.player_name));
        hashCombine(combinedHash, std::hash<uint64_t>{}(message.player_color));
        hashCombine(combinedHash, std::hash<std::string>{}(message.msg));
        hashCombine(combinedHash, std::hash<uint64_t>{}(message.timestamp));
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::msg::UpdateLobbyOwner>
{
    size_t operator()(const turtle_checkers_interfaces::msg::UpdateLobbyOwner &message) const noexcept
    {
        size_t combinedHash = 0u;
        hashCombine(combinedHash, std::hash<std::string>{}(message.lobby_name));
        hashCombine(combinedHash, std::hash<std::string>{}(message.lobby_id));
        hashCombine(combinedHash, std::hash<std::string>{}(message.lobby_owner_player_name));
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::msg::UpdateTimer>
{
    size_t operator()(const turtle_checkers_interfaces::msg::UpdateTimer &message) const noexcept
    {
        size_t combinedHash = 0u;
        hashCombine(combinedHash, std::hash<std::string>{}(message.lobby_name));
        hashCombine(combinedHash, std::hash<std::string>{}(message.lobby_id));
        hashCombine(combinedHash, std::hash<uint64_t>{}(message.timer_seconds));
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::srv::ChangeAccountPassword::Request::SharedPtr>
{
    size_t operator()(const turtle_checkers_interfaces::srv::ChangeAccountPassword::Request::SharedPtr &request) const noexcept
    {
        size_t combinedHash = 0u;
        hashCombine(combinedHash, std::hash<std::string>{}(request->player_name));
        hashCombine(combinedHash, std::hash<uint64_t>{}(request->encrypted_hashed_previous_player_password));
        hashCombine(combinedHash, std::hash<uint64_t>{}(request->encrypted_hashed_new_player_password));
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::srv::ChangeAccountPassword::Response::SharedPtr>
{
    size_t operator()(const turtle_checkers_interfaces::srv::ChangeAccountPassword::Response::SharedPtr &response) const noexcept
    {
        size_t combinedHash = 0u;
        hashCombine(combinedHash, std::hash<bool>{}(response->changed));
        hashCombine(combinedHash, std::hash<std::string>{}(response->player_name));
        hashCombine(combinedHash, std::hash<std::string>{}(response->error_msg));
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::srv::CreateAccount::Response::SharedPtr>
{
    size_t operator()(const turtle_checkers_interfaces::srv::CreateAccount::Response::SharedPtr &response) const noexcept
    {
        size_t combinedHash = 0u;
        hashCombine(combinedHash, std::hash<bool>{}(response->created));
        hashCombine(combinedHash, std::hash<std::string>{}(response->player_name));
        hashCombine(combinedHash, std::hash<std::string>{}(response->error_msg));
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::srv::CreateLobby::Request::SharedPtr>
{
    size_t operator()(const turtle_checkers_interfaces::srv::CreateLobby::Request::SharedPtr &request) const noexcept
    {
        size_t combinedHash = 0u;
        hashCombine(combinedHash, std::hash<std::string>{}(request->player_name));
        hashCombine(combinedHash, std::hash<std::string>{}(request->lobby_name));
        hashCombine(combinedHash, std::hash<uint64_t>{}(request->desired_player_color));
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::srv::CreateLobby::Response::SharedPtr>
{
    size_t operator()(const turtle_checkers_interfaces::srv::CreateLobby::Response::SharedPtr &response) const noexcept
    {
        size_t combinedHash = 0u;
        hashCombine(combinedHash, std::hash<std::string>{}(response->lobby_name));
        hashCombine(combinedHash, std::hash<std::string>{}(response->lobby_id));
        hashCombine(combinedHash, std::hash<bool>{}(response->created));
        hashCombine(combinedHash, std::hash<std::string>{}(response->black_player_name));
        hashCombine(combinedHash, std::hash<std::string>{}(response->red_player_name));
        hashCombine(combinedHash, std::hash<bool>{}(response->black_player_ready));
        hashCombine(combinedHash, std::hash<bool>{}(response->red_player_ready));
        hashCombine(combinedHash, std::hash<std::string>{}(response->error_msg));
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::srv::GetLobbyList::Response::SharedPtr>
{
    size_t operator()(const turtle_checkers_interfaces::srv::GetLobbyList::Response::SharedPtr &response) const noexcept
    {
        size_t combinedHash = 0u;
        hashCombine(combinedHash, std::hash<std::vector<std::string>>{}(response->lobby_names));
        hashCombine(combinedHash, std::hash<std::vector<std::string>>{}(response->lobby_ids));
        hashCombine(combinedHash, std::hash<std::vector<std::string>>{}(response->joined_black_player_names));
        hashCombine(combinedHash, std::hash<std::vector<std::string>>{}(response->joined_red_player_names));
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::srv::JoinLobby::Request::SharedPtr>
{
    size_t operator()(const turtle_checkers_interfaces::srv::JoinLobby::Request::SharedPtr &request) const noexcept
    {
        size_t combinedHash = 0u;
        hashCombine(combinedHash, std::hash<std::string>{}(request->player_name));
        hashCombine(combinedHash, std::hash<std::string>{}(request->lobby_name));
        hashCombine(combinedHash, std::hash<std::string>{}(request->lobby_id));
        hashCombine(combinedHash, std::hash<uint64_t>{}(request->desired_player_color));
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::srv::JoinLobby::Response::SharedPtr>
{
    size_t operator()(const turtle_checkers_interfaces::srv::JoinLobby::Response::SharedPtr &response) const noexcept
    {
        size_t combinedHash = 0u;
        hashCombine(combinedHash, std::hash<std::string>{}(response->lobby_name));
        hashCombine(combinedHash, std::hash<std::string>{}(response->lobby_id));
        hashCombine(combinedHash, std::hash<bool>{}(response->joined));
        hashCombine(combinedHash, std::hash<std::string>{}(response->black_player_name));
        hashCombine(combinedHash, std::hash<std::string>{}(response->red_player_name));
        hashCombine(combinedHash, std::hash<bool>{}(response->black_player_ready));
        hashCombine(combinedHash, std::hash<bool>{}(response->red_player_ready));
        hashCombine(combinedHash, std::hash<uint64_t>{}(response->timer_seconds));
        hashCombine(combinedHash, std::hash<std::string>{}(response->error_msg));
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::srv::LogInAccount::Response::SharedPtr>
{
    size_t operator()(const turtle_checkers_interfaces::srv::LogInAccount::Response::SharedPtr &response) const noexcept
    {
        size_t combinedHash = 0u;
        hashCombine(combinedHash, std::hash<bool>{}(response->logged_in));
        hashCombine(combinedHash, std::hash<std::string>{}(response->player_name));
        hashCombine(combinedHash, std::hash<std::string>{}(response->error_msg));
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::srv::RequestBoardState::Request::SharedPtr>
{
    size_t operator()(const turtle_checkers_interfaces::srv::RequestBoardState::Request::SharedPtr &request) const noexcept
    {
        size_t combinedHash = 0u;
        hashCombine(combinedHash, std::hash<std::string>{}(request->lobby_name));
        hashCombine(combinedHash, std::hash<std::string>{}(request->lobby_id));
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::srv::RequestBoardState::Response::SharedPtr>
{
    size_t operator()(const turtle_checkers_interfaces::srv::RequestBoardState::Response::SharedPtr &response) const noexcept
    {
        size_t combinedHash = 0u;
        hashCombine(combinedHash, std::hash<std::vector<uint64_t>>{}(response->turtle_color_per_tile_index));
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::srv::RequestPieceMove::Request::SharedPtr>
{
    size_t operator()(const turtle_checkers_interfaces::srv::RequestPieceMove::Request::SharedPtr &request) const noexcept
    {
        size_t combinedHash = 0u;
        hashCombine(combinedHash, std::hash<std::string>{}(request->lobby_name));
        hashCombine(combinedHash, std::hash<std::string>{}(request->lobby_id));
        hashCombine(combinedHash, std::hash<uint64_t>{}(request->source_tile_index));
        hashCombine(combinedHash, std::hash<uint64_t>{}(request->destination_tile_index));
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::srv::RequestPieceMove::Response::SharedPtr>
{
    size_t operator()(const turtle_checkers_interfaces::srv::RequestPieceMove::Response::SharedPtr &response) const noexcept
    {
        size_t combinedHash = 0u;
        hashCombine(combinedHash, std::hash<bool>{}(response->move_accepted));
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::srv::RequestReachableTiles::Request::SharedPtr>
{
    size_t operator()(const turtle_checkers_interfaces::srv::RequestReachableTiles::Request::SharedPtr &request) const noexcept
    {
        size_t combinedHash = 0u;
        hashCombine(combinedHash, std::hash<std::string>{}(request->lobby_name));
        hashCombine(combinedHash, std::hash<std::string>{}(request->lobby_id));
        hashCombine(combinedHash, std::hash<uint64_t>{}(request->tile_index));
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::srv::RequestReachableTiles::Response::SharedPtr>
{
    size_t operator()(const turtle_checkers_interfaces::srv::RequestReachableTiles::Response::SharedPtr &response) const noexcept
    {
        size_t combinedHash = 0u;
        hashCombine(combinedHash, std::hash<std::vector<uint64_t>>{}(response->reachable_tile_indices));
        return combinedHash;
    }
};

template <>
struct std::hash<turtle_checkers_interfaces::srv::ResyncBoard::Response::SharedPtr>
{
    size_t operator()(const turtle_checkers_interfaces::srv::ResyncBoard::Response::SharedPtr &response) const noexcept
    {
        size_t combinedHash = 0u;
        hashCombine(combinedHash, std::hash<std::string>{}(response->lobby_name));
        hashCombine(combinedHash, std::hash<std::string>{}(response->lobby_id));
        hashCombine(combinedHash, std::hash<uint64_t>{}(response->black_time_remaining_seconds));
        hashCombine(combinedHash, std::hash<uint64_t>{}(response->red_time_remaining_seconds));
        hashCombine(combinedHash, std::hash<uint64_t>{}(response->game_state));
        hashCombine(combinedHash, std::hash<uint64_t>{}(response->black_pieces_remaining));
        hashCombine(combinedHash, std::hash<uint64_t>{}(response->red_pieces_remaining));
        hashCombine(combinedHash, std::hash<std::vector<std::string>>{}(response->turtle_piece_name_per_tile));
        hashCombine(combinedHash, std::hash<std::vector<uint64_t>>{}(response->turtle_piece_color_per_tile));
        hashCombine(combinedHash, std::hash<std::vector<bool>>{}(response->turtle_piece_is_kinged_per_tile));
        return combinedHash;
    }
};