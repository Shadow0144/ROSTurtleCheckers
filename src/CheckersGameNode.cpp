#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "turtle_checkers_interfaces/srv/request_reachable_tiles.hpp"

#include "CheckersGameLobby.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class GamePlayerNode : public rclcpp::Node
{
public:
    GamePlayerNode()
        : Node("checkers_game_node")
    {
        // A game node creates a 2-player game for player nodes to publish their moves to
        // The game node publishes when it is ready for which player's next move
        /*subscription_ = this->create_subscription<std_msgs::msg::String>(
            "player_move", 10, std::bind(&GamePlayerNode::player_move_callback, this, _1));*/
        /*publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&CheckersPlayerNode::timer_callback, this));*/

        std::string lobbyName = "";
        std::string blackPlayerName = "";
        std::string redPlayerName = "";
        checkersGameLobby = std::make_shared<CheckersGameLobby>(lobbyName, blackPlayerName, redPlayerName);

        requestReachableTilesService =
            this->create_service<turtle_checkers_interfaces::srv::RequestReachableTiles>("RequestReachableTiles", std::bind(&GamePlayerNode::requestReachableTilesRequest, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    /*void player_move_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }*/

    void requestReachableTilesRequest(const std::shared_ptr<turtle_checkers_interfaces::srv::RequestReachableTiles::Request> request,
                                      std::shared_ptr<turtle_checkers_interfaces::srv::RequestReachableTiles::Response> response)
    {
        response->reachable_tile_indices = checkersGameLobby->requestReachableTiles(request->piece_name);
    }

    //rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    rclcpp::Service<turtle_checkers_interfaces::srv::RequestReachableTiles>::SharedPtr requestReachableTilesService;

    CheckersGameLobbyPtr checkersGameLobby;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GamePlayerNode>());
    rclcpp::shutdown();
    return 0;
}