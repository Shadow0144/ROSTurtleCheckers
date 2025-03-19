#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class GamePlayerNode : public rclcpp::Node
{
public:
    GamePlayerNode()
        : Node("checkers_game_node")
    {

        // A game node creates a 2-player game for player nodes to publish their moves to
        // The game node publishes when it is ready for which player's next move
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "player_move", 10, std::bind(&GamePlayerNode::player_move_callback, this, _1));
    }

private:
    void player_move_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GamePlayerNode>());
    rclcpp::shutdown();
    return 0;
}