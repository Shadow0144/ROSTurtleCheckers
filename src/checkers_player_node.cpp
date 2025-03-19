#include "checkers_player_node.hpp"
#include "checkers_board_frame.hpp"

#include <rclcpp/rclcpp.hpp>

CheckersPlayerNode::CheckersPlayerNode(int & argc, char ** argv)
    : QApplication(argc, argv)
{
    rclcpp::init(argc, argv);
    board_node = rclcpp::Node::make_shared("checkers_board_node");
    /*publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&CheckersPlayerNode::timer_callback, this));*/
}

int CheckersPlayerNode::exec()
{
    CheckersBoardFrame frame(board_node);
    frame.show();

    return QApplication::exec();
}

int main(int argc, char **argv)
{
    CheckersPlayerNode node(argc, argv);
    return node.exec();
}