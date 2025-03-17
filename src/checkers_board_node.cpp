#include "checkers_board_frame.hpp"
#include "checkers_board_node.hpp"

#include <rclcpp/rclcpp.hpp>

CheckersBoardNode::CheckersBoardNode(int & argc, char ** argv)
    : QApplication(argc, argv)
{
    rclcpp::init(argc, argv);
    board_node = rclcpp::Node::make_shared("checkers_board_node");
    /*publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&CheckersBoardNode::timer_callback, this));*/
}

int CheckersBoardNode::exec()
{
    CheckersBoardFrame frame(board_node);
    frame.show();

    return QApplication::exec();
}

int main(int argc, char **argv)
{
    CheckersBoardNode node(argc, argv);
    return node.exec();
}