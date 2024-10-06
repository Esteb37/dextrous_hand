#include <rclcpp/rclcpp.hpp>

class BasicNode : public rclcpp::Node
{
public:
    BasicNode() : Node("basic_node")
    {
        RCLCPP_INFO(this->get_logger(), "Hello, ROS 2 (C++)!");
    }
};