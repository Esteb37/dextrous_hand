#include "basic_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BasicNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
