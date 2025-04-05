#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("string_publisher");
    auto publisher = node->create_publisher<std_msgs::msg::String>("/test_topic", 10);
    rclcpp::Rate rate(1); // 1 Hz
    int count = 0;

    while (rclcpp::ok())
    {
        std_msgs::msg::String msg;
        msg.data = "Hello, ROS 2! " + std::to_string(count++);
        RCLCPP_INFO(node->get_logger(), "Publishing: '%s'", msg.data.c_str());
        publisher->publish(msg);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}