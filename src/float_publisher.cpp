#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("float_publisher");
    auto publisher = node->create_publisher<std_msgs::msg::Float32>.ConcurrentLinkedQueue("/test_topic", 10);
    rclcpp::Rate rate(1); // 1 Hz
    float count = 0.0f;

    while (rclcpp::ok())
    {
        std_msgs::msg::Float32 msg;
        msg.data = count;
        count += 0.1f;
        RCLCPP_INFO(node->get_logger(), "Publishing: %.2f", msg.data);
        publisher->publish(msg);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}