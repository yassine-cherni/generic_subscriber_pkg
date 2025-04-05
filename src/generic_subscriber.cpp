#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include <memory>
#include <string>

class GenericSubscriber : public rclcpp::Node
{
public:
    GenericSubscriber()
    : Node("generic_subscriber", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    {
        // Declare parameter for topic name
        this->declare_parameter<std::string>("topic_name", "/test_topic");
        topic_name_ = this->get_parameter("topic_name").as_string();

        // Wait for topic to be available and detect its type
        bool type_found = false;
        while (rclcpp::ok() && !type_found)
        {
            auto topic_names_and_types = this->get_topic_names_and_types();
            for (const auto& topic : topic_names_and_types)
            {
                if (topic.first == topic_name_ && !topic.second.empty())
                {
                    topic_type_ = topic.second[0]; 
                    type_found = true;
                    break;
                }
            }
            if (!type_found)
            {
                RCLCPP_WARN(this->get_logger(), "Topic '%s' not found, retrying in 1 second...", topic_name_.c_str());
                rclcpp::sleep_for(std::chrono::seconds(1));
            }
        }

        // Create generic subscription
        subscription_ = this->create_generic_subscription(
            topic_name_, "serialized", 10,
            std::bind(&GenericSubscriber::callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Subscribed to topic '%s' with message type '%s'",
                    topic_name_.c_str(), topic_type_.c_str());
    }

private:
    std::string topic_name_;
    std::string topic_type_;
    rclcpp::GenericSubscription::SharedPtr subscription_;

    void callback(const std::shared_ptr<rclcpp::SerializedMessage> msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received message on topic '%s' with type '%s'",
                    topic_name_.c_str(), topic_type_.c_str());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GenericSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}