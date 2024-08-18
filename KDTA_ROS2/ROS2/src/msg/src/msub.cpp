#include "msg/msub.hpp"

MessageSubscriber::MessageSubscriber()
: Node("msub")
{
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    _sub = this->create_subscription<std_msgs::msg::String>("message1", qos_profile,std::bind(&MessageSubscriber::sub_message_msg, this, std::placeholders::_1));
}

void MessageSubscriber::sub_message_msg(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "I heard: '%s'", msg->data.c_str());
}