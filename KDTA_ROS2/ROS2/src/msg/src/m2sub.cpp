#include "msg/m2sub.hpp"

Message2Subscriber::Message2Subscriber()
: Node("m2sub")
{
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    _sub = this->create_subscription<std_msgs::msg::String>("message2", qos_profile,std::bind(&Message2Subscriber::sub_message2_msg, this, std::placeholders::_1));
}

void Message2Subscriber::sub_message2_msg(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "I heard: '%s'", msg->data.c_str());
}