#include "msg/mtsub.hpp"

MessageTimeSubscriber::MessageTimeSubscriber()
: Node("mtsub")
{
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    _sub1 = this->create_subscription<std_msgs::msg::String>("message1", qos_profile,std::bind(&MessageTimeSubscriber::sub_message_msg, this, std::placeholders::_1));
    _sub2 = this->create_subscription<std_msgs::msg::Header>("time", qos_profile,std::bind(&MessageTimeSubscriber::sub_time_msg, this, std::placeholders::_1));
}

void MessageTimeSubscriber::sub_message_msg(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "I heard: '%s'", msg->data.c_str());
}

void MessageTimeSubscriber::sub_time_msg(const std_msgs::msg::Header::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "frame id: %s", msg->frame_id.c_str());
    RCLCPP_INFO(get_logger(), "stamp: %d.%d", msg->stamp.sec, msg->stamp.nanosec);
}