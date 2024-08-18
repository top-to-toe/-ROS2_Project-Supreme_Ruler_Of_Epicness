#include "msg/mpub.hpp"

MessagePublisher::MessagePublisher()
: Node("mpub"), _i(0)
{
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    _pub1 = this->create_publisher<std_msgs::msg::String>("message1", qos_profile);
    _pub2 = this->create_publisher<std_msgs::msg::String>("message2", qos_profile);
    _timer = this->create_wall_timer(1s, std::bind(&MessagePublisher::publish_message_msg,this));
}

void MessagePublisher::publish_message_msg() {
    auto msg1 = std_msgs::msg::String();
    msg1.data = "Message1" + to_string(_i);

    auto msg2 = std_msgs::msg::String();
    msg2.data = "Message2" + to_string(_i);
    
    _pub1->publish(msg1);
    _pub2->publish(msg2);
    _i++;
}