#ifndef MPUB_HPP
#define MPUB_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>

using namespace std;
using namespace std::chrono_literals;

class MessagePublisher : public rclcpp::Node
{
public:
    MessagePublisher();

private:
    int _i;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub1;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub2;
    rclcpp::TimerBase::SharedPtr _timer;
    void publish_message_msg();
};

#endif