#ifndef MSUB_HPP
#define MSUB_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>

using namespace std;
using namespace std::chrono_literals;

class MessageSubscriber : public rclcpp::Node
{
public:
    MessageSubscriber();

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub;
    void sub_message_msg(const std_msgs::msg::String::SharedPtr msg);
};

#endif