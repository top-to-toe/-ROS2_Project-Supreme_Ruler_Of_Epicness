#include "msg/mpub.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MessagePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}