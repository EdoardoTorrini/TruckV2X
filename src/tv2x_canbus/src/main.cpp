#include <tv2x_canbus/tv2x_canbus.hpp>

void handle_signal(int signal) {
    if (signal == SIGINT)
        std::cout << "Reciving SIGINT. Requesting stop." << std::endl;
}

int main(int argc, char* argv[])
{
    signal(SIGINT, handle_signal);
    rclcpp::init(argc, argv);

    auto node = std::make_shared<tv2x_canbus::CANbusBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}