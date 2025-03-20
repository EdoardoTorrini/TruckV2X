#include <tv2x_pwm/tv2x_pwm.hpp>

void handle_signal(int signal) {
    if (signal == SIGINT)
        std::cout << "Reciving SIGINT. Requesting stop." << std::endl;
}

// TODO: add the definition for the EDF_SCHEDULER
int main(int argc, char* argv[])
{
    signal(SIGINT, handle_signal);
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<pwm_node::PWMnode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}