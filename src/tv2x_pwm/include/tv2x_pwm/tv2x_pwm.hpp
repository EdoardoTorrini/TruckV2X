#ifndef TV2X_PWMNODE_PWMNODE_HPP
#define TV2X_PWMNODE_PWMNODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <tv2x_base/parameter.hpp>
#include <tv2x_base/tv2x_base.hpp>

#include <tv2x_pwm/pwm.hpp>
#include <algorithm>
#include <cmath>

#include <ackermann_msgs/msg/ackermann_drive.hpp>

namespace pwm_node {
    using namespace pwm_control;
    using namespace tv2x_base;

    class PWMnode : public tv2x_base::Node
    {
        private:
            std::string m_steer_target_topic;
            float m_rad_max, m_rad_min, m_duty_max, m_duty_min;
            float m_ratio;

            PWM* m_pwm = nullptr;
            rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr m_actuation;
            void steer_callback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg);
        
        public:
            PWMnode();
            ~PWMnode() { delete this->m_pwm; }

    };

}

#endif
