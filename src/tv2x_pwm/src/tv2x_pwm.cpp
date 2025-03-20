#include <tv2x_pwm/tv2x_pwm.hpp>

namespace pwm_node {

  PWMnode::PWMnode() : tv2x_base::Node("tv2x_pwm_node"), 
    m_steer_target_topic(Parameters(this, "topic").get<std::string>("steerCmd", "/steer/target")),
    m_rad_max(Parameters(this, "steer_controller").get<float>("rad_max", 0.35)),
    m_rad_min(Parameters(this, "steer_controller").get<float>("rad_min", -0.35)),
    m_duty_max(Parameters(this, "steer_controller").get<float>("duty_max", 20.0)),
    m_duty_min(Parameters(this, "steer_controller").get<float>("duty_min", 10.0))
  {

    std::string interface = Parameters(this, "generic").get<std::string>("interface", "pwmchip2");
    this->m_pwm = new PWM(
      interface,
      Parameters(this, "generic").get<int>("period", 10000),
      Parameters(this, "generic").get<int>("duty_cycle", 1500)
    );
    
    this->m_ratio = (this->m_duty_max - this->m_duty_min) / (this->m_rad_max - this->m_rad_min);
    this->m_actuation = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
      this->m_steer_target_topic, 1, std::bind(&PWMnode::steer_callback, this, std::placeholders::_1));

  }

  void PWMnode::steer_callback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg) {
    float rad_target = std::clamp<float>(msg->steering_angle, this->m_rad_min, this->m_rad_max);
    int duty = std::round(this->m_duty_min + ((rad_target - this->m_rad_min) * this->m_ratio));
    RCLCPP_INFO(this->get_logger(), "tv2x_pwm: set duty cycle to %d", duty);

#ifndef PWM_TEST
    RCLCPP_INFO(this->get_logger(), "not debug");
    this->m_pwm->set_duty_cycle(duty);
#endif

  }

}