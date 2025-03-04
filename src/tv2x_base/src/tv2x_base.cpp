#include <tv2x_base/tv2x_base.hpp>

namespace tv2x_base {

  Node::Node(const std::string& name) : rclcpp::Node(name), m_frame_id(name) {
    auto topic = name + "/heartbit";
    this->m_heartbit = this->create_publisher<std_msgs::msg::Header>(topic, 1);
#ifndef USE_EDF
    this->m_timer = this->create_wall_timer(100ms, std::bind(&Node::heartbit_callback, this));
#endif
  }

  void Node::heartbit_callback() {
    std_msgs::msg::Header::SharedPtr msg(new std_msgs::msg::Header());
    msg->frame_id = this->m_frame_id;
    msg->stamp.sec = timing::Clock::get_time<std::chrono::seconds>().count();
    msg->stamp.nanosec = timing::Clock::get_time<std::chrono::nanoseconds>().count() % timing::NANOSECONDS_MOD;
    this->m_heartbit->publish(*msg);
  }
}