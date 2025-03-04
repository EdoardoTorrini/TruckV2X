#ifndef TV2X_BASESNODE_BASESNODE_HPP
#define TV2X_BASESNODE_BASESNODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <tv2x_base/timing.hpp>

namespace tv2x_base {
  using namespace std::chrono;

  class Node : public rclcpp::Node {

    private:
      std::string m_frame_id;
      
      rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr m_heartbit;
      rclcpp::TimerBase::SharedPtr m_timer;
      
#ifndef USE_EDF
      void heartbit_callback();
#endif

      protected:
        Node(const std::string& name);

#ifdef USE_EDF
      public:
        void heartbit_callback();
#endif

  };

}
#endif