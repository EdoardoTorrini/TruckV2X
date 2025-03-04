#include <tv2x_canbus/tv2x_canbus.hpp>


namespace tv2x_canbus {

  CANbusBridge::CANbusBridge() : 
    tv2x_base::Node("tv2x_canbus_node"), m_interface(Parameters(this).get<std::string>("interface", "can0")),
    m_bitrate(Parameters(this).get<int>("bitrate", 1000000)) {
    
      RCLCPP_INFO(this->get_logger(), "starting listening");

  }

}