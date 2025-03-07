#include <tv2x_canbus/tv2x_canbus.hpp>


namespace tv2x_canbus {

  CANbusBridge::CANbusBridge() : 
    tv2x_base::Node("tv2x_canbus_node"), m_interface(Parameters(this).get<std::string>("interface", "can0")),
    m_can_id(Parameters(this, "can_filter").get<int>("can_id", 0u)),
    m_can_mask(Parameters(this, "can_filter").get<int>("can_mask", 0u)) {
    
      RCLCPP_INFO(this->get_logger(), "starting listening");
      this->m_canbus = new CANbus(this->m_interface);

#ifdef DEBUG_CAN
      this->m_test_can = this->create_wall_timer(1s, std::bind(&CANbusBridge::timer_callback, this));
#endif

  }

#ifdef DEBUG_CAN
  void CANbusBridge::timer_callback() {
    struct can_frame frame;
    RCLCPP_INFO(this->get_logger(), "enter into timer callback");
    while (!this->m_canbus->get_msg(&frame))
      RCLCPP_INFO(this->get_logger(), "[ new MSG ] id: %03x", frame.can_id);

    struct can_frame tx_frame;
    unsigned char payload[4] = {0xDE, 0xAD, 0xBE, 0xEF};
    tx_frame.can_id = 0x120;
    tx_frame.can_dlc = 4;
    memcpy(tx_frame.data, payload, sizeof(unsigned char) * 4);
    
    try {
      this->m_canbus->write_msg(tx_frame);
    } catch (...) { RCLCPP_ERROR(this->get_logger(), "tv2x_canbus_node: %s", strerror(errno)); }
    RCLCPP_INFO(this->get_logger(), "exit from timer");
    }
#endif

}