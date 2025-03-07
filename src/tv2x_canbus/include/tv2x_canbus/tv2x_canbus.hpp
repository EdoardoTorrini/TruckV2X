#ifndef TV2X_CANBUSNODE_CANBUSNODE_HPP
#define TV2X_CANBUSNODE_CANBUSNODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <tv2x_base/parameter.hpp>
#include <tv2x_base/tv2x_base.hpp>

#include <tv2x_canbus/canbus.hpp>

namespace tv2x_canbus {
  using namespace std::chrono;
  using namespace tv2x_canbus_interface;

  class CANbusBridge : public tv2x_base::Node 
  {

    private:

      // Parameters
      std::string m_interface;
      uint32_t m_can_id, m_can_mask;

      // Class attribute
      CANbus* m_canbus = nullptr;

#ifdef DEBUG_CAN
      rclcpp::TimerBase::SharedPtr m_test_can;
      void timer_callback();
#endif

    public:

      CANbusBridge();
      ~CANbusBridge() { delete this->m_canbus; }


  };

}

#endif