#ifndef TV2X_CANBUSNODE_CANBUSNODE_HPP
#define TV2X_CANBUSNODE_CANBUSNODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <tv2x_base/parameter.hpp>
#include <tv2x_base/tv2x_base.hpp>

namespace tv2x_canbus {

    class CANbusBridge : public tv2x_base::Node 
    {

        private:
            
            std::string m_interface;
            int m_bitrate;

        public:

            CANbusBridge();


    };

}

#endif