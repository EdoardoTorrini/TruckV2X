#ifndef TV2X_CANBUSNODE_CANBUSINTERFACE_HPP
#define TV2X_CANBUSNODE_CANBUSINTERFACE_HPP

#include <mutex>
#include <atomic>
#include <string>
#include <deque>
#include <thread>
#include <functional>
#include <stdexcept>

#include <string.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>

#include <net/if.h>
#include <sys/ioctl.h>

#define NACK  1
#define ACK   0


namespace tv2x_canbus_interface {

  class CANbus 
  {
    private:

      std::string m_interface;
      struct can_filter m_filter;
      int m_socket;
      std::mutex m_mutex_socket;
      std::atomic_bool m_stop = false;

      struct ifreq m_ifr;
      struct sockaddr_can m_addr;

      std::mutex m_mutex_fifo;
      std::deque<struct can_frame> m_fifo;
      std::thread m_tread;

      void read_msg() {
        while (!this->m_stop) {
          struct can_frame frame;
          if (read(this->m_socket, &frame, sizeof(struct can_frame)) > 0) {
            if ((frame.can_id & this->m_filter.can_mask) == (this->m_filter.can_id & this->m_filter.can_mask)) {
              std::unique_lock<std::mutex> lock(this->m_mutex_fifo);
              this->m_fifo.push_back(frame);
            }
          }
        }
      }

    public:

      CANbus(std::string& interface, uint32_t can_id = 0x00, uint32_t can_mask = 0x00) : 
        m_interface(interface), m_filter({.can_id = can_id, .can_mask = can_mask}) {

        // connection to CANbus socket
        this->m_socket = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
        if (this->m_socket < 0)
          throw std::runtime_error("Error on open CANbus socket: " + std::string(strerror(errno)));

        strcpy(this->m_ifr.ifr_name, this->m_interface.c_str());
        ioctl(this->m_socket, SIOCGIFINDEX, &this->m_ifr);
        memset(&this->m_addr, 0, sizeof(this->m_addr));

        this->m_addr.can_family = AF_CAN;
        this->m_addr.can_ifindex = this->m_ifr.ifr_ifindex;

        if (bind(this->m_socket, (struct sockaddr *)&this->m_addr, sizeof(this->m_addr)) < 0)
          throw std::runtime_error("Error on bind CANbus socket " + std::string(strerror(errno)));

        // starting thread for the CANbus reading
        this->m_tread = std::thread(std::bind(&CANbus::read_msg, this));
      }

      void write_msg(struct can_frame frame) {
        if (write(this->m_socket, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
          throw std::runtime_error("Error unable to write on CANbus: " + std::string(strerror(errno)));
      }

      int get_msg(struct can_frame* frame) {
        std::unique_lock<std::mutex> lock(this->m_mutex_fifo);
        if (this->m_fifo.empty()) return NACK;
        memcpy(frame, &this->m_fifo.front(), sizeof(struct can_frame));
        this->m_fifo.pop_front();
        return ACK;
      }

      ~CANbus() {
        this->m_stop.store(true);
        this->m_tread.join();
        close(this->m_socket);
      }

  };

}

#endif