#ifndef TV2X_BASESNODE_BASESNODE_HPP
#define TV2X_BASESNODE_BASESNODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <tv2x_base/timing.hpp>

#include <sys/syscall.h>
#include <linux/sched.h>
#include <sched.h>

namespace tv2x_base {
  using namespace std::chrono;

  #define gettid() syscall(SYS_gettid)

  struct sched_attr {
    uint32_t size;
    uint32_t sched_policy;
    uint64_t sched_flags;
    int32_t sched_nice;
    uint32_t sched_priority;
    uint64_t sched_runtime;
    uint64_t sched_deadline;
    uint64_t sched_period;
  };

  class Node : public rclcpp::Node {

    private:
      std::string m_frame_id;
      
      rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr m_heartbit;
      rclcpp::TimerBase::SharedPtr m_timer;
      
#ifndef USE_EDF
      void heartbit_callback();
#endif

      protected:

#ifdef USE_EDF
        Node(const std::string& name, uint64_t period_ns, uint64_t runtime_ns, uint64_t deadline_ns);
#else
        Node(const std::string& name);
#endif
        void set_edf_scheduler(uint64_t period_ns, uint64_t runtime_ns, uint64_t deadline_ns);
        void set_affinity(int8_t cpu);

#ifdef USE_EDF
      public:
        void heartbit_callback();
#endif

  };

}
#endif