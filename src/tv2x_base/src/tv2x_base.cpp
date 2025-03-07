#include <tv2x_base/tv2x_base.hpp>

namespace tv2x_base {

#ifdef USE_EDF
  Node::Node(const std::string& name, uint64_t period_ns, uint64_t runtime_ns, uint64_t deadline_ns) : rclcpp::Node(name), 
    m_frame_id(name) {
    auto topic = name + "/heartbit";
    this->m_heartbit = this->create_publisher<std_msgs::msg::Header>(topic, 1);
    this->set_edf_scheduler(period_ns, runtime_ns, deadline_ns);
    this->m_timer = this->create_wall_timer(100ms, std::bind(&Node::heartbit_callback, this));
  }
#else
  Node::Node(const std::string& name) : rclcpp::Node(name), m_frame_id(name) {
    auto topic = name + "/heartbit";
    this->m_heartbit = this->create_publisher<std_msgs::msg::Header>(topic, 1);
    this->m_timer = this->create_wall_timer(100ms, std::bind(&Node::heartbit_callback, this));
  }
#endif


  void Node::set_edf_scheduler(uint64_t period_ns, uint64_t runtime_ns, uint64_t deadline_ns) {
    sched_attr attr = {
      .size = sizeof(attr),
      .sched_policy = SCHED_DEADLINE,
      .sched_flags = SCHED_FLAG_RESET_ON_FORK,
      .sched_nice = SCHED_OTHER,
      .sched_priority = 0,
      .sched_runtime = runtime_ns,
      .sched_deadline = deadline_ns,
      .sched_period = period_ns,
    };

    if (syscall(SYS_sched_setattr, gettid(), &attr, 0) != 0)
      throw std::runtime_error("Wrong parameters for EDF scheduler: " + std::string(strerror(errno)));
  }

  void Node::set_affinity(int8_t cpu) {
    if (cpu < 0) return;

    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(cpu, &set);

    if (sched_setaffinity(getpid(), sizeof(set), &set) == -1)
      throw std::runtime_error("wrong usage of sched_setaffinity" + std::string(strerror(errno)));
  }

  void Node::heartbit_callback() {
    std_msgs::msg::Header::SharedPtr msg(new std_msgs::msg::Header());
    msg->frame_id = this->m_frame_id;
    msg->stamp.sec = timing::Clock::get_time<std::chrono::seconds>().count();
    msg->stamp.nanosec = timing::Clock::get_time<std::chrono::nanoseconds>().count() % timing::NANOSECONDS_MOD;
    this->m_heartbit->publish(*msg);
  }
}