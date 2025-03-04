#ifndef TV2X_CANBUSNODE_PARAMETERS_HPP
#define TV2X_CANBUSNODE_PARAMETERS_HPP

#include <rclcpp/rclcpp.hpp>


template<typename T>
struct is_std_vector : std::false_type {};

template<typename T>
struct is_std_vector<std::vector<T>> : std::true_type {};


namespace tv2x_canbus {

  class Parameters {

    private:
      rclcpp::Node& m_node;
      std::string m_prefix;

      inline std::string get_param_name(const std::string& s) const {
        return m_prefix.size() > 0 ? m_prefix + "." + s : s;
      }

      template <typename T>
      inline T log_val(const std::string& s, T value) const {
        std::string repr;
        if constexpr (std::is_same_v<T, std::string>)
          repr = value;
        else if constexpr(is_std_vector<T>::value) {
          for (auto& v: value)
            repr += std::to_string(v) + "";
        }
        else
          repr = std::to_string(value);
        
        RCLCPP_INFO(m_node.get_logger(), "Parameter %s: %s", s.c_str(), repr.c_str());
        return value;
      }

    public:

      Parameters(rclcpp::Node* node, const std::string& prefix = "") : m_node(*node), m_prefix(prefix) {}

      template <typename T>
      T get(const std::string& s, std::optional<T> default_value = std::nullopt) const {
        auto name = this->get_param_name(s);
        try {

          if (default_value.has_value()) {
            auto param = m_node.declare_parameter(name, rclcpp::ParameterValue(T{}).get_type());
            T ans;

            if (m_node.get_parameter(name, ans))
              return this->log_val<T>(name, ans);
            else
              return this->log_val<T>(name, *default_value);

          } else
            return this->log_val<T>(name, m_node.declare_parameter(name, rclcpp::ParameterValue(T{}).get_type()).get<T>());

        } catch (...) {
          RCLCPP_ERROR(m_node.get_logger(), "Error on parsing parameter %s", name.c_str());
          throw;
        }
      }
  };

}

#endif
