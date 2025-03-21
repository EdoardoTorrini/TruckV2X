#ifndef TV2X_PWMNODE_PWMCONTROL_HPP
#define TV2X_PWMNODE_PWMCONTROL_HPP

#include <iostream>
#include <stdexcept>
#include <fstream>
#include <string>
#include <exception>

namespace pwm_control {

  class PWM
  {

    private:
      int m_period, m_duty_cycle;

      std::string m_path = "/sys/class/pwm/";
      std::string m_export_path = "/sys/class/pwm/";

      bool write_on_file(const std::string& path, const std::string& value) const {
#ifndef PWM_TEST
        std::ofstream file(path);
        if (!file.is_open()) {
          std::cerr << "Failed to open file: " << path << std::endl;
          return false;
        }
        file << value;
        return file.good();
#else
        return true;
#endif
      }

      void export_pwm() { this->write_on_file(this->m_export_path, "0"); }
      void enable_pwm(int enable) {
        if ((enable != 0) && (enable != 1))
          throw std::runtime_error("Wrong value for enabling pwm");
        this->write_on_file(this->m_path + "enable", std::to_string(enable)); 
      }
      void change_period() { this->write_on_file(this->m_path + "period", std::to_string(this->m_period)); }
      void change_duty_cycle() { this->write_on_file(this->m_path + "duty_cycle", std::to_string(this->m_duty_cycle)); }

    public:
      PWM(std::string& interface, int period, int duty_cycle) : 
        m_period(period), m_duty_cycle(duty_cycle) 
      {

        if ((period <= 0) || ((duty_cycle <= 0) || (duty_cycle >= period)))
          throw std::runtime_error("Wrong value for pwm");
        
        this->m_path += interface + "/pwm0/";
        this->m_export_path += interface + "/export";

#ifndef PWM_TEST
        // configure pwm
        this->export_pwm();

        // TODO: is used to avoid error, check if there is a better way
        sleep(1);
        
        this->change_period();
        this->change_duty_cycle();
        // enable pwm
        this->enable_pwm(1);
#endif
      }

    void set_duty_cycle(int duty_cycle) {
      this->m_duty_cycle = duty_cycle;
      this->change_duty_cycle();
    }

    ~PWM() { this->enable_pwm(0); }

  };

}

#endif