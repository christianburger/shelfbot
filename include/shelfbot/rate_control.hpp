#ifndef RATE_CONTROL_HPP
#define RATE_CONTROL_HPP

#include <chrono>
#include "rclcpp/rclcpp.hpp"

namespace shelfbot {

class RateControl {
public:
    explicit RateControl(double rate_hz) : period_(1.0 / rate_hz),
          last_update_time_(std::chrono::steady_clock::now()) {}

    bool shouldUpdate() {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration<double>(now - last_update_time_).count();
        
        if (elapsed >= period_) {
            last_update_time_ = now;
            return true;
        }
        return false;
    }

private:
    double period_;
    std::chrono::steady_clock::time_point last_update_time_;
};

}

#endif
