#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// types.hpp  —  POD types shared between the exploration nodes.
//
// Nothing here may include rclcpp or any ROS2 header so the types remain
// usable in pure unit-test contexts without a ROS2 install.
// ─────────────────────────────────────────────────────────────────────────────

#include <chrono>
#include <cstdint>
#include <string>

namespace shelfbot::exploration {

// ─────────────────────────────────────────────────────────────────────────────
enum class FrontierStatus : uint8_t {
    PENDING   = 0,  // waiting to be navigated to
    ACTIVE    = 1,  // currently being navigated to (marked by get_next)
    SUCCEEDED = 2,  // robot reached the frontier
    BLOCKED   = 3,  // nav2 could not reach it; eligible for requeue
};

inline const char* to_cstr(FrontierStatus s) {
    switch (s) {
        case FrontierStatus::PENDING:   return "PENDING";
        case FrontierStatus::ACTIVE:    return "ACTIVE";
        case FrontierStatus::SUCCEEDED: return "SUCCEEDED";
        case FrontierStatus::BLOCKED:   return "BLOCKED";
    }
    return "UNKNOWN";
}

inline bool from_string(const std::string& s, FrontierStatus& out) {
    if (s == "PENDING")   { out = FrontierStatus::PENDING;   return true; }
    if (s == "ACTIVE")    { out = FrontierStatus::ACTIVE;    return true; }
    if (s == "SUCCEEDED") { out = FrontierStatus::SUCCEEDED; return true; }
    if (s == "BLOCKED")   { out = FrontierStatus::BLOCKED;   return true; }
    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
struct FrontierEntry {
    int32_t  id          {-1};
    double   x           {0.0};
    double   y           {0.0};
    FrontierStatus status{FrontierStatus::PENDING};
    int32_t  attempts    {0};

    using Clock    = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;
    TimePoint created_at {Clock::now()};
    TimePoint updated_at {Clock::now()};

    double distance_to(double ox, double oy) const {
        const double dx = x - ox, dy = y - oy;
        return std::sqrt(dx * dx + dy * dy);
    }

    // RViz2 colour (R, G, B) per status
    struct Rgb { float r, g, b; };
    Rgb status_color() const {
        switch (status) {
            case FrontierStatus::PENDING:   return {0.0f, 0.85f, 1.0f};
            case FrontierStatus::ACTIVE:    return {1.0f, 0.85f, 0.0f};
            case FrontierStatus::SUCCEEDED: return {0.0f, 1.0f,  0.3f};
            case FrontierStatus::BLOCKED:   return {1.0f, 0.2f,  0.2f};
        }
        return {0.5f, 0.5f, 0.5f};
    }
};

}
