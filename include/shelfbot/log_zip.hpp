#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// log_zip — compact single-line structured telemetry log
//
// Format (one line per call):
//   [<epoch_ms>][<MODULE>:<TAG>] k=v,k=v,...
//
// Writes to:
//   • A rolling text file  ~/.ros/log/shelfbot_zip.log  (≤ MAX_FILE_BYTES,
//     then rotated to shelfbot_zip.log.1)
//   • stderr via RCLCPP_DEBUG so it appears with --log-level debug if wanted
//
// Usage:
//   log_zip("ODO", "UPD", {{"x",x_},{"y",y_},{"th",theta_},{"dt",dt}});
//   log_zip("HW",  "RD",  {{"ok",1},{"p0",hw_positions_[0]}});
//   log_zip("LC",  "ACT", {{"st","ok"}});
//
// Keep keys ≤ 4 chars, values numeric or single-word.  No spaces in values.
// ─────────────────────────────────────────────────────────────────────────────

#include <array>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <initializer_list>
#include <mutex>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <filesystem>

#include <rclcpp/rclcpp.hpp>

namespace shelfbot {

// ── Tunables ──────────────────────────────────────────────────────────────────
static constexpr std::size_t LOG_ZIP_MAX_FILE_BYTES = 4 * 1024 * 1024; // 4 MB
static constexpr int         LOG_ZIP_PRECISION       = 4;               // decimal digits

// ── Internal singleton ────────────────────────────────────────────────────────
namespace detail {

class ZipSink {
public:
    static ZipSink& instance() {
        static ZipSink s;
        return s;
    }

    void write(const char* module, const char* tag,
               std::initializer_list<std::pair<std::string_view, double>> kv)
    {
        // Build line
        char line[512];
        int  pos = 0;

        // Timestamp (ms since Unix epoch)
        auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        pos += std::snprintf(line + pos, sizeof(line) - pos,
                             "[%lld][%s:%s]", (long long)now_ms, module, tag);

        bool first = true;
        for (const auto& [k, v] : kv) {
            pos += std::snprintf(line + pos, sizeof(line) - pos,
                                 first ? " %.*s=%.*f"
                                       : ",%.*s=%.*f",
                                 (int)k.size(), k.data(),
                                 LOG_ZIP_PRECISION, v);
            first = false;
        }
        // Clamp, append newline, null-terminate
        if (pos >= (int)sizeof(line) - 2) pos = (int)sizeof(line) - 2;
        line[pos++] = '\n';  // pos now points ONE PAST the \n
        line[pos]   = '\0';  // safe sentinel, not written

        std::lock_guard<std::mutex> lk(mu_);
        ensure_open();
        if (file_.is_open()) {
            file_.write(line, pos); // write all bytes INCLUDING the \n
            file_.flush();
            bytes_written_ += pos;
            if (bytes_written_ >= LOG_ZIP_MAX_FILE_BYTES) rotate();
        }
        // Mirror to ROS debug channel (zero-cost when level > DEBUG)
        RCLCPP_DEBUG(rclcpp::get_logger("log_zip"), "%s", line);
    }

    // String-value overload stored as double (encoded as 0.0, string separately)
    void write_str(const char* module, const char* tag,
                   std::initializer_list<std::pair<std::string_view, std::string_view>> kv)
    {
        char line[512];
        int  pos = 0;
        auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        pos += std::snprintf(line + pos, sizeof(line) - pos,
                             "[%lld][%s:%s]", (long long)now_ms, module, tag);
        bool first = true;
        for (const auto& [k, v] : kv) {
            pos += std::snprintf(line + pos, sizeof(line) - pos,
                                 first ? " %.*s=%.*s"
                                       : ",%.*s=%.*s",
                                 (int)k.size(), k.data(),
                                 (int)v.size(), v.data());
            first = false;
        }
        // Clamp, append newline, null-terminate
        if (pos >= (int)sizeof(line) - 2) pos = (int)sizeof(line) - 2;
        line[pos++] = '\n';  // pos now points ONE PAST the \n
        line[pos]   = '\0';  // safe sentinel, not written

        std::lock_guard<std::mutex> lk(mu_);
        ensure_open();
        if (file_.is_open()) {
            file_.write(line, pos); // write all bytes INCLUDING the \n
            file_.flush();
            bytes_written_ += pos;
            if (bytes_written_ >= LOG_ZIP_MAX_FILE_BYTES) rotate();
        }
        RCLCPP_DEBUG(rclcpp::get_logger("log_zip"), "%s", line);
    }

private:
    ZipSink() = default;

    void ensure_open() {
        if (file_.is_open()) return;
        // Try ~/.ros/log/ first, fall back to /tmp
        const char* home = std::getenv("HOME");
        std::string dir  = home ? std::string(home) + "/.ros/log" : "/tmp";
        try { std::filesystem::create_directories(dir); } catch (...) { dir = "/tmp"; }
        path_ = dir + "/shelfbot_zip.log";
        file_.open(path_, std::ios::app);
        if (!file_.is_open()) {
            // Last resort
            path_ = "/tmp/shelfbot_zip.log";
            file_.open(path_, std::ios::app);
        }
        // Measure existing size
        if (file_.is_open()) {
            file_.seekp(0, std::ios::end);
            bytes_written_ = static_cast<std::size_t>(file_.tellp());
        }
    }

    void rotate() {
        file_.close();
        std::string bak = path_ + ".1";
        std::rename(path_.c_str(), bak.c_str());
        file_.open(path_, std::ios::trunc);
        bytes_written_ = 0;
    }

    std::mutex    mu_;
    std::ofstream file_;
    std::string   path_;
    std::size_t   bytes_written_{0};
};

} // namespace detail

// ── Public API ────────────────────────────────────────────────────────────────

/// Numeric key-value pairs.  Keys should be ≤ 4 chars.
inline void log_zip(const char* module, const char* tag,
                    std::initializer_list<std::pair<std::string_view, double>> kv)
{
    detail::ZipSink::instance().write(module, tag, kv);
}

/// String key-value pairs (mix of text and numbers as text).
inline void log_zip_s(const char* module, const char* tag,
                      std::initializer_list<std::pair<std::string_view, std::string_view>> kv)
{
    detail::ZipSink::instance().write_str(module, tag, kv);
}

} // namespace shelfbot
