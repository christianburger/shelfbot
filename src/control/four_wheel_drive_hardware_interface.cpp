#include "shelfbot/four_wheel_drive_hardware_interface.hpp"
#include "shelfbot/microros_communication.hpp"
#include "shelfbot/shelfbot_utils.hpp"  // pulls in log_zip.hpp
#include <sstream>

namespace shelfbot {

FourWheelDriveHardwareInterface::FourWheelDriveHardwareInterface() {
    log_info("FourWheelDriveHardwareInterface", "Constructor",
             "Hardware interface constructor called.");
}

hardware_interface::CallbackReturn
FourWheelDriveHardwareInterface::on_init(const hardware_interface::HardwareInfo& info)
{
    log_zip_s("HW", "INIT", {{"st", "begin"}});

    if (hardware_interface::SystemInterface::on_init(info)
            != hardware_interface::CallbackReturn::SUCCESS) {
        log_zip_s("HW", "INIT", {{"st", "base_fail"}});
        log_error("FourWheelDriveHardwareInterface", "on_init", "Base class on_init failed.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    hw_positions_.resize(info_.joints.size(), 0.0);
    hw_velocities_.resize(info_.joints.size(), 0.0);
    hw_velocity_commands_.resize(info_.joints.size(), 0.0);
    hw_max_speeds_.resize(info_.joints.size(), 10.0);

    std::string comm_type = info_.hardware_parameters.at("communication_type");

    node_       = std::make_shared<rclcpp::Node>("shelfbot_odometry_node");
    node_spinner_ = std::thread([this]() { rclcpp::spin(node_); });

    if (comm_type == "microros") {
        comm_ = std::make_unique<MicroRosCommunication>();
        if (!comm_->open("")) {
            log_zip_s("HW", "INIT", {{"st", "comm_fail"}, {"type", "uros"}});
            log_error("FourWheelDriveHardwareInterface", "on_init",
                      "Failed to open micro-ROS communication.");
            return hardware_interface::CallbackReturn::ERROR;
        }
    } else {
        log_zip_s("HW", "INIT", {{"st", "bad_comm"}, {"type", comm_type}});
        log_error("FourWheelDriveHardwareInterface", "on_init",
                  "Unsupported communication type: " + comm_type);
        return hardware_interface::CallbackReturn::ERROR;
    }

    odometry_ = std::make_unique<FourWheelDriveOdometry>(
        node_, node_->get_clock(),
        std::stod(info_.hardware_parameters.at("wheel_separation")),
        std::stod(info_.hardware_parameters.at("wheel_radius")));

    // ── log_zip: init complete ────────────────────────────────────────────
    log_zip("HW", "INIT", {
        {"joints", (double)info_.joints.size()},
        {"sep",    std::stod(info_.hardware_parameters.at("wheel_separation"))},
        {"rad",    std::stod(info_.hardware_parameters.at("wheel_radius"))}
    });
    log_info("FourWheelDriveHardwareInterface", "on_init", "--- on_init successful ---");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
FourWheelDriveHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state)
{
    log_zip_s("HW", "CFG", {{"from", previous_state.label()}});
    log_info("FourWheelDriveHardwareInterface", "on_configure", "--- on_configure successful ---");
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
FourWheelDriveHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
        state_interfaces.emplace_back(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
    }
    log_zip("HW", "EXPO", {{"cnt", (double)state_interfaces.size()}});
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
FourWheelDriveHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_commands_[i]);
        command_interfaces.emplace_back(
            info_.joints[i].name, "max_speed", &hw_max_speeds_[i]);
    }
    log_zip("HW", "EXPC", {{"cnt", (double)command_interfaces.size()}});
    return command_interfaces;
}

hardware_interface::CallbackReturn
FourWheelDriveHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
    std::fill(hw_velocity_commands_.begin(), hw_velocity_commands_.end(), 0.0);
    std::fill(hw_positions_.begin(),         hw_positions_.end(),         0.0);
    std::fill(hw_velocities_.begin(),        hw_velocities_.end(),        0.0);

    if (comm_) comm_->writeSpeedsToHardware(hw_velocity_commands_);

    // ── log_zip: activated ────────────────────────────────────────────────
    log_zip_s("HW", "ACT", {{"from", previous_state.label()}, {"st", "ok"}});
    log_info("FourWheelDriveHardwareInterface", "on_activate", "--- on_activate successful ---");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
FourWheelDriveHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
    if (comm_) comm_->close();
    log_zip_s("HW", "DEACT", {{"from", previous_state.label()}, {"st", "ok"}});
    log_info("FourWheelDriveHardwareInterface", "on_deactivate",
             "--- on_deactivate successful ---");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
FourWheelDriveHardwareInterface::read(const rclcpp::Time& time,
                                      const rclcpp::Duration& period)
{
    if (!comm_) {
        log_warn("FourWheelDriveHardwareInterface", "read",
                 "Communication interface not available.");
        return hardware_interface::return_type::OK;
    }

    bool healthy = comm_->is_communication_healthy();

    if (!healthy) {
        static auto last_warn_time = time;
        if ((time - last_warn_time).seconds() > 5.0) {
            log_zip("HW", "RD", {{"hlth", 0}, {"skip", 0}});
            log_warn("FourWheelDriveHardwareInterface", "read",
                    "Micro-ROS communication unhealthy - attempting read anyway");
            last_warn_time = time;
        }
    }

    if (!comm_->readStateFromHardware(hw_positions_)) {
        if (healthy) {
            log_zip("HW", "RD", {{"hlth", 1}, {"ok", 0}});
            log_warn("FourWheelDriveHardwareInterface", "read",
                     "Failed to read from hardware despite healthy communication check");
        }
        return hardware_interface::return_type::OK;
    }

    // ── READ‑SIDE FLIP: Negate left‑side positions for real robot ─────────
    // This makes the odometry see increasing left positions when moving forward.
    hw_positions_[0] = -hw_positions_[0];  // front left
    hw_positions_[2] = -hw_positions_[2];  // back left
    // ─────────────────────────────────────────────────────────────────────

    // ── log_zip: successful read with raw positions ───────────────────────
    log_zip("HW", "RD", {
        {"hlth", healthy ? 1.0 : 0.0},
        {"p0",   hw_positions_[0]},
        {"p1",   hw_positions_[1]},
        {"p2",   hw_positions_[2]},
        {"p3",   hw_positions_[3]}
    });

    if (odometry_) odometry_->update(hw_positions_, period);

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
FourWheelDriveHardwareInterface::write(const rclcpp::Time& time,
                                       const rclcpp::Duration& /*period*/)
{
    if (!comm_) {
        log_error("FourWheelDriveHardwareInterface", "write",
                  "Communication interface not available.");
        return hardware_interface::return_type::ERROR;
    }

    bool healthy = comm_->is_communication_healthy();

    if (!healthy) {
        static auto last_warn_time = time;
        if ((time - last_warn_time).seconds() > 5.0) {
            log_zip("HW", "WR", {{"hlth", 0}, {"skip", 0}});
            log_warn("FourWheelDriveHardwareInterface", "write",
                    "Micro-ROS communication unhealthy - attempting write anyway");
            last_warn_time = time;
        }
    }

    // ── COMMAND‑SIDE FLIP: Negate left‑side commands for real robot ───────
    // This makes left motors receive negative velocities (CCW) for forward motion.
    hw_velocity_commands_[0] = -hw_velocity_commands_[0];  // front left
    hw_velocity_commands_[2] = -hw_velocity_commands_[2];  // back left
    // ─────────────────────────────────────────────────────────────────────

    if (!comm_->writeSpeedsToHardware(hw_velocity_commands_)) {
        if (healthy) {
            log_zip("HW", "WR", {{"hlth", 1}, {"ok", 0}});
            log_error("FourWheelDriveHardwareInterface", "write",
                     "Failed to write speeds to hardware despite healthy communication check");
            return hardware_interface::return_type::ERROR;
        }
        log_zip("HW", "WR", {{"hlth", 0}, {"ok", 0}});
        return hardware_interface::return_type::OK;
    }

    // ── log_zip: write succeeded with velocity commands ───────────────────
    log_zip("HW", "WR", {
        {"hlth", healthy ? 1.0 : 0.0},
        {"c0",   hw_velocity_commands_[0]},
        {"c1",   hw_velocity_commands_[1]},
        {"c2",   hw_velocity_commands_[2]},
        {"c3",   hw_velocity_commands_[3]}
    });

    return hardware_interface::return_type::OK;
}

} // namespace shelfbot

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    shelfbot::FourWheelDriveHardwareInterface,
    hardware_interface::SystemInterface)