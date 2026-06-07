#include "shelfbot/four_wheel_drive_controller.hpp"
#include "shelfbot/shelfbot_utils.hpp"  // pulls in log_zip.hpp

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using std::string;
using std::vector;

namespace shelfbot {

FourWheelDriveController::FourWheelDriveController()
    : controller_interface::ControllerInterface(), cmd_vel_timeout_(0, 0)
{}

CallbackReturn FourWheelDriveController::on_init() {
    try {
        auto_declare<vector<string>>("joint_names",             vector<string>());
        auto_declare<vector<string>>("front_left_joint_names",  vector<string>());
        auto_declare<vector<string>>("back_left_joint_names",   vector<string>());
        auto_declare<vector<string>>("front_right_joint_names", vector<string>());
        auto_declare<vector<string>>("back_right_joint_names",  vector<string>());
        auto_declare<double>("wheel_separation", 0.0);
        auto_declare<double>("wheel_radius",     0.0);
        auto_declare<double>("cmd_vel_timeout",  0.5);
    } catch (const std::exception& e) {
        log_zip_s("CTRL", "INIT", {{"st", "err"}});
        log_error("FourWheelDriveController", "on_init",
                  string("Exception: ") + e.what());
        return CallbackReturn::ERROR;
    }
    log_zip_s("CTRL", "INIT", {{"st", "ok"}});
    return CallbackReturn::SUCCESS;
}

CallbackReturn FourWheelDriveController::on_configure(
    const rclcpp_lifecycle::State& previous_state)
{
    joint_names_             = get_node()->get_parameter("joint_names").as_string_array();
    front_left_joint_names_  = get_node()->get_parameter("front_left_joint_names").as_string_array();
    back_left_joint_names_   = get_node()->get_parameter("back_left_joint_names").as_string_array();
    front_right_joint_names_ = get_node()->get_parameter("front_right_joint_names").as_string_array();
    back_right_joint_names_  = get_node()->get_parameter("back_right_joint_names").as_string_array();
    wheel_separation_        = get_node()->get_parameter("wheel_separation").as_double();
    wheel_radius_            = get_node()->get_parameter("wheel_radius").as_double();
    cmd_vel_timeout_         = rclcpp::Duration::from_seconds(
        get_node()->get_parameter("cmd_vel_timeout").as_double());

    for (const auto& joint : joint_names_) {
        axis_positions_[joint] = 0.0;
        axis_commands_[joint]  = 0.0;
    }

    // ── log_zip: controller configured ───────────────────────────────────
    log_zip("CTRL", "CFG", {
        {"jcnt", (double)joint_names_.size()},
        {"sep",  wheel_separation_},
        {"rad",  wheel_radius_},
        {"tmo",  cmd_vel_timeout_.seconds()}
    });

    cmd_vel_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
        "~/cmd_vel", rclcpp::SystemDefaultsQoS(),
        std::bind(&FourWheelDriveController::cmd_vel_callback, this,
                  std::placeholders::_1));

    direct_cmd_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
        "~/direct_commands", 10,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            if (msg->data.size() == joint_names_.size()) {
                for (size_t i = 0; i < joint_names_.size(); ++i)
                    axis_commands_[joint_names_[i]] = msg->data[i];
                last_cmd_vel_ = nullptr;
            }
        });

    joint_state_publisher_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
        "joint_states", 10);

    return CallbackReturn::SUCCESS;
}

void FourWheelDriveController::cmd_vel_callback(
    const std::shared_ptr<geometry_msgs::msg::Twist> msg)
{
    last_cmd_vel_      = msg;
    last_cmd_vel_time_ = get_node()->now();

    // ── log_zip: velocity goal received ──────────────────────────────────
    log_zip("CTRL", "GOAL", {{"vx", msg->linear.x}, {"wz", msg->angular.z}});
}

CallbackReturn FourWheelDriveController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    for (size_t i = 0; i < joint_names_.size(); ++i)
        axis_commands_[joint_names_[i]] = state_interfaces_[i].get_value();
    last_cmd_vel_ = nullptr;

    log_zip_s("CTRL", "ACT", {{"st", "ok"}});
    return CallbackReturn::SUCCESS;
}

CallbackReturn FourWheelDriveController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    log_zip_s("CTRL", "DEACT", {{"st", "ok"}});
    return CallbackReturn::SUCCESS;
}

controller_interface::return_type FourWheelDriveController::update(
    const rclcpp::Time& time, const rclcpp::Duration& period)
{
    // Read wheel positions
    for (size_t i = 0; i < joint_names_.size(); ++i)
        axis_positions_[joint_names_[i]] = state_interfaces_[i].get_value();

    bool cmd_active = last_cmd_vel_ && (time - last_cmd_vel_time_) < cmd_vel_timeout_;

    if (cmd_active) {
        const double linear_vel  = last_cmd_vel_->linear.x;
        const double angular_vel = last_cmd_vel_->angular.z;

        const double vel_left  = (linear_vel - angular_vel * wheel_separation_ * 0.5) / wheel_radius_;
        const double vel_right = (linear_vel + angular_vel * wheel_separation_ * 0.5) / wheel_radius_;

        axis_commands_[front_left_joint_names_[0]]  = vel_left;
        axis_commands_[front_right_joint_names_[0]] = vel_right;
        axis_commands_[back_left_joint_names_[0]]   = vel_left;
        axis_commands_[back_right_joint_names_[0]]  = vel_right;
    } else {
        // Timeout or no command
        bool timed_out = last_cmd_vel_ && (time - last_cmd_vel_time_) >= cmd_vel_timeout_;
        if (timed_out) {
            // ── log_zip: command timeout (log once per timeout event) ─────
            static rclcpp::Time last_timeout_log{0, 0, RCL_ROS_TIME};
            if ((time - last_timeout_log).seconds() > 1.0) {
                log_zip("CTRL", "TMO", {
                    {"age", (time - last_cmd_vel_time_).seconds()},
                    {"lim", cmd_vel_timeout_.seconds()}
                });
                last_timeout_log = time;
            }
        }
        for (const auto& joint : joint_names_)
            axis_commands_[joint] = 0.0;
    }

    for (size_t i = 0; i < joint_names_.size(); ++i)
        command_interfaces_[i].set_value(axis_commands_[joint_names_[i]]);

    publish_joint_states();
    return controller_interface::return_type::OK;
}

void FourWheelDriveController::publish_joint_states() {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp    = get_node()->now();
    msg.header.frame_id = "base_link";

    for (const auto& joint : joint_names_) {
        msg.name.push_back(joint);
        msg.position.push_back(axis_positions_[joint]);
    }

    std::stringstream ss;
    ss << "[hw_positions] publish_joint_states: positions = [";
    for (size_t i = 0; i < msg.position.size(); ++i)
        ss << msg.position[i] << (i < msg.position.size() - 1 ? ", " : "");
    ss << "]";
    log_info("FourWheelDriveController", "publish_joint_states", ss.str());

    joint_state_publisher_->publish(msg);
}

InterfaceConfiguration
FourWheelDriveController::command_interface_configuration() const {
    InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto& joint : joint_names_)
        config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    return config;
}

InterfaceConfiguration
FourWheelDriveController::state_interface_configuration() const {
    InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto& joint : joint_names_)
        config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    return config;
}

} // namespace shelfbot

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    shelfbot::FourWheelDriveController,
    controller_interface::ControllerInterface)
