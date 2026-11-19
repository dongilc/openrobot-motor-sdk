// ros2_control SystemInterface for OpenRobot motor controllers on one CAN bus.
//
// Every joint is one controller (CAN ID). All values are on the output shaft:
// position rad, velocity rad/s, effort Nm (current x Kt). Each write() sends
// one command per joint and takes the joint state from the reply, so one
// control cycle costs one request per joint (two for velocity and position
// modes, which also read the multi-turn angle).
//
// SPDX-License-Identifier: MIT
#include "openrobot_motor_hardware/openrobot_motor_system.hpp"

#include <cmath>
#include <limits>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace openrobot_motor_hardware {

namespace hi = hardware_interface;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace {

const double DEG = M_PI / 180.0;

std::string param(const std::unordered_map<std::string, std::string> &p, const std::string &key,
                  const std::string &fallback) {
    auto it = p.find(key);
    return it == p.end() ? fallback : it->second;
}

double param_d(const std::unordered_map<std::string, std::string> &p, const std::string &key, double fallback) {
    auto it = p.find(key);
    return it == p.end() ? fallback : std::stod(it->second);
}

double or_zero(double v) { return std::isnan(v) ? 0.0 : v; }

}  // namespace

#if __has_include("hardware_interface/types/hardware_component_interface_params.hpp")
CallbackReturn OpenRobotMotorSystem::on_init(const hi::HardwareComponentInterfaceParams &params) {
    if (hi::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;
    return parse_info(params.hardware_info);
}
#else
CallbackReturn OpenRobotMotorSystem::on_init(const hi::HardwareInfo &info) {
    if (hi::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;
    return parse_info(info);
}
#endif

CallbackReturn OpenRobotMotorSystem::parse_info(const hi::HardwareInfo &info) {
    interface_ = param(info.hardware_parameters, "interface", "socketcan");
    channel_ = param(info.hardware_parameters, "channel", "can0");
    timeout_ms_ = (uint32_t)param_d(info.hardware_parameters, "timeout_ms", 5);
    max_missed_ = (int)param_d(info.hardware_parameters, "max_missed", 3);

    for (const auto &j : info.joints) {
        Joint jt;
        jt.name = j.name;
        jt.id = (uint8_t)param_d(j.parameters, "id", 0);
        const std::string mode = param(j.parameters, "mode", "mit");
        if (mode == "mit") {
            jt.mode = Mode::MIT;
        } else if (mode == "velocity") {
            jt.mode = Mode::VELOCITY;
        } else if (mode == "velocity_erpm") {
            jt.mode = Mode::VELOCITY_ERPM;
        } else if (mode == "position") {
            jt.mode = Mode::POSITION;
        } else {
            RCLCPP_ERROR(logger(), "joint %s: unknown mode '%s' (mit, velocity, velocity_erpm, position)", j.name.c_str(), mode.c_str());
            return CallbackReturn::ERROR;
        }
        jt.kp = param_d(j.parameters, "kp", 10.0);
        jt.kd = param_d(j.parameters, "kd", 0.5);
        jt.max_velocity = param_d(j.parameters, "max_velocity", 1.0);
        jt.pole_pairs = param_d(j.parameters, "pole_pairs", 0);
        if (jt.mode == Mode::VELOCITY_ERPM && jt.pole_pairs < 1) {
            RCLCPP_ERROR(logger(), "joint %s: mode velocity_erpm needs parameter pole_pairs", j.name.c_str());
            return CallbackReturn::ERROR;
        }
        jt.zero_on_activate = param(j.parameters, "zero_on_activate", "false") == "true";
        if (jt.id < 1 || jt.id > 32) {
            RCLCPP_ERROR(logger(), "joint %s: parameter id must be 1..32", j.name.c_str());
            return CallbackReturn::ERROR;
        }
        for (const auto &ci : j.command_interfaces) {
            bool ok = ci.name == hi::HW_IF_POSITION || ci.name == hi::HW_IF_VELOCITY || ci.name == hi::HW_IF_EFFORT;
            if (jt.mode == Mode::VELOCITY || jt.mode == Mode::VELOCITY_ERPM) ok = ci.name == hi::HW_IF_VELOCITY;
            if (jt.mode == Mode::POSITION) ok = ci.name == hi::HW_IF_POSITION;
            if (!ok) {
                RCLCPP_ERROR(logger(), "joint %s: command interface '%s' does not fit mode '%s'", j.name.c_str(),
                             ci.name.c_str(), mode.c_str());
                return CallbackReturn::ERROR;
            }
        }
        joints_.push_back(std::move(jt));
    }
    return CallbackReturn::SUCCESS;
}

std::vector<hi::StateInterface> OpenRobotMotorSystem::export_state_interfaces() {
    std::vector<hi::StateInterface> out;
    for (auto &j : joints_) {
        out.emplace_back(j.name, hi::HW_IF_POSITION, &j.st_pos);
        out.emplace_back(j.name, hi::HW_IF_VELOCITY, &j.st_vel);
        out.emplace_back(j.name, hi::HW_IF_EFFORT, &j.st_eff);
    }
    return out;
}

std::vector<hi::CommandInterface> OpenRobotMotorSystem::export_command_interfaces() {
    std::vector<hi::CommandInterface> out;
    for (size_t k = 0; k < joints_.size(); ++k) {
        auto &j = joints_[k];
        for (const auto &ci : info_.joints[k].command_interfaces) {
            double *v = ci.name == hi::HW_IF_POSITION ? &j.cmd_pos : ci.name == hi::HW_IF_VELOCITY ? &j.cmd_vel : &j.cmd_eff;
            out.emplace_back(j.name, ci.name, v);
        }
    }
    return out;
}

CallbackReturn OpenRobotMotorSystem::on_configure(const rclcpp_lifecycle::State &) {
    try {
        transport_ = openrobot::open_transport(interface_, channel_);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(logger(), "cannot open %s %s: %s", interface_.c_str(), channel_.c_str(), e.what());
        return CallbackReturn::ERROR;
    }
    bus_.reset(new openrobot::MotorBus(*transport_));
    bus_->on_fault(&OpenRobotMotorSystem::fault_cb, this);
    for (auto &j : joints_) {
        j.motor.reset(new openrobot::Motor(*bus_, j.id, true, timeout_ms_));
        openrobot::MitParams par;
        if (j.motor->mit_params(par) != openrobot::OK) {
            RCLCPP_ERROR(logger(), "joint %s: motor %d does not answer on %s %s", j.name.c_str(), j.id,
                         interface_.c_str(), channel_.c_str());
            return CallbackReturn::ERROR;
        }
        j.gear = par.gear_ratio > 0 ? par.gear_ratio : 1.0;
        j.kt = par.kt;
        RCLCPP_INFO(logger(), "joint %s: motor %d, gear %.2f, Kt %.3f Nm/A", j.name.c_str(), j.id, j.gear, j.kt);
    }
    return CallbackReturn::SUCCESS;
}

CallbackReturn OpenRobotMotorSystem::on_activate(const rclcpp_lifecycle::State &) {
    fault_ = false;
    for (auto &j : joints_) {
        j.missed = 0;
        if (j.mode == Mode::MIT) {
            if (j.motor->mit_enter() != openrobot::OK) return fail_activate(j);
            if (j.zero_on_activate && j.motor->mit_zero() != openrobot::OK) return fail_activate(j);
            // A zero-gain command produces no torque and returns the state.
            openrobot::MitState s;
            if (j.motor->mit(0, 0, 0, 0, 0, &s) != openrobot::OK) return fail_activate(j);
            take_mit_state(j, s);
        } else {
            openrobot::Status s;
            if (j.motor->status(s) != openrobot::OK || !read_position(j)) return fail_activate(j);
            take_status(j, s);
        }
        j.cmd_pos = j.st_pos;           // start by holding where the joint is
        j.cmd_vel = 0;
        j.cmd_eff = 0;
    }
    RCLCPP_INFO(logger(), "active: %zu joints", joints_.size());
    return CallbackReturn::SUCCESS;
}

CallbackReturn OpenRobotMotorSystem::on_deactivate(const rclcpp_lifecycle::State &) {
    release_all();
    return CallbackReturn::SUCCESS;
}

CallbackReturn OpenRobotMotorSystem::on_cleanup(const rclcpp_lifecycle::State &) {
    joints_motors_reset();
    return CallbackReturn::SUCCESS;
}

CallbackReturn OpenRobotMotorSystem::on_shutdown(const rclcpp_lifecycle::State &) {
    release_all();
    joints_motors_reset();
    return CallbackReturn::SUCCESS;
}

CallbackReturn OpenRobotMotorSystem::on_error(const rclcpp_lifecycle::State &) {
    release_all();
    return CallbackReturn::SUCCESS;
}

hi::return_type OpenRobotMotorSystem::read(const rclcpp::Time &, const rclcpp::Duration &) {
    if (!bus_) return hi::return_type::OK;
    bus_->poll();                       // fault notices
    if (fault_) {
        RCLCPP_ERROR(logger(), "motor %d reports fault %d (%s): stopping", fault_id_, fault_code_,
                     openrobot::fault_name(fault_code_));
        return hi::return_type::ERROR;
    }
    // MIT joints get their state from the reply in write(). The others read
    // the multi-turn angle here; speed and current come with the reply.
    for (auto &j : joints_) {
        if (j.mode == Mode::MIT) continue;
        if (!read_position(j) && !count_miss(j)) return hi::return_type::ERROR;
    }
    return hi::return_type::OK;
}

hi::return_type OpenRobotMotorSystem::write(const rclcpp::Time &, const rclcpp::Duration &) {
    for (auto &j : joints_) {
        openrobot::Result r;
        if (j.mode == Mode::MIT) {
            openrobot::MitState s;
            double p = std::isnan(j.cmd_pos) ? j.st_pos : j.cmd_pos;
            r = j.motor->mit((float)p, (float)or_zero(j.cmd_vel), (float)j.kp, (float)j.kd, (float)or_zero(j.cmd_eff), &s);
            if (r == openrobot::OK) take_mit_state(j, s);
        } else if (j.mode == Mode::VELOCITY) {
            openrobot::Status s;
            r = j.motor->speed_dps((float)(or_zero(j.cmd_vel) / DEG * j.gear), &s);
            if (r == openrobot::OK) take_status(j, s);
        } else if (j.mode == Mode::VELOCITY_ERPM) {
            // output rad/s -> motor RPM -> electrical RPM
            openrobot::Status s;
            r = j.motor->speed_erpm((float)(or_zero(j.cmd_vel) * j.gear * 60.0 / (2 * M_PI) * j.pole_pairs), &s);
            if (r == openrobot::OK) take_status(j, s);
        } else {
            openrobot::Status s;
            if (std::isnan(j.cmd_pos)) continue;
            r = j.motor->position((float)(j.cmd_pos / DEG * j.gear), (float)(j.max_velocity / DEG * j.gear), &s);
            if (r == openrobot::BAD_ARGUMENT) {
                RCLCPP_ERROR(logger(), "joint %s: max_velocity out of range", j.name.c_str());
                return hi::return_type::ERROR;
            }
            if (r == openrobot::OK) take_status(j, s);
        }
        if (r == openrobot::OK) {
            j.missed = 0;
        } else if (!count_miss(j)) {
            return hi::return_type::ERROR;
        }
    }
    return hi::return_type::OK;
}

// ---------------------------------------------------------------- helpers --

void OpenRobotMotorSystem::fault_cb(uint8_t motor_id, uint8_t code, void *user) {
    auto *self = static_cast<OpenRobotMotorSystem *>(user);
    self->fault_ = true;
    self->fault_id_ = motor_id;
    self->fault_code_ = code;
}

bool OpenRobotMotorSystem::read_position(Joint &j) {
    double deg;
    if (j.motor->multi_turn_deg(deg) != openrobot::OK) return false;
    j.st_pos = deg * DEG / j.gear;
    return true;
}

void OpenRobotMotorSystem::take_status(Joint &j, const openrobot::Status &s) {
    j.st_vel = s.speed_dps * DEG / j.gear;
    j.st_eff = s.current_a * j.kt;
}

void OpenRobotMotorSystem::take_mit_state(Joint &j, const openrobot::MitState &s) {
    j.st_pos = s.position_rad;
    j.st_vel = s.speed_dps * DEG;       // output side in MIT
    j.st_eff = s.current_a * j.kt;
}

// false once a joint has missed max_missed replies in a row.
bool OpenRobotMotorSystem::count_miss(Joint &j) {
    if (++j.missed < max_missed_) return true;
    RCLCPP_ERROR(logger(), "joint %s: motor %d missed %d replies in a row: stopping", j.name.c_str(), j.id, j.missed);
    return false;
}

CallbackReturn OpenRobotMotorSystem::fail_activate(Joint &j) {
    RCLCPP_ERROR(logger(), "joint %s: motor %d does not answer", j.name.c_str(), j.id);
    release_all();
    return CallbackReturn::ERROR;
}

void OpenRobotMotorSystem::release_all() {
    for (auto &j : joints_) {
        if (!j.motor) continue;
        if (j.mode == Mode::MIT) j.motor->mit_exit();
        j.motor->off();
    }
}

void OpenRobotMotorSystem::joints_motors_reset() {
    for (auto &j : joints_) j.motor.reset();   // each sends Motor off
    bus_.reset();
    transport_.reset();
}

}  // namespace openrobot_motor_hardware

PLUGINLIB_EXPORT_CLASS(openrobot_motor_hardware::OpenRobotMotorSystem, hardware_interface::SystemInterface)
