// ros2_control SystemInterface for OpenRobot motor controllers on one CAN bus.
//
// SPDX-License-Identifier: MIT
#ifndef OPENROBOT_MOTOR_HARDWARE__OPENROBOT_MOTOR_SYSTEM_HPP_
#define OPENROBOT_MOTOR_HARDWARE__OPENROBOT_MOTOR_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "openrobot_motor/pc_transport.hpp"
#include "rclcpp/rclcpp.hpp"

namespace openrobot_motor_hardware {

class OpenRobotMotorSystem : public hardware_interface::SystemInterface {
public:
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

#if __has_include("hardware_interface/types/hardware_component_interface_params.hpp")
    // Jazzy (recent) and later
    CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams &params) override;
#else
    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
#endif
    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous) override;
    CallbackReturn on_error(const rclcpp_lifecycle::State &previous) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
    enum class Mode { MIT, VELOCITY, VELOCITY_ERPM, POSITION };

    struct Joint {
        std::string name;
        uint8_t id = 0;
        Mode mode = Mode::MIT;
        double kp = 10, kd = 0.5;       // MIT gains, Nm/rad and Nm s/rad
        double max_velocity = 1.0;      // position mode, rad/s output
        double pole_pairs = 0;          // velocity_erpm mode, from the motor setup
        bool zero_on_activate = false;  // MIT: present position becomes 0
        double gear = 1, kt = 0;        // read from the controller
        double cmd_pos = 0, cmd_vel = 0, cmd_eff = 0;
        double st_pos = 0, st_vel = 0, st_eff = 0;
        int missed = 0;
        std::unique_ptr<openrobot::Motor> motor;
    };

    CallbackReturn parse_info(const hardware_interface::HardwareInfo &info);
    static void fault_cb(uint8_t motor_id, uint8_t code, void *user);
    bool read_position(Joint &j);
    void take_status(Joint &j, const openrobot::Status &s);
    void take_mit_state(Joint &j, const openrobot::MitState &s);
    bool count_miss(Joint &j);
    CallbackReturn fail_activate(Joint &j);
    void release_all();
    void joints_motors_reset();
    rclcpp::Logger logger() const { return rclcpp::get_logger("OpenRobotMotorSystem"); }

    std::string interface_, channel_;
    uint32_t timeout_ms_ = 5;
    int max_missed_ = 3;
    // Declared before joints_ so the motors (which send Motor off when
    // destroyed) go first.
    std::unique_ptr<openrobot::Transport> transport_;
    std::unique_ptr<openrobot::MotorBus> bus_;
    std::vector<Joint> joints_;
    bool fault_ = false;
    uint8_t fault_id_ = 0, fault_code_ = 0;
};

}  // namespace openrobot_motor_hardware

#endif  // OPENROBOT_MOTOR_HARDWARE__OPENROBOT_MOTOR_SYSTEM_HPP_
