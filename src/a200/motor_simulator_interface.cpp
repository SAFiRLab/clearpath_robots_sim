#include "clearpath_robots_sim/a200/motor_simulator_interface.hpp"


CallbackReturn MotorSimulatorInterface::on_init(const hardware_interface::HardwareInfo & info) override
{
    hw_commands_.resize(info.joints.size(), 0.0);
    hw_states_pos_.resize(info.joints.size(), 0.0);
    hw_states_vel_.resize(info.joints.size(), 0.0);
    return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> MotorSimulatorInterface::export_state_interfaces() override
{
    std::vector<StateInterface> states;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        states.emplace_back(info_.joints[i].name, "position", &hw_states_pos_[i]);
        states.emplace_back(info_.joints[i].name, "velocity", &hw_states_vel_[i]);
    }
    return states;
}

std::vector<CommandInterface> MotorSimulatorInterface::export_command_interfaces() override
{
    std::vector<CommandInterface> commands;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        commands.emplace_back(info_.joints[i].name, "velocity", &hw_commands_[i]);
    }
    return commands;
}

return_type MotorSimulatorInterface::read(const rclcpp::Time &, const rclcpp::Duration &) override
{
    // Read wheel states FROM Unity
    return return_type::OK;
}

return_type MotorSimulatorInterface::write(const rclcpp::Time &, const rclcpp::Duration &) override
{
    // Send wheel velocity commands TO Unity
    return return_type::OK;
}
