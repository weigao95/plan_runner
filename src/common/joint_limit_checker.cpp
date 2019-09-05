//
// Created by wei on 9/4/19.
//

#include "common/joint_limit_checker.h"


bool arm_runner::JointVelocityLimitChecker::HasRequiredField(
    const arm_runner::CommandInput &input,
    const arm_runner::RobotArmCommand &command
) {
    // Always true
    return true;
}


arm_runner::SafetyChecker::CheckResult arm_runner::JointVelocityLimitChecker::CheckSafety(
    const arm_runner::CommandInput &input,
    const arm_runner::RobotArmCommand &command
) {
    // The functor to check nan
    const int num_joint = input.robot_rbt->get_num_positions();
    auto has_nan = [num_joint](const double* array) -> bool {
        for(auto i = 0; i < num_joint; i++) {
            if(std::isnan(array[i]))
                return true;
        }
    };

    // Check it
    if(command.position_validity && has_nan(command.joint_position))
        return CheckResult{false, 0};
    if(command.velocity_validity && has_nan(command.joint_velocities))
        return CheckResult{false, 0};
    if(command.torque_validity && has_nan(command.joint_torque))
        return CheckResult{false, 0};

    // Check joint velocity limit on position
    const auto& command_history = input.robot_history->GetCommandHistory();
    if((!command_history.empty())) {
        const auto& prev_command = command_history.back();
        if(prev_command.position_validity && command.position_validity) {
            for(auto i = 0; i < num_joint; i++) {
                auto diff = std::abs(prev_command.joint_position[i] - command.joint_position[i]);
                if(diff > max_joint_velocity_ * input.control_interval_second) {
                    return CheckResult{false, 0};
                }
            }
        }
    }

    // Check joint velocity limit on velocity
    if(command.velocity_validity) {
        for(auto i = 0; i < num_joint; i++) {
            if(std::abs(command.joint_velocities[i]) > max_joint_velocity_)
                return CheckResult{false, 0};
        }
    }

    // Everything is OK
    return CheckResult{true, 0};
}