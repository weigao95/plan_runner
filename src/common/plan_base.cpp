//
// Created by wei on 8/27/19.
//

#include "common/plan_base.h"


void arm_runner::RobotPlanBase::ComputeCommand(
        const CommandInput& input,
        arm_runner::RobotArmCommand &command
) {
    if(status_ == PlanStatus::Running)
        computeCommand(input, command);
    else {
        // Should not happen, send warning
        KeepCurrentConfigurationCommand(*input.latest_measurement, command);
    }
}


void arm_runner::RobotPlanBase::KeepCurrentConfigurationCommand(
        const arm_runner::RobotArmMeasurement &measurement,
        arm_runner::RobotArmCommand &command
) {
    // Seems OK
    for(auto i = 0; i < MAX_NUM_JOINTS; i++) {
        command.joint_position[i] = measurement.joint_position[i];
        command.joint_torque[i] = 0;
        command.joint_velocities[i] = 0;
    }

    // Setup the command validity flag
    command.position_validity = true;
    command.velocity_validity = true;
    command.torque_validity = false;
}


bool arm_runner::RobotPlanBase::CheckSafety(
    const arm_runner::CommandInput &input,
    const arm_runner::RobotArmCommand &command
) {
    // Any fails of the checker will result in safety failed
    for(auto& checker : safety_checker_stack_) {
        bool is_safe = true;
        if(checker->HasRequiredField(input, command)) {
            is_safe = checker->CheckSafety(input, command);
        }
        if(!is_safe)
            return false;
    }

    // No fail in the checkers
    return true;
}
