//
// Created by wei on 8/27/19.
//

#include "common/plan_base.h"


bool arm_runner::RobotPlanBase::CheckSafety(
        const arm_runner::CommandInput &input,
        const arm_runner::RobotArmCommand &command
) {
    // Any fails of the checker will result in safety failed
    for(auto& checker : safety_checker_stack_) {
        bool is_safe = true;
        if(checker->HasRequiredField(input, command)) {
            auto result = checker->CheckSafety(input, command);
            is_safe = result.is_safe;
        }
        if(!is_safe)
            return false;
    }

    // No fail in the checkers
    return true;
}


// The method about keep current configuration
arm_runner::KeepCurrentConfigurationPlan::KeepCurrentConfigurationPlan() {
    kept_configuration_.set_invalid();
}


void arm_runner::KeepCurrentConfigurationPlan::InitializePlan(const arm_runner::CommandInput &input) {
    kept_configuration_ = *input.latest_measurement;
    RobotPlanBase::InitializePlan(input);
}


void arm_runner::KeepCurrentConfigurationPlan::ComputeCommand(
    const arm_runner::CommandInput &input,
    arm_runner::RobotArmCommand &command
) {
    CopyConfigurationToCommand(kept_configuration_, command);
    command.time_stamp = input.latest_measurement->time_stamp;
}


void arm_runner::KeepCurrentConfigurationPlan::CopyConfigurationToCommand(
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