//
// Created by wei on 8/27/19.
//

#include "arm_runner/plan_base.h"

void arm_runner::RobotPlanBase::keepCurrentConfigurationCommand(
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

void arm_runner::RobotPlanBase::ComputeCommand(
        const arm_runner::RobotArmMeasurement &measurement,
        const RobotCommunication& history,
        arm_runner::RobotArmCommand &command
) {
    if(status_ == PlanStatus::Running)
        computeCommand(measurement, history, command);
    else {
        // Should not happen, send warning
        keepCurrentConfigurationCommand(measurement, command);
    }
}