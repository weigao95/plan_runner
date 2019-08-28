//
// Created by wei on 8/28/19.
//

#include "arm_runner/trajectory_plan.h"

void arm_runner::JointTrajectoryPlan::computeCommand(
        const arm_runner::RobotArmMeasurement &measurement,
        const arm_runner::RobotCommunication &history,
        arm_runner::RobotArmCommand &command
) {
    auto t = measurement.time_stamp.since_plan_start_second;
    DRAKE_ASSERT(t > 0);
    q_command_cache = joint_trajectory_.value(t);
    v_command_cache = joint_velocity_trajectory_.value(t);

    // Write to command
    command.SetInvalid();
    for(auto i = 0; i < q_command_cache.size(); i++) {
        command.joint_position[i] = q_command_cache[i];
        command.joint_velocities[i] = v_command_cache[i];
    }
    command.position_validity = true;
    command.velocity_validity = true;
}

arm_runner::JointTrajectoryPlan::JointTrajectoryPlan(
        const arm_runner::JointTrajectoryPlan::PiecewisePolynomial &joint_trajectory)
        : joint_trajectory_(joint_trajectory) {
    DRAKE_ASSERT(joint_trajectory_.cols() == 1);
    joint_velocity_trajectory_ = joint_trajectory_.derivative(1);
}
