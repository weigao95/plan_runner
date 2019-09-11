//
// Created by wei on 9/11/19.
//

#include "common/robot_communication.h"
#include "robot_plan/joint_impedance_limit_dq.h"

void plan_runner::JointImpedanceControllerLimitDq::ProcessCommand(
    const plan_runner::RobotCommunication & history,
    plan_runner::RobotArmCommand & command
) {
    // Get measurement
    const auto& latest_measurement = history.GetMeasurementHistory().back();
    const double* measured_joint_position = latest_measurement.joint_position;

    // Check the command
    bool need_scale = false;
    double scale_factor = 1.0;
    constexpr int KUKA_IIWA_N_JOINTS = 7;
    for(auto i = 0; i < KUKA_IIWA_N_JOINTS; i++) {
        auto command_position_i = command.joint_position[i];
        auto q_diff_i = std::abs(command_position_i - measured_joint_position[i]);
        if(q_diff_i > max_dq_) {
            need_scale = true;
            scale_factor = std::min(scale_factor, q_diff_i / max_dq_);
        }
    }

    // Everything goes well
    if(!need_scale) return;

    // Do scale
    for(auto i = 0; i < KUKA_IIWA_N_JOINTS; i++) {
        // Scale the position
        auto command_position_i = command.joint_position[i];
        auto q_diff_i = command_position_i - measured_joint_position[i];
        command.joint_position[i] = q_diff_i * scale_factor + measured_joint_position[i];

        // Scale the velocity
        if(command.velocity_validity)
            command.joint_velocities[i] *= scale_factor;
    }
}
