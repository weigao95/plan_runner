//
// Created by wei on 9/2/19.
//

#include "common/forceguard_checker.h"


arm_runner::SafetyChecker::CheckResult arm_runner::TotalForceGuardChecker::CheckSafety(
        const arm_runner::CommandInput &input,
        const arm_runner::RobotArmCommand &command
) {
    // When it comes here the measurement should be valid
    DRAKE_ASSERT(input.is_valid());
    DRAKE_ASSERT(HasRequiredField(input, command));

    // Get type
    int nq = input.robot_rbt->get_num_positions();
    Eigen::Map<const Eigen::VectorXd> tau(input.latest_measurement->joint_torque, nq);

    // Check it
    CheckResult result; result.set_safe();
    result.violation = tau.norm() / threshold_;
    result.is_safe = result.violation > 1.0;
    return result;
}


arm_runner::SafetyChecker::CheckResult arm_runner::ExternalForceGuardChecker::CheckSafety(
    const arm_runner::CommandInput &input,
    const arm_runner::RobotArmCommand &command
) {

}

arm_runner::ExternalForceGuardChecker::ExternalForceGuardChecker(
    int body_idx, Eigen::Vector3d point_in_body,
    Eigen::Vector3d force_threshold, int force_expressed_in_frame
) : body_idx_(body_idx), point_on_body_(std::move(point_in_body)),
    force_threshold_(std::move(force_threshold)), force_expressed_in_frame_(force_expressed_in_frame)
{

}
