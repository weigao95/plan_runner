//
// Created by wei on 9/2/19.
//

#include "common/forceguard_checker.h"
#include "common/rbt_utils.h"


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


arm_runner::ExternalForceGuardChecker::ExternalForceGuardChecker(
        int body_idx, Eigen::Vector3d point_in_body,
        Eigen::Vector3d force_threshold, int force_expressed_in_frame
) : body_idx_(body_idx), point_on_body_(std::move(point_in_body)),
    force_threshold_(std::move(force_threshold)), force_expressed_in_frame_(force_expressed_in_frame)
{}


bool arm_runner::ExternalForceGuardChecker::HasRequiredField(
        const arm_runner::CommandInput &input,
        const arm_runner::RobotArmCommand &command
) {
    // Should have force measurement
    if(!input.latest_measurement->torque_validity)
        return false;
}

arm_runner::SafetyChecker::CheckResult arm_runner::ExternalForceGuardChecker::CheckSafety(
    const arm_runner::CommandInput &input,
    const arm_runner::RobotArmCommand &command
) {
    // Unpack the data
    const RigidBodyTree<double>& tree = *input.robot_rbt;
    const auto& cache = *input.measured_state_cache;

    // Transform the force into world
    int world_frame = RigidBodyTreeConstants::kWorldBodyIndex;
    Eigen::Isometry3d force_transform = tree.relativeTransform(cache, world_frame, force_expressed_in_frame_);
    Eigen::Matrix3d force_rotation = force_transform.linear();
    Eigen::Vector3d force_threshold_in_world = force_rotation * force_threshold_;

    // Compute the point jacobian
    jacobian_cache = tree.transformPointsJacobian(
        cache,
        point_on_body_,
        body_idx_, world_frame,
        true);
    torque_threshold_cache = jacobian_cache.transpose() * force_threshold_in_world;

    // Check safety
    int nq = input.robot_rbt->get_num_positions();
    Eigen::Map<const Eigen::VectorXd> tau(input.latest_measurement->joint_torque, nq);
    CheckResult result; result.set_safe();
    result.violation = tau.norm() / (torque_threshold_cache.norm() + 1e-4);
    result.is_safe = result.violation > 1.0;
    return result;
}


arm_runner::ExternalForceGuardChecker::Ptr arm_runner::ExternalForceGuardChecker::ConstructFromMessage(
    const RigidBodyTree<double> &tree,
    const robot_msgs::ExternalForceGuard &message
) {
    // Collect info about force
    const auto& force_msg = message.force;
    Eigen::Vector3d force_threshold = Eigen::Vector3d(
        force_msg.vector.x, force_msg.vector.y, force_msg.vector.z);

    // Current not supported
    Eigen::Vector3d point_in_body = Eigen::Vector3d::Zero();

    // Collect info about frame
    auto body_frame_name = message.body_frame;
    auto force_expressed_in_frame = force_msg.header.frame_id;
    if(force_expressed_in_frame == "base" || force_expressed_in_frame.empty())
        force_expressed_in_frame = "world";

    // Check the name
    if((!bodyOrFrameContainedInTree(tree, body_frame_name))
    || (!bodyOrFrameContainedInTree(tree, force_expressed_in_frame)))
        return nullptr;

    // OK
    int body_frame_index = getBodyOrFrameIndex(tree, body_frame_name);
    int force_expressed_in_frame_index = getBodyOrFrameIndex(tree, force_expressed_in_frame);
    return std::make_shared<ExternalForceGuardChecker>(
        body_frame_index, std::move(point_in_body),
        force_threshold, force_expressed_in_frame_index);
}