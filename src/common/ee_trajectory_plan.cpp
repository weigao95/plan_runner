//
// Created by wei on 8/30/19.
//

#include "common/ee_trajectory_plan.h"

// The world frame in drake is always zero
const int arm_runner::EETrajectoryPlan::world_frame = 0;


// The workforce function
void arm_runner::EETrajectoryPlan::computeCommand(
    const arm_runner::CommandInput &input,
    arm_runner::RobotArmCommand &command
) {
    // Collect data
    DRAKE_ASSERT(input.is_valid());
    const RigidBodyTree<double>& tree = *input.robot_rbt;
    const auto& cache = *input.measured_state_cache;
    auto t = input.latest_measurement->time_stamp.since_plan_start_second;

    // Compute the desired frame transformation
    Eigen::Vector3d ee_position_ref = ee_xyz_trajectory_.value(t);
    Eigen::Quaterniond ee_orientation_ref = ee_orientation_trajectory_.orientation(t);
    Eigen::Isometry3d ee_transform_ref;
    ee_transform_ref.linear() = ee_orientation_ref.toRotationMatrix();
    ee_transform_ref.translation() = ee_position_ref;

    // Compute the actual transform
    Eigen::Isometry3d ee_transform = tree.relativeTransform(cache, world_frame, task_frame_index_);

    // The rotation error term
    Eigen::Isometry3d transform_error = ee_transform.inverse() * ee_transform_ref;
    Eigen::Vector3d log_rotation_error = logSO3(transform_error.linear());

    // The twist feedback
    using TwistVector = drake::TwistVector<double>;
    TwistVector twist_pd;
    twist_pd.head(3) = kp_rotation_.array() * log_rotation_error.array();
    twist_pd.tail(3) = kp_translation_.array() * transform_error.translation().array();

    // The twist jacobian
    ee_twist_jacobian_expressed_in_ee = tree.geometricJacobian(
        cache, world_frame, task_frame_index_, task_frame_index_);

    // Compute sudo-inverse
    auto svd = ee_twist_jacobian_expressed_in_ee.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    svd.setThreshold(0.01);
    Eigen::VectorXd qdot_command = svd.solve(twist_pd);

    // Write to the result
    command.set_invalid();
    for(auto i = 0; i < qdot_command.size(); i++) {
        command.joint_velocities[i] = qdot_command[i];
        command.joint_position[i] =
                input.latest_measurement->joint_position[i] + qdot_command[i] * input.control_interval_second;
    }
    command.velocity_validity = true;
    command.position_validity = true;
}


Eigen::Vector3d arm_runner::EETrajectoryPlan::logSO3(const Eigen::Matrix3d& rotation) {
    Eigen::AngleAxisd angle_axis(rotation);
    return angle_axis.angle() * angle_axis.axis();
}


void arm_runner::EETrajectoryPlan::InitializePlan(const arm_runner::CommandInput &input) {
    // Get data
    const auto& tree = *input.robot_rbt;
    const auto& cache = *input.measured_state_cache;
    task_frame_index_ = getBodyOrFrameIndex(tree, task_frame_name_);

    // Compute the initial configuration
    Eigen::Isometry3d ee_transform = tree.relativeTransform(cache, world_frame, task_frame_index_);
    ee_xyz_knots_[0] = ee_transform.translation();
    ee_quat_knots_[0] = Eigen::Quaterniond(ee_transform.linear());

    // Compute the trajectory
    Eigen::MatrixXd knot_dot = Eigen::MatrixXd::Zero(3, 1);
    ee_xyz_trajectory_ = PiecewisePolynomial::Cubic(input_time_, ee_xyz_knots_, knot_dot, knot_dot);
    ee_orientation_trajectory_ = PiecewiseQuaternionSlerp(input_time_, ee_quat_knots_);

    // Set the flag
    RobotPlanBase::InitializePlan(input);
}


int arm_runner::EETrajectoryPlan::getBodyOrFrameIndex(
    const RigidBodyTree<double> &tree,
    const std::string &body_or_frame_name
) {
    int body_frame_index = 0;

    // First try the body
    try {
        body_frame_index = tree.FindBodyIndex(body_or_frame_name);
    } catch (const std::logic_error& e) {
        // Then try the frame
        auto frame = tree.findFrame(body_or_frame_name);
        body_frame_index = frame->get_frame_index();
    }

    // OK
    return body_frame_index;
}