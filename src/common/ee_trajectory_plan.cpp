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
    Eigen::Vector3d ee_xyz_ref = ee_xyz_trajectory_.value(t);
    Eigen::Quaterniond ee_quat_ref = ee_orientation_trajectory_.orientation(t);
    Eigen::Isometry3d ee_transform_ref;
    ee_transform_ref.linear() = ee_quat_ref.toRotationMatrix();
    ee_transform_ref.translation() = ee_xyz_ref;

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
        command.joint_position[i] = qdot_command[i];
    }
    command.position_validity = true;
}


Eigen::Vector3d arm_runner::EETrajectoryPlan::logSO3(const Eigen::Matrix3d& rotation) {
    Eigen::AngleAxisd angle_axis(rotation);
    return angle_axis.angle() * angle_axis.axis();
}