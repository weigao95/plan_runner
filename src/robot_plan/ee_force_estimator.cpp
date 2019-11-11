//
// Created by wei on 11/11/19.
//

#include "robot_plan/ee_force_estimator.h"
#include "common/rbt_utils.h"


plan_runner::EEForceTorqueEstimator::EEForceTorqueEstimator(
    ros::NodeHandle &nh,
    std::unique_ptr<RigidBodyTree<double>> tree,
    std::string estimation_publish_topic,
    std::string joint_state_topic,
    std::string ee_frame_id
) : node_handle_(nh),
    tree_(std::move(tree)),
    estimation_publish_topic_(std::move(estimation_publish_topic)),
    joint_state_topic_(std::move(joint_state_topic)),
    ee_frame_id_(std::move(ee_frame_id))
{

}

static Eigen::MatrixXd AngularVelocityJacobian(
    const RigidBodyTreed& tree,
    const KinematicsCache<double>& cache, int ee_frame_id) {
    auto world_frame = RigidBodyTreeConstants::kWorldBodyIndex;
    Eigen::MatrixXd twist_jacobian = tree.geometricJacobian(cache, world_frame, ee_frame_id, world_frame);
    auto n_cols = twist_jacobian.cols();
    Eigen::MatrixXd angular_velocity_jacobian = twist_jacobian.block(0, 0, 3, n_cols);
    return angular_velocity_jacobian;
}

void plan_runner::EEForceTorqueEstimator::estimateEEForceTorque(
    const sensor_msgs::JointState::ConstPtr &joint_state,
    Eigen::Vector3d &force_in_world,
    Eigen::Vector3d &torque_in_world
) {
    // Get the data
    auto q_size = tree_->get_num_positions();
    Eigen::Map<const Eigen::VectorXd> q = Eigen::Map<const Eigen::VectorXd>(joint_state->position.data(), q_size);
    Eigen::Map<const Eigen::VectorXd> joint_torque = Eigen::Map<const Eigen::VectorXd>(joint_state->effort.data(), q_size);

    // Kinematic computation
    KinematicsCache<double> cache = tree_->CreateKinematicsCache();
    cache.initialize(q);
    tree_->doKinematics(cache);

    // Get the jacobian
    auto ee_frame_index = getBodyOrFrameIndex(*tree_, ee_frame_id_);
    auto world_frame = RigidBodyTreeConstants::kWorldBodyIndex;

    // Compute jacobian
    Eigen::MatrixXd angular_velocity_jacobian = AngularVelocityJacobian(*tree_, cache, ee_frame_index);
    Eigen::MatrixXd linear_velocity_jacobian = tree_->transformPointsJacobian(
        cache, Eigen::Vector3d::Zero(), ee_frame_index, world_frame, true);
    auto n_cols = linear_velocity_jacobian.cols();
    Eigen::MatrixXd concantated_jacobian;
    concantated_jacobian.resize(6, n_cols);
    concantated_jacobian.block(0, 0, 3, n_cols) = angular_velocity_jacobian;
    concantated_jacobian.block(3, 0, 3, n_cols) = linear_velocity_jacobian;

    // Compute force torque
    // J_T dot (torque, force) = joint_torque
    Eigen::MatrixXd J_T = concantated_jacobian.transpose();
    auto svd = J_T.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    svd.setThreshold(0.01);
    Eigen::VectorXd torque_force = svd.solve(joint_torque);

    // Save the result
    for(auto i = 0; i < 3; i++) {
        torque_in_world[i] = torque_force[i];
        force_in_world[i] = torque_force[i + 3];
    }
}
