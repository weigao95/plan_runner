//
// Created by wei on 11/11/19.
//

#include "robot_plan/ee_force_estimator.h"
#include "common/rbt_utils.h"
#include <robot_msgs/ForceTorque.h>


plan_runner::EEForceTorqueEstimator::EEForceTorqueEstimator(
    ros::NodeHandle &nh,
    std::unique_ptr<RigidBodyTree<double>> tree,
    std::string estimation_publish_topic,
    std::string reinit_service_name,
    std::string joint_state_topic,
    std::string ee_frame_id
) : node_handle_(nh),
    tree_(std::move(tree)),
    estimation_publish_topic_(std::move(estimation_publish_topic)),
    joint_state_topic_(std::move(joint_state_topic)),
    reinit_offset_srv_name_(std::move(reinit_service_name)),
    ee_frame_id_(std::move(ee_frame_id)),
    offset_valid_(false)
{
    estimation_publisher_ = node_handle_.advertise<robot_msgs::ForceTorque>(estimation_publish_topic_, 1);
    reinit_offset_server_ = std::make_shared<ros::ServiceServer>(
        node_handle_.advertiseService(reinit_offset_srv_name_,
                                  &EEForceTorqueEstimator::onReinitServiceRequeat, this));
}


plan_runner::EEForceTorqueEstimator::~EEForceTorqueEstimator() {
    if(joint_state_subscriber_ != nullptr)
        joint_state_subscriber_->shutdown();
    if(reinit_offset_server_ != nullptr)
        reinit_offset_server_->shutdown();
}


void plan_runner::EEForceTorqueEstimator::Initialize() {
    // The startup of subscriber
    joint_state_subscriber_ = std::make_shared<ros::Subscriber>(
        node_handle_.subscribe(
            joint_state_topic_,
            1,
            &EEForceTorqueEstimator::onReceiveJointState, this));
    std::cout <<  "The force/torque estimator initialization OK" << std::endl;
}


void plan_runner::EEForceTorqueEstimator::onReceiveJointState(const sensor_msgs::JointState::ConstPtr& joint_state) {
    if(joint_state->position.size() <= 2 || joint_state->effort.size() <= 2)
        return;

    // Compute force and torque
    Eigen::Vector3d force_in_world, torque_in_world;
    estimateEEForceTorque(joint_state, force_in_world, torque_in_world);

    // Make msg and publish it
    robot_msgs::ForceTorque ft_msg;
    ft_msg.frame_id = ee_frame_id_;
    ft_msg.force.x = force_in_world[0];
    ft_msg.force.y = force_in_world[1];
    ft_msg.force.z = force_in_world[2];

    ft_msg.torque.x = torque_in_world[0];
    ft_msg.torque.y = torque_in_world[1];
    ft_msg.torque.z = torque_in_world[2];

    // Send the msg
    estimation_publisher_.publish(ft_msg);
}


bool plan_runner::EEForceTorqueEstimator::onReinitServiceRequeat(
    std_srvs::Trigger::Request &req,
    std_srvs::Trigger::Response &res
) {
    std::lock_guard<std::mutex> guard(mutex_);
    offset_valid_ = false;
    res.success = true;
    std::cout << "Reinit the joint torque offset" << std::endl;
    return true;
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
    Eigen::Map<const Eigen::VectorXd> raw_joint_torque = Eigen::Map<const Eigen::VectorXd>(joint_state->effort.data(), q_size);

    // The offset
    mutex_.lock();
    if(!offset_valid_) {
        torque_offset_.resize(q_size);
        torque_offset_ = raw_joint_torque;
        offset_valid_ = true;
    }
    Eigen::VectorXd joint_torque = raw_joint_torque - torque_offset_;
    mutex_.unlock();

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

    // Compute force torque using QP
    // min: ||(torque, force)||^2
    // s.t: J_T dot (torque, force) = joint_torque
    // The KKT condition is
    // [I_6x6 J    ] [ x     ] = 0
    // [J_T   0_7x7] [ lambda] = joint_torque
    /*Eigen::MatrixXd K; K.resize(6 + n_cols, 6 + n_cols); K.setZero();
    for(auto i = 0; i < 6; i++)
        K(i, i) = 1;
    K.block(0, 6, 6, 7) = concantated_jacobian;
    K.block(6, 6, 7, 6) = concantated_jacobian.transpose();

    Eigen::VectorXd rhs; rhs.resize(6 + n_cols); rhs.setZero();
    for(auto i = 0; i < n_cols; i++)
        rhs[i + 6] = joint_torque[i];

    // Solve it
    auto svd = K.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    svd.setThreshold(0.01);
    Eigen::VectorXd torque_force_lambda = svd.solve(rhs);*/
    
    // The old solver
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
