//
// Created by wei on 9/7/19.
//

#include "common/rbt_utils.h"
#include "common/robot_plan/ee_velocity_streaming.h"


arm_runner::EEVelocityStreamingPlan::EEVelocityStreamingPlan(
    ros::NodeHandle &nh,
    std::string topic
) : node_handle_(nh),
    topic_(std::move(topic)),
    command_valid_(false),
    streaming_subscriber_(nullptr)
{ }


void arm_runner::EEVelocityStreamingPlan::InitializePlan(const arm_runner::CommandInput &input) {
    // The startup of subscriber
    streaming_subscriber_ = std::make_shared<ros::Subscriber>(
        node_handle_.subscribe(
            topic_,
            1,
            &EEVelocityStreamingPlan::updateStreamedCommand, this));

    // Basic cases
    RobotPlanBase::InitializePlan(input);
}


void arm_runner::EEVelocityStreamingPlan::StopPlan(arm_runner::ActionToCurrentPlan action) {
    streaming_subscriber_->shutdown();
    RobotPlanBase::StopPlan(action);
}


void arm_runner::EEVelocityStreamingPlan::ComputeCommand(
        const arm_runner::CommandInput &input,
        arm_runner::RobotArmCommand &command
) {
    // Collect info
    mutex_.lock();
    cmd_frame_linear_velocity_cache = cmd_frame_linear_velocity_;
    cmd_frame_angular_velocity_cache = cmd_frame_angular_velocity_;
    ee_frame_id_cache = ee_frame_id_;
    command_frame_to_ee_cache = command_frame_to_ee_;
    bool command_valid = command_valid_;
    mutex_.unlock();

    // Get tree
    const auto& tree = *input.robot_rbt;
    const auto& cache = *input.measured_state_cache;
    auto num_joints = tree.get_num_positions();

    // Command fwd
    const double* q_fwd = nullptr;
    const auto& history = input.robot_history->GetCommandHistory();
    if(history.empty())
        q_fwd = input.latest_measurement->joint_position;
    else
        q_fwd = history.back().joint_position;
    DRAKE_ASSERT(q_fwd != nullptr);

    // The common flag
    command.set_invalid();
    command.position_validity = true;
    command.velocity_validity = true;
    command.time_stamp = input.latest_measurement->time_stamp;

    // Just return if not valid
    if(!command_valid || !bodyOrFrameContainedInTree(tree, ee_frame_id_cache)) {
        for(auto i = 0; i < num_joints; i++) {
            command.joint_position[i] = q_fwd[i];
            command.joint_velocities[i] = 0;
        }
        return;
    }

    // Now the command is valid
    auto ee_frame_index = getBodyOrFrameIndex(tree, ee_frame_id_cache);
    auto world_frame = RigidBodyTreeConstants::kWorldBodyIndex;

    // Compute the actual transform ee_to_world
    Eigen::Isometry3d ee_to_world = tree.relativeTransform(cache, world_frame, ee_frame_index);
    Eigen::Isometry3d world_to_ee = ee_to_world.inverse();

    // The relative position
    Eigen::Vector3d cmd2ee_in_ee = command_frame_to_ee_cache.translation();
    Eigen::Vector3d cmd2ee_in_world = ee_to_world.rotation() * cmd2ee_in_ee;

    // The velocity information of ee
    Eigen::Vector3d ee_angular_velocity = cmd_frame_angular_velocity_cache;
    Eigen::Vector3d ee_linear_velocity = cmd_frame_linear_velocity_cache + ee_angular_velocity.cross(cmd2ee_in_world);

    // Expressed in ee frame
    Eigen::Vector3d ee_linear_v_in_ee = world_to_ee.rotation() * ee_linear_velocity;
    Eigen::Vector3d ee_angular_v_in_ee = world_to_ee.rotation() * ee_angular_velocity;
    using TwistVector = drake::TwistVector<double>;
    TwistVector twist_fwd;
    twist_fwd.head(3) = ee_angular_v_in_ee;
    twist_fwd.tail(3) = ee_linear_v_in_ee;

    // The twist jacobian
    ee_twist_jacobian_expressed_in_ee = tree.geometricJacobian(
        cache, world_frame, ee_frame_index, ee_frame_index);

    // Compute sudo-inverse
    TwistVector desired_twist = twist_fwd;
    auto svd = ee_twist_jacobian_expressed_in_ee.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    svd.setThreshold(0.01);
    Eigen::VectorXd qdot_command = svd.solve(desired_twist);

    // Write to the result
    for(auto i = 0; i < qdot_command.size(); i++) {
        command.joint_velocities[i] = qdot_command[i];
        command.joint_position[i] = q_fwd[i] + qdot_command[i] * input.control_interval_second;
    }
}


void arm_runner::EEVelocityStreamingPlan::updateStreamedCommand(const robot_msgs::EEVelocityGoal::ConstPtr &message) {
    std::lock_guard<std::mutex> guard(mutex_);
    ee_frame_id_ = message->ee_frame_id;

    // The linear and angular velocity
    cmd_frame_linear_velocity_[0] = message->linear_velocity.x;
    cmd_frame_linear_velocity_[1] = message->linear_velocity.y;
    cmd_frame_linear_velocity_[2] = message->linear_velocity.z;

    cmd_frame_angular_velocity_[0] = message->angular_velocity.x;
    cmd_frame_angular_velocity_[1] = message->angular_velocity.y;
    cmd_frame_angular_velocity_[2] = message->angular_velocity.z;

    // The command frame w.r.t ee
    const auto& cmd_to_ee_orientation_msg = message->command_frame_to_ee.orientation;
    const auto& cmd_to_ee_position_msg = message->command_frame_to_ee.position;
    Eigen::Quaterniond cmd_to_ee_rotation(
        cmd_to_ee_orientation_msg.w,
        cmd_to_ee_orientation_msg.x,
        cmd_to_ee_orientation_msg.y,
        cmd_to_ee_orientation_msg.z);
    Eigen::Vector3d cmd_to_ee_position(cmd_to_ee_position_msg.x, cmd_to_ee_position_msg.y, cmd_to_ee_position_msg.z);
    command_frame_to_ee_.linear() = cmd_to_ee_rotation.toRotationMatrix();
    command_frame_to_ee_.translation() = cmd_to_ee_position;

    // The flag
    if(!command_valid_)
        command_valid_ = true;
}
