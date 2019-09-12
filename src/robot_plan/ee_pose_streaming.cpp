//
// Created by wei on 9/12/19.
//

#include "common/rbt_utils.h"
#include "robot_plan/ee_pose_streaming.h"


plan_runner::EEPoseStreamingPlan::EEPoseStreamingPlan(
    ros::NodeHandle &nh,
    std::string topic
) : node_handle_(nh),
    topic_(std::move(topic)),
    command_valid_(false),
    streaming_subscriber_(nullptr)
{
    // The hyper-parameters
    kp_rotation_.setConstant(5);
    kp_translation_.setConstant(10);
}


void plan_runner::EEPoseStreamingPlan::InitializePlan(const plan_runner::CommandInput &input) {
    // The startup of subscriber
    streaming_subscriber_ = std::make_shared<ros::Subscriber>(
        node_handle_.subscribe(
            topic_,
            1,
            &EEPoseStreamingPlan::updateStreamedCommand, this));

    // Basic cases
    RobotPlanBase::InitializePlan(input);
}


void plan_runner::EEPoseStreamingPlan::StopPlan(plan_runner::ActionToCurrentPlan action) {
    streaming_subscriber_->shutdown();
    RobotPlanBase::StopPlan(action);
}


void plan_runner::EEPoseStreamingPlan::ComputeCommand(
    const plan_runner::CommandInput &input,
    plan_runner::RobotArmCommand &command
) {
    // Get the data
    mutex_.lock();
    Eigen::Isometry3d target_pose_cache = target_frame_;
    ee_frame_id_cache = ee_frame_id_;
    expressed_in_frame_cache = target_expressed_in_frame_;
    bool command_valid = command_valid_;
    mutex_.unlock();

    // Get tree
    const auto& tree = *input.robot_rbt;
    const auto& cache = *input.measured_state_cache;
    auto num_joints = tree.get_num_positions();

    // The integration option
    const auto* q_fwd = integrate_option_.GetForwardIntegrationJointPosition(input);
    DRAKE_ASSERT(q_fwd != nullptr);

    // The common flag
    command.set_invalid();
    command.position_validity = true;
    command.velocity_validity = true;
    command.time_stamp = input.latest_measurement->time_stamp;

    // Just return if not valid
    if(!command_valid
    || !bodyOrFrameContainedInTree(tree, ee_frame_id_cache)
    || !bodyOrFrameContainedInTree(tree, expressed_in_frame_cache)) {
        for(auto i = 0; i < num_joints; i++) {
            command.joint_position[i] = q_fwd[i];
            command.joint_velocities[i] = 0;
        }
        return;
    }

    // Now the command is valid
    auto ee_frame_index = getBodyOrFrameIndex(tree, ee_frame_id_cache);
    auto expressed_in_frame_index = getBodyOrFrameIndex(tree, expressed_in_frame_cache);
    auto world_frame = RigidBodyTreeConstants::kWorldBodyIndex;

    // Compute the target pose
    Eigen::Isometry3d target_pose_in_world =
        tree.relativeTransform(cache, world_frame, expressed_in_frame_index) * target_pose_cache;

    // Get transformation
    Eigen::Isometry3d ee_to_world = tree.relativeTransform(cache, world_frame, ee_frame_index);
    Eigen::Isometry3d world_to_ee = ee_to_world.inverse();

    // The rotation error term
    Eigen::Isometry3d transform_error = world_to_ee * target_pose_in_world;
    Eigen::Vector3d log_rotation_error = logSO3(transform_error.linear());

    // The twist feedback
    using TwistVector = drake::TwistVector<double>;
    TwistVector twist_pd;
    twist_pd.head(3) = kp_rotation_.array() * log_rotation_error.array();
    twist_pd.tail(3) = kp_translation_.array() * transform_error.translation().array();

    // The twist jacobian
    ee_twist_jacobian_expressed_in_ee = tree.geometricJacobian(
        cache, world_frame, ee_frame_index, ee_frame_index);

    // Compute sudo-inverse
    TwistVector desired_twist = twist_pd;
    auto svd = ee_twist_jacobian_expressed_in_ee.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    svd.setThreshold(0.01);
    Eigen::VectorXd qdot_command = svd.solve(desired_twist);

    // Write to the result
    command.set_invalid();
    for(auto i = 0; i < qdot_command.size(); i++) {
        command.joint_velocities[i] = qdot_command[i];
        command.joint_position[i] = q_fwd[i] + qdot_command[i] * input.control_interval_second;
    }
}


void plan_runner::EEPoseStreamingPlan::updateStreamedCommand(
    const robot_msgs::CartesianGoalPoint::ConstPtr &message
) {
    std::lock_guard<std::mutex> guard(mutex_);
    ee_frame_id_ = message->ee_frame_id;
    
    // The target position
    const auto& point_msg = message->xyz_point;
    target_expressed_in_frame_ = point_msg.header.frame_id;
    if(target_expressed_in_frame_ == "base")
        target_expressed_in_frame_ = "world";
    Eigen::Vector3d target_position(point_msg.point.x, point_msg.point.y, point_msg.point.z);
    
    // The target orientation
    const auto& orientation_msg = message->quaternion;
    Eigen::Quaterniond target_rotation(
        orientation_msg.w,
        orientation_msg.x,
        orientation_msg.y,
        orientation_msg.z);
    target_frame_.translation() = target_position;
    target_frame_.linear() = target_rotation.toRotationMatrix();
    
    // The flag
    if(!command_valid_)
        command_valid_ = true;
}


// The processing of parameter
plan_runner::LoadParameterStatus plan_runner::EEPoseStreamingPlan::LoadParameterFrom(const YAML::Node &datamap) {
    // The base class
    auto base_load_result = integrate_option_.LoadParameterFrom(datamap);

    // Check the key
    auto key = DefaultClassParameterNameKey();
    if(!datamap[key]) {
        // Keep current value
        return TheWorseStatus(base_load_result, LoadParameterStatus::NonFatalError);
    }

    // Load it
    if(datamap[key]["rotation"]) {
        kp_rotation_[0] = datamap[key]["rotation"][0].as<double>();
        kp_rotation_[1] = datamap[key]["rotation"][1].as<double>();
        kp_rotation_[2] = datamap[key]["rotation"][2].as<double>();
    }

    if(datamap[key]["translation"]) {
        kp_translation_[0] = datamap[key]["translation"][0].as<double>();
        kp_translation_[1] = datamap[key]["translation"][1].as<double>();
        kp_translation_[2] = datamap[key]["translation"][2].as<double>();
    }
    return TheWorseStatus(base_load_result, LoadParameterStatus::Success);
}

void plan_runner::EEPoseStreamingPlan::SaveParameterTo(YAML::Node &datamap) const {
    // The base class
    integrate_option_.SaveParameterTo(datamap);

    // The rotation gain
    auto key = DefaultClassParameterNameKey();
    datamap[key]["rotation"].push_back(kp_rotation_[0]);
    datamap[key]["rotation"].push_back(kp_rotation_[1]);
    datamap[key]["rotation"].push_back(kp_rotation_[2]);

    // The translation gain
    datamap[key]["translation"].push_back(kp_translation_[0]);
    datamap[key]["translation"].push_back(kp_translation_[1]);
    datamap[key]["translation"].push_back(kp_translation_[2]);
}