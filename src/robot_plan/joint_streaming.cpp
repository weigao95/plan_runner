//
// Created by wei on 9/6/19.
//

#include "robot_plan/joint_streaming.h"


// The method in command base
plan_runner::JointStreamingPlanBase::JointStreamingPlanBase(
    ros::NodeHandle& nh,
    std::string topic
) : node_handle_(nh),
    topic_(std::move(topic))
{
    // Do NOT start streaming at construction
    streaming_subscriber_ = nullptr;
    num_joints_ = 0;
}


void plan_runner::JointStreamingPlanBase::InitializePlan(const plan_runner::CommandInput &input) {
    // Get information from the tree
    const auto& tree = *input.robot_rbt;
    num_joints_ = tree.get_num_positions();
    joint_name_to_index_ = tree.computePositionNameToIndexMap();

    // The startup of subscriber
    streaming_subscriber_ = std::make_shared<ros::Subscriber>(
        node_handle_.subscribe(
            topic_,
            1,
            &JointStreamingPlanBase::updateStreamedCommand, this));

    // Init the other members
    RobotPlanBase::InitializePlan(input);
}


void plan_runner::JointStreamingPlanBase::StopPlan(plan_runner::ActionToCurrentPlan action) {
    streaming_subscriber_->shutdown();
    RobotPlanBase::StopPlan(action);
}


plan_runner::LoadParameterStatus plan_runner::JointStreamingPlanBase::LoadParameterFrom(const YAML::Node &datamap) {
    auto key = DefaultClassParameterNameKey();
    if(!datamap[key] || !datamap[key]["topic"] ) {
        if(topic_.empty())
            return LoadParameterStatus::FatalError;
        else
            return LoadParameterStatus::NonFatalError;
    }

    // Load it
    topic_ = datamap[key]["topic"].as<std::string>();
    return LoadParameterStatus::Success;
}


void plan_runner::JointStreamingPlanBase::SaveParameterTo(YAML::Node &datamap) const {
    auto key = DefaultClassParameterNameKey();
    datamap[key]["topic"] = topic_;
}


// The joint position streaming
plan_runner::JointPositionStreamingPlan::JointPositionStreamingPlan(
   ros::NodeHandle &nh, std::string topic
) : JointStreamingPlanBase(nh, std::move(topic))
{
    command_valid_flag_ = false;
}


void plan_runner::JointPositionStreamingPlan::updateStreamedCommand(
    const sensor_msgs::JointState::ConstPtr &message
) {
    std::lock_guard<std::mutex> guard(mutex_);
    commanded_position_.resize(num_joints_);
    for (unsigned i = 0; i < message->name.size(); i++) {
        if (joint_name_to_index_.count(message->name[i]) == 0) {
            continue;
        }

        // Copy to joint
        int idx = joint_name_to_index_[message->name[i]];
        if(idx < num_joints_) {
            commanded_position_[idx] = message->position[i];
        }
    }

    // Setup the flag
    if(!command_valid_flag_) {
        command_valid_flag_ = true;
    }
}


void plan_runner::JointPositionStreamingPlan::ComputeCommand(
    const plan_runner::CommandInput &input,
    plan_runner::RobotArmCommand &command
) {
    // Copy the position
    bool command_valid;
    mutex_.lock();
    streamed_command_position_cache = commanded_position_;
    command_valid = command_valid_flag_;
    mutex_.unlock();

    // Send to command
    const double* q_fwd = nullptr;
    const auto& history = input.robot_history->GetCommandHistory();
    if(history.empty())
        q_fwd = input.latest_measurement->joint_position;
    else
        q_fwd = history.back().joint_position;
    DRAKE_ASSERT(q_fwd != nullptr);

    // Setup the command position
    command.set_invalid();
    if(command_valid) {
        for(auto i = 0; i < num_joints_; i++) {
            command.joint_position[i] = streamed_command_position_cache[i];
        }
    } else {
        for(auto i = 0; i < num_joints_; i++) {
            command.joint_position[i] = q_fwd[i];
        }
    }

    // Update flag
    command.position_validity = true;
    command.time_stamp = input.latest_measurement->time_stamp;
}


// The joint velocity streaming
plan_runner::JointVelocityStreamingPlan::JointVelocityStreamingPlan(
    ros::NodeHandle &nh,
    std::string topic
) : JointStreamingPlanBase(nh, std::move(topic))
{
    command_valid_flag_ = false;
}


void plan_runner::JointVelocityStreamingPlan::updateStreamedCommand(
    const sensor_msgs::JointState::ConstPtr &message
) {
    std::lock_guard<std::mutex> guard(mutex_);
    commanded_velocity_.resize(num_joints_);
    for (unsigned i = 0; i < message->name.size(); i++) {
        if (joint_name_to_index_.count(message->name[i]) == 0) {
            continue;
        }

        // Copy to joint
        int idx = joint_name_to_index_[message->name[i]];
        if(idx < num_joints_) {
            commanded_velocity_[idx] = message->velocity[i];
        }
    }

    // Setup the flag
    if(!command_valid_flag_) {
        command_valid_flag_ = true;
    }
}


void plan_runner::JointVelocityStreamingPlan::ComputeCommand(
        const plan_runner::CommandInput &input,
        plan_runner::RobotArmCommand &command
) {
    // Copy the position
    bool command_valid;
    mutex_.lock();
    streamed_command_velocity_cache = commanded_velocity_;
    command_valid = command_valid_flag_;
    mutex_.unlock();

    // Send to command
    const double* q_fwd = nullptr;
    const auto& history = input.robot_history->GetCommandHistory();
    if(history.empty())
        q_fwd = input.latest_measurement->joint_position;
    else
        q_fwd = history.back().joint_position;
    DRAKE_ASSERT(q_fwd != nullptr);

    // Setup the command position
    command.set_invalid();
    if(command_valid) {
        for(auto i = 0; i < num_joints_; i++) {
            command.joint_position[i] =  q_fwd[i] + streamed_command_velocity_cache[i] * input.control_interval_second;
            command.joint_velocities[i] = streamed_command_velocity_cache[i];
        }
    } else {
        for(auto i = 0; i < num_joints_; i++) {
            command.joint_position[i] = q_fwd[i];
            command.joint_velocities[i] = 0;
        }
    }

    // Update flag
    command.position_validity = true;
    command.velocity_validity = true;
    command.time_stamp = input.latest_measurement->time_stamp;
}