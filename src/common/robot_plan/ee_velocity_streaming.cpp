//
// Created by wei on 9/7/19.
//

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

}


void arm_runner::EEVelocityStreamingPlan::updateStreamedCommand(const robot_msgs::EEVelocityGoal::ConstPtr &message) {
    std::lock_guard<std::mutex> guard(mutex_);
    ee_frame_id_ = message->ee_frame_id;

    // The linear and angular velocity
    ee_linear_velocity_[0] = message->linear_velocity.x;
    ee_linear_velocity_[1] = message->linear_velocity.y;
    ee_linear_velocity_[2] = message->linear_velocity.z;

    ee_angular_velocity_[0] = message->angular_velocity.x;
    ee_angular_velocity_[1] = message->angular_velocity.y;
    ee_angular_velocity_[2] = message->angular_velocity.z;

    // The flag
    if(!command_valid_)
        command_valid_ = true;
}
