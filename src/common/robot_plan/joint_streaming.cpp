//
// Created by wei on 9/6/19.
//

#include "common/robot_plan/joint_streaming.h"


arm_runner::JointStreamingPlanBase::JointStreamingPlanBase(
    std::map<std::string, int> name_to_index,
    ros::NodeHandle nh,
    std::string topic
) : joint_name_to_index_(std::move(name_to_index)),
    node_handle_(nh),
    topic_(std::move(topic))
{
    // Do NOT start streaming at construction
    streaming_subscriber_ = nullptr;
}


void arm_runner::JointStreamingPlanBase::InitializePlan(const arm_runner::CommandInput &input) {
    // The startup of subscriber
    streaming_subscriber_ = std::make_shared<ros::Subscriber>(
        node_handle_.subscribe(
            topic_,
            1,
            &JointStreamingPlanBase::updateStreamedCommand, this));

    // Init the other members
    RobotPlanBase::InitializePlan(input);
}


arm_runner::LoadParameterStatus arm_runner::JointStreamingPlanBase::LoadParameterFrom(const YAML::Node &datamap) {
    auto key = DefaultClassParameterNameKey();
    topic_ = datamap[key]["topic"].as<std::string>();
}


void arm_runner::JointStreamingPlanBase::SaveParameterTo(YAML::Node &datamap) const {
    auto key = DefaultClassParameterNameKey();
    datamap[key]["topic"] = topic_;
}