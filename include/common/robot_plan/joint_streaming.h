//
// Created by wei on 9/6/19.
//

#pragma once

#include <map>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "common/plan_base.h"

namespace arm_runner {


    // The base of all joint-space streaming plan
    class JointStreamingPlanBase : public RobotPlanBase {
    public:
        using Ptr = std::shared_ptr<JointStreamingPlanBase>;
        JointStreamingPlanBase(
            std::map<std::string, int> name_to_index,
            ros::NodeHandle nh,
            std::string topic="");
        ~JointStreamingPlanBase() override = default;

        // The finished interface
        bool HasFinished(const RobotArmMeasurement& measurement) const override { return false; }
        void InitializePlan(const CommandInput& input) override;

        // The joint name to index of the plan
    public:
        virtual void updateStreamedCommand(const sensor_msgs::JointState::ConstPtr& message) = 0;
    private:
        std::string topic_;
        ros::NodeHandle node_handle_;
        std::shared_ptr<ros::Subscriber> streaming_subscriber_;
        std::map<std::string, int> joint_name_to_index_;

        // All joint streaming plans only have topic as parameter
    public:
        LoadParameterStatus LoadParameterFrom(const YAML::Node& datamap) override;
        void SaveParameterTo(YAML::Node& datamap) const override;
        std::string DefaultClassParameterNameKey() const override { return "JointStreamingPlanBaseTopic"; }
    };

}