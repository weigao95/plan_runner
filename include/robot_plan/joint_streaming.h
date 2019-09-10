//
// Created by wei on 9/6/19.
//

#pragma once

#include <map>
#include <mutex>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "common/plan_base.h"

namespace plan_runner {


    // The base of all joint-space streaming plan
    class JointStreamingPlanBase : public RobotPlanBase {
    public:
        using Ptr = std::shared_ptr<JointStreamingPlanBase>;
        JointStreamingPlanBase(
            ros::NodeHandle& nh,
            std::string topic);
        ~JointStreamingPlanBase() override = default;

        // The finished interface
        bool HasFinished(const RobotArmMeasurement& measurement) const override { return false; }
        void InitializePlan(const CommandInput& input) override;
        void StopPlan(ActionToCurrentPlan action) override;

        // The joint name to index of the plan
    public:
        virtual void updateStreamedCommand(const sensor_msgs::JointState::ConstPtr& message) = 0;
    private:
        // The ros info
        std::string topic_;
        ros::NodeHandle node_handle_;
        std::shared_ptr<ros::Subscriber> streaming_subscriber_;

        // These info comes from tree
    protected:
        int num_joints_;
        std::map<std::string, int> joint_name_to_index_;

        // All joint streaming plans only have topic as parameter
    public:
        LoadParameterStatus LoadParameterFrom(const YAML::Node& datamap) override;
        void SaveParameterTo(YAML::Node& datamap) const override;
        std::string DefaultClassParameterNameKey() const override { return "JointStreamingPlanBaseTopic"; }
    };


    // Joint position streaming
    class JointPositionStreamingPlan : public JointStreamingPlanBase {
    public:
        using Ptr = std::shared_ptr<JointPositionStreamingPlan>;
        JointPositionStreamingPlan(
            ros::NodeHandle& nh,
            std::string topic);
        ~JointPositionStreamingPlan() final = default;

        // The interface
        PlanType GetPlanType() const final { return PlanType::JointPositionStreaming; }
        void updateStreamedCommand(const sensor_msgs::JointState::ConstPtr& message) final;
        void ComputeCommand(
            const CommandInput& input,
            RobotArmCommand& command) final;

    private:
        // The latest streamming position
        std::mutex mutex_;
        Eigen::VectorXd commanded_position_;
        bool command_valid_flag_;
        // The cache
        Eigen::VectorXd streamed_command_position_cache;
    };


    // The velocity streaming plan
    class JointVelocityStreamingPlan : public JointStreamingPlanBase {
    public:
        using Ptr = std::shared_ptr<JointVelocityStreamingPlan>;
        JointVelocityStreamingPlan(ros::NodeHandle& nh, std::string topic);
        ~JointVelocityStreamingPlan() final = default;

        // The interface
        PlanType GetPlanType() const final { return PlanType::JointVelocityStreaming; }
        void updateStreamedCommand(const sensor_msgs::JointState::ConstPtr& message) final;
        void ComputeCommand(
                const CommandInput& input,
                RobotArmCommand& command) final;
    private:
        // The latest streamming velocity
        std::mutex mutex_;
        Eigen::VectorXd commanded_velocity_;
        bool command_valid_flag_;
        // The cache
        Eigen::VectorXd streamed_command_velocity_cache;
    };
}