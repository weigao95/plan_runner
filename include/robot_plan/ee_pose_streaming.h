//
// Created by wei on 9/12/19.
//

#pragma once

#include "robot_msgs/CartesianGoalPoint.h"

#include "common/plan_base.h"
#include "robot_plan/position_integrate_option.h"


namespace plan_runner {
    
    class EEPoseStreamingPlan : public RobotPlanBase {
    public:
        using Ptr = std::shared_ptr<EEPoseStreamingPlan>;
        EEPoseStreamingPlan(ros::NodeHandle& nh, std::string topic);
        ~EEPoseStreamingPlan() override = default;

        // The interface
        PlanType GetPlanType() const final { return PlanType::EEConfigurationStreaming; }
        bool HasFinished(const RobotArmMeasurement& measurement) const override { return false; }
        void InitializePlan(const CommandInput& input) override;
        void StopPlan(ActionToCurrentPlan action) override;
        void ComputeCommand(const CommandInput& input, RobotArmCommand& command) override;

        // The information about the plan
    private:
        std::mutex mutex_;
        std::string ee_frame_id_;
        std::string target_expressed_in_frame_;
        Eigen::Isometry3d command_frame_desired_pose_;
        Eigen::Isometry3d command_frame_in_ee_;
        bool command_valid_;

        // The ROS handler
    public:
        void updateStreamedCommand(const robot_msgs::CartesianGoalPoint::ConstPtr& message);
    private:
        // The ros info
        std::string topic_;
        ros::NodeHandle node_handle_;
        std::shared_ptr<ros::Subscriber> streaming_subscriber_;

        // The hyper-parameter
    private:
        PositionIntegratorOption integrate_option_;
        Eigen::Vector3d kp_rotation_;
        Eigen::Vector3d kp_translation_;
    public:
        LoadParameterStatus LoadParameterFrom(const YAML::Node& datamap) override;
        void SaveParameterTo(YAML::Node& datamap) const override;
        std::string DefaultClassParameterNameKey() const override { return "EEPoseStreamingHyperparameter"; }

        // The caches
        // Only on main thread, assign before use
    private:
        std::string ee_frame_id_cache;
        std::string expressed_in_frame_cache;
        Eigen::MatrixXd ee_twist_jacobian_expressed_in_ee;
    };
}