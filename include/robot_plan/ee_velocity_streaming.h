//
// Created by wei on 9/7/19.
//

#pragma once

#include <mutex>
#include "common/plan_base.h"
#include "robot_plan/position_integrate_option.h"
#include "robot_msgs/EEVelocityGoal.h"


namespace plan_runner {

    // The plan that receives end-effector velocity and transform it into joint command
    class EEVelocityStreamingPlan : public RobotPlanBase {
    public:
        using Ptr = std::shared_ptr<EEVelocityStreamingPlan>;
        EEVelocityStreamingPlan(ros::NodeHandle& nh, std::string topic);
        ~EEVelocityStreamingPlan() override = default;

        // The interface
        PlanType GetPlanType() const final { return PlanType::EEVelocityStreaming; }
        bool HasFinished(const RobotArmMeasurement& measurement) const override { return false; }
        void InitializePlan(const CommandInput& input) override;
        void StopPlan(ActionToCurrentPlan action) override;
        void ComputeCommand(const CommandInput& input, RobotArmCommand& command) override;

        // The information about the plan
    private:
        std::mutex mutex_;
        std::string ee_frame_id_;
        Eigen::Vector3d cmd_frame_linear_velocity_;
        Eigen::Vector3d cmd_frame_angular_velocity_;
        Eigen::Isometry3d command_frame_to_ee_;
        bool command_valid_;

        // The ROS handler
    public:
        void updateStreamedCommand(const robot_msgs::EEVelocityGoal::ConstPtr& message);
    private:
        // The ros info
        std::string topic_;
        ros::NodeHandle node_handle_;
        std::shared_ptr<ros::Subscriber> streaming_subscriber_;

        // Parameters
    private:
        PositionIntegratorOption integrator_option_;
    public:
        LoadParameterStatus LoadParameterFrom(const YAML::Node& datamap) override {return integrator_option_.LoadParameterFrom(datamap);};
        void SaveParameterTo(YAML::Node& datamap) const override { integrator_option_.SaveParameterTo(datamap); };

        // Caches
    private:
        std::string ee_frame_id_cache;
        Eigen::Vector3d cmd_frame_linear_velocity_cache, cmd_frame_angular_velocity_cache;
        Eigen::Isometry3d command_frame_to_ee_cache;
        Eigen::MatrixXd ee_twist_jacobian_expressed_in_ee;
    };

}