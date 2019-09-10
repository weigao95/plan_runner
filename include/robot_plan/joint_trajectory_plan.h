//
// Created by wei on 8/28/19.
//

#pragma once

#include <memory>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/multibody/rigid_body_tree.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <ros/ros.h>

#include "common/plan_base.h"
#include "robot_plan/position_integrate_option.h"
#include "robot_msgs/JointTrajectoryAction.h"


namespace plan_runner {

    class JointTrajectoryPlan : public RobotPlanBase {
    public:
        using Ptr = std::shared_ptr<JointTrajectoryPlan>;
        using PiecewisePolynomial = drake::trajectories::PiecewisePolynomial<double>;
        JointTrajectoryPlan(std::vector<double> input_time, std::vector<Eigen::MatrixXd> knots);
        ~JointTrajectoryPlan() override = default;
        static std::shared_ptr<JointTrajectoryPlan> ConstructFromMessage(
            const RigidBodyTree<double>& tree,
            const robot_msgs::JointTrajectoryGoal::ConstPtr &goal);

        // The method in base class
        void InitializePlan(const CommandInput& input) override;
        PlanType GetPlanType() const override { return PlanType::JointTrajectory; }
        bool HasFinished(const RobotArmMeasurement& measurement) const override {
            constexpr double TRAJECTORY_TIME_SCALE_TRACKING_CONVERGE = 1.1;
            return GetTimeSincePlanStartSecond(measurement.time_stamp) >=
                TRAJECTORY_TIME_SCALE_TRACKING_CONVERGE * TrajectoryDuration();
        }

        // The total time for the trajectory
        double TrajectoryDuration() const {
            if (joint_trajectory_.get_number_of_segments() > 0) {
                return joint_trajectory_.end_time() - joint_trajectory_.start_time();
            } else {
                return 0.;
            }
        }

        // The computation interface
        void ComputeCommand(
            const CommandInput& input,
            RobotArmCommand& command) override;

    private:
        // The data used to construct the trajectory
        std::vector<double> input_time_;
        std::vector<Eigen::MatrixXd> knots_;

        // The trajectory
        PiecewisePolynomial joint_trajectory_;
        PiecewisePolynomial joint_velocity_trajectory_;

        // The parameter is only the integrator option
    private:
        PositionIntegratorOption integrator_option_;
    public:
        LoadParameterStatus LoadParameterFrom(const YAML::Node& datamap) override {return integrator_option_.LoadParameterFrom(datamap);};
        void SaveParameterTo(YAML::Node& datamap) const override { integrator_option_.SaveParameterTo(datamap); };

        // Cache
        Eigen::VectorXd q_command_cache, v_command_cache;
    };
}