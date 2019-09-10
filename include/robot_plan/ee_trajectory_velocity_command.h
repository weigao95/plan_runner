//
// Created by wei on 8/30/19.
//

#pragma once

#include <memory>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/common/trajectories/piecewise_quaternion.h>
#include <drake/multibody/rigid_body_tree.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <ros/ros.h>
#include <robot_msgs/CartesianTrajectoryAction.h>

#include "common/plan_base.h"
#include "robot_plan/position_integrate_option.h"


namespace plan_runner {


    class EETrajectoryVelocityCommandPlan : public RobotPlanBase {
    public:
        using Ptr = std::shared_ptr<EETrajectoryVelocityCommandPlan>;
        using PiecewisePolynomial = drake::trajectories::PiecewisePolynomial<double>;
        using PiecewiseQuaternionSlerp = drake::trajectories::PiecewiseQuaternionSlerp<double>;
        EETrajectoryVelocityCommandPlan(
            bool has_quaternion,
            std::string task_frame,
            std::vector<double> input_time,
            std::vector<Eigen::MatrixXd> ee_xyz_knots,
            std::vector<Eigen::Quaterniond> ee_quat_knots,
            std::string wrt_frame);
        ~EETrajectoryVelocityCommandPlan() override = default;

        // Construct from message
        static std::shared_ptr<EETrajectoryVelocityCommandPlan> ConstructFromMessage(
            const RigidBodyTree<double>& tree,
            const robot_msgs::CartesianTrajectoryGoal::ConstPtr &goal);

        // The initialization
        void InitializePlan(const CommandInput& input) override;

        // Getters
        PlanType GetPlanType() const override { return PlanType::EETrajectory; }
        bool HasFinished(const RobotArmMeasurement& measurement) const override {
            constexpr double TRACKING_CONVERGE_FACTOR = 1.1;
            return GetTimeSincePlanStartSecond(measurement.time_stamp) >= TRACKING_CONVERGE_FACTOR * TrajectoryDuration();
        }

        // The total time for the trajectory
        double TrajectoryDuration() const {
            if (ee_xyz_trajectory_.get_number_of_segments() > 0) {
                return ee_xyz_trajectory_.end_time() - ee_xyz_trajectory_.start_time();
            } else {
                return 0.;
            }
        }


        // The actual computation
        void ComputeCommand(
            const CommandInput& input,
            RobotArmCommand& command) override;
        static Eigen::Vector3d logSO3(const Eigen::Matrix3d& rotation);


        // The member used for construction
    private:
        bool has_quaternion_;
        std::string task_frame_name_;
        std::string wrt_frame_name_;
        std::vector<double> input_time_;
        std::vector<Eigen::MatrixXd> ee_xyz_knots_;
        std::vector<Eigen::Quaterniond> ee_quat_knots_;

        // The constructed members
    private:
        int task_frame_index_;
        PiecewisePolynomial ee_xyz_trajectory_;
        PiecewisePolynomial ee_xyz_velocity_trajectory_;
        PiecewiseQuaternionSlerp ee_orientation_trajectory_;

        // The cache
        Eigen::MatrixXd ee_twist_jacobian_expressed_in_ee;

        // The hypter-parameter
    private:
        PositionIntegratorOption integrate_option_;
        Eigen::Vector3d kp_rotation_;
        Eigen::Vector3d kp_translation_;
    public:
        LoadParameterStatus LoadParameterFrom(const YAML::Node& datamap) override;
        void SaveParameterTo(YAML::Node& datamap) const override;
        std::string DefaultClassParameterNameKey() const override { return "EETrajectoryPlanHyperparameter"; }
    };
}
