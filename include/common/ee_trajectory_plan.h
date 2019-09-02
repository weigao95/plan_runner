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


namespace arm_runner {


    class EETrajectoryPlan : public RobotPlanBase {
    public:
        using Ptr = std::shared_ptr<EETrajectoryPlan>;
        using PiecewisePolynomial = drake::trajectories::PiecewisePolynomial<double>;
        using PiecewiseQuaternionSlerp = drake::trajectories::PiecewiseQuaternionSlerp<double>;
        EETrajectoryPlan(
            bool has_quaternion,
            std::string task_frame,
            std::vector<double> input_time,
            std::vector<Eigen::MatrixXd> ee_xyz_knots,
            std::vector<Eigen::Quaterniond> ee_quat_knots,
            std::string wrt_frame);
        ~EETrajectoryPlan() override = default;

        // The initialization
        void InitializePlan(const CommandInput& input) override;

        // Getters
        PlanType GetPlanType() const override { return PlanType::EETrajectory; }
        bool HasFinished(const RobotArmMeasurement& measurement) const override {
            return measurement.time_stamp.since_plan_start_second >= TrajectoryDuration();
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
    protected:
        void computeCommand(
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
        static const int world_frame;
        int task_frame_index_;
        PiecewisePolynomial ee_xyz_trajectory_;
        PiecewiseQuaternionSlerp ee_orientation_trajectory_;
        Eigen::Vector3d kp_rotation_;
        Eigen::Vector3d kp_translation_;

        // The cache
        Eigen::MatrixXd ee_twist_jacobian_expressed_in_ee;

    public:
        static std::shared_ptr<EETrajectoryPlan> ConstructFromMessage(
            const RigidBodyTree<double>& tree,
            const robot_msgs::CartesianTrajectoryGoal::ConstPtr &goal);
    };
}
