//
// Created by wei on 8/28/19.
//

#pragma once

#include <memory>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/multibody/rigid_body_tree.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include "common/plan_base.h"
#include "robot_msgs/JointTrajectoryAction.h"


namespace arm_runner {

    class JointTrajectoryPlan : public RobotPlanBase {
    public:
        using Ptr = std::shared_ptr<JointTrajectoryPlan>;
        using PiecewisePolynomial = drake::trajectories::PiecewisePolynomial<double>;
        JointTrajectoryPlan(const PiecewisePolynomial& joint_trajectory);
        ~JointTrajectoryPlan() = default;

        // The method in base class
        PlanType GetPlanType() const override { return PlanType::JointTrajectory; }
        bool HasFinished(const RobotArmMeasurement& measurement) const override {
            return measurement.time_stamp.since_plan_start_second >= TrajectoryDuration();
        }

        // The total time for the trajectory
        double TrajectoryDuration() const {
            if (joint_trajectory_.get_number_of_segments() > 0) {
                return joint_trajectory_.end_time() - joint_trajectory_.start_time();
            } else {
                return 0.;
            }
        }

    protected:
        void computeCommand(
                const CommandInput& input,
                RobotArmCommand& command) override;

    private:
        // The trajectory
        PiecewisePolynomial joint_trajectory_;
        PiecewisePolynomial joint_velocity_trajectory_;

        // Cache
        Eigen::VectorXd q_command_cache, v_command_cache;
    };

    // Construct a trajectory plan
    JointTrajectoryPlan::Ptr ConstructJointTrajectoryPlan(
        const RigidBodyTree<double>& tree,
        const robot_msgs::JointTrajectoryGoal::ConstPtr &goal,
        const RobotArmMeasurement& measurement,
        const RobotArmCommand& latest_command
    );
}