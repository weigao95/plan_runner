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

#include "common/plan_base.h"


namespace arm_runner {

    class EETrajectoryPlan : public RobotPlanBase {
    public:
        using Ptr = std::shared_ptr<EETrajectoryPlan>;
        using PiecewisePolynomial = drake::trajectories::PiecewisePolynomial<double>;
        using PiecewiseQuaternionSlerp = drake::trajectories::PiecewiseQuaternionSlerp<double>;

        // The actual computation
    protected:
        void computeCommand(
            const CommandInput& input,
            RobotArmCommand& command) override;
        static Eigen::Vector3d logSO3(const Eigen::Matrix3d& rotation);

        // The members
    private:
        static const int world_frame;
        int task_frame_index_;
        PiecewisePolynomial ee_xyz_trajectory_;
        PiecewiseQuaternionSlerp ee_orientation_trajectory_;
        Eigen::Vector3d kp_rotation_;
        Eigen::Vector3d kp_translation_;

        // The cache
        Eigen::MatrixXd ee_twist_jacobian_expressed_in_ee;
    };
}
