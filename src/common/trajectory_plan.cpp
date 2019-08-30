//
// Created by wei on 8/28/19.
//

#include "common/trajectory_plan.h"


arm_runner::JointTrajectoryPlan::JointTrajectoryPlan(
    std::vector<double> input_time, std::vector<Eigen::MatrixXd> knots
) : input_time_(std::move(input_time)), knots_(std::move(knots)) {}


void arm_runner::JointTrajectoryPlan::InitializePlan(const arm_runner::CommandInput &input) {
    // Fill the first knot
    auto nq = knots_[0].rows();
    DRAKE_ASSERT(nq == input.robot_rbt->get_num_positions());
    for(auto joint_idx = 0; joint_idx < nq; joint_idx++) {
        knots_[0](joint_idx, 0) = input.latest_measurement->joint_position[joint_idx];
    }

    // Compute the trajectory
    const Eigen::MatrixXd knot_dot = Eigen::MatrixXd::Zero(nq, 1);
    joint_trajectory_ = JointTrajectoryPlan::PiecewisePolynomial::Cubic(input_time_, knots_, knot_dot, knot_dot);
    joint_velocity_trajectory_ = joint_trajectory_.derivative(1);

    // Setup the flag
    RobotPlanBase::InitializePlan(input);
}


void arm_runner::JointTrajectoryPlan::computeCommand(
        const CommandInput& input,
        arm_runner::RobotArmCommand &command
) {
    const auto& measurement = *input.latest_measurement;
    auto t = measurement.time_stamp.since_plan_start_second;
    DRAKE_ASSERT(t > 0);
    q_command_cache = joint_trajectory_.value(t);
    v_command_cache = joint_velocity_trajectory_.value(t);

    // Write to command
    command.set_invalid();
    for(auto i = 0; i < q_command_cache.size(); i++) {
        command.joint_position[i] = q_command_cache[i];
        command.joint_velocities[i] = v_command_cache[i];
    }
    command.position_validity = true;
    command.velocity_validity = true;
}


arm_runner::JointTrajectoryPlan::Ptr arm_runner::ConstructJointTrajectoryPlan(
    const std::map<std::string, int>& joint_name_to_idx,
    int num_joints,
    const robot_msgs::JointTrajectoryGoal::ConstPtr &goal
) {
    // Basic info
    int num_knot_points = goal->trajectory.points.size();
    const trajectory_msgs::JointTrajectory &trajectory = goal->trajectory;
    std::vector<Eigen::MatrixXd> knots(num_knot_points, Eigen::MatrixXd::Zero(num_joints, 1));

    // Iterate
    std::vector<double> input_time;
    for (int i = 0; i < num_knot_points; ++i) {
        const trajectory_msgs::JointTrajectoryPoint &traj_point =
                trajectory.points[i];
        for (int j = 0; j < trajectory.joint_names.size(); ++j) {
            // Check the current joint
            std::string joint_name = trajectory.joint_names[j];
            auto iter = joint_name_to_idx.find(joint_name);

            // There is no such joint
            if (iter == joint_name_to_idx.end()) {
                continue;
            }

            // This name is in map
            int joint_idx = iter->second;
            // Treat the matrix at knots[i] as a column vector.
            if (i == 0) {
                // Will be filled later
                knots[0](joint_idx, 0) = 0;
            } else {
                knots[i](joint_idx, 0) = traj_point.positions[j];
            }
        }

        input_time.push_back(traj_point.time_from_start.toSec());
    }

    // OK
    auto plan = std::make_shared<JointTrajectoryPlan>(std::move(input_time), std::move(knots));
    return plan;
}