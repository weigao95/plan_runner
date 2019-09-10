//
// Created by wei on 8/28/19.
//

#include "robot_plan/joint_trajectory_plan.h"


plan_runner::JointTrajectoryPlan::JointTrajectoryPlan(
    std::vector<double> input_time, std::vector<Eigen::MatrixXd> knots
) : input_time_(std::move(input_time)),
    knots_(std::move(knots))
{}


std::shared_ptr<plan_runner::JointTrajectoryPlan> plan_runner::JointTrajectoryPlan::ConstructFromMessage(
    const RigidBodyTree<double>& tree,
    const robot_msgs::JointTrajectoryGoal::ConstPtr &goal
) {
    // Basic info
    const trajectory_msgs::JointTrajectory &trajectory = goal->trajectory;
    int num_knot_points = goal->trajectory.points.size();
    int num_joints = tree.get_num_positions();
    auto joint_name_to_idx = tree.computePositionNameToIndexMap();

    // Check the msg
    for(auto i = 0; i < num_knot_points; i++) {
        const auto &traj_point = trajectory.points[i];
        if(traj_point.positions.size() != trajectory.joint_names.size()) {
            ROS_INFO("The joint trajectory size is not correct");
            return nullptr;
        }
    }

    // Check the joint name
    for(const auto& required_joint : joint_name_to_idx) {
        // Check joint
        bool found = false;
        for(const auto& provided_joint : trajectory.joint_names) {
            if(provided_joint == required_joint.first) {
                found = true;
                break;
            }
        }

        // Not found
        if(!found) {
            ROS_INFO("The position for joint %s is not provided", required_joint.first.c_str());
            return nullptr;
        }
    }

    // Iterate
    std::vector<Eigen::MatrixXd> knots(num_knot_points, Eigen::MatrixXd::Zero(num_joints, 1));
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


void plan_runner::JointTrajectoryPlan::InitializePlan(const plan_runner::CommandInput &input) {
    // Use measurement or command to fill the first knot
    const double* q_init = nullptr;
    const auto& history = input.robot_history->GetCommandHistory();
    if(use_commanded_fwd_integration_ && (!history.empty())) {
        q_init =  history.back().joint_position;
    } else {
        q_init = input.latest_measurement->joint_position;
    }

    // Fill the first knot
    auto nq = knots_[0].rows();
    DRAKE_ASSERT(nq == input.robot_rbt->get_num_positions());
    for(auto joint_idx = 0; joint_idx < nq; joint_idx++) {
        knots_[0](joint_idx, 0) = q_init[joint_idx];
    }

    // Compute the trajectory
    const Eigen::MatrixXd knot_dot = Eigen::MatrixXd::Zero(nq, 1);
    joint_trajectory_ = JointTrajectoryPlan::PiecewisePolynomial::Cubic(input_time_, knots_, knot_dot, knot_dot);
    joint_velocity_trajectory_ = joint_trajectory_.derivative(1);

    // Setup the flag
    RobotPlanBase::InitializePlan(input);
}


void plan_runner::JointTrajectoryPlan::ComputeCommand(
        const CommandInput& input,
        plan_runner::RobotArmCommand &command
) {
    const auto& measurement = *input.latest_measurement;
    auto t = GetTimeSincePlanStartSecond(input.latest_measurement->time_stamp);
    DRAKE_ASSERT(t >= 0);
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
