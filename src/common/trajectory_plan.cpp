//
// Created by wei on 8/28/19.
//

#include "common/trajectory_plan.h"


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


arm_runner::JointTrajectoryPlan::JointTrajectoryPlan(
        const arm_runner::JointTrajectoryPlan::PiecewisePolynomial &joint_trajectory)
        : joint_trajectory_(joint_trajectory) {
    DRAKE_ASSERT(joint_trajectory_.cols() == 1);
    joint_velocity_trajectory_ = joint_trajectory_.derivative(1);
}


arm_runner::JointTrajectoryPlan::Ptr arm_runner::ConstructJointTrajectoryPlan(
    const RigidBodyTree<double> &tree,
    const robot_msgs::JointTrajectoryGoal::ConstPtr &goal,
    const arm_runner::RobotArmMeasurement &measurement,
    const arm_runner::RobotArmCommand &latest_command
) {
    int num_knot_points = goal->trajectory.points.size();
    int num_joints = tree.get_num_positions();
    const trajectory_msgs::JointTrajectory &trajectory = goal->trajectory;
    std::vector<Eigen::MatrixXd> knots(num_knot_points,
                                       Eigen::MatrixXd::Zero(num_joints, 1));
    std::map<std::string, int> name_to_idx = tree.computePositionNameToIndexMap();

    std::vector<double> input_time;
    for (int i = 0; i < num_knot_points; ++i) {
        const trajectory_msgs::JointTrajectoryPoint &traj_point =
                trajectory.points[i];
        for (int j = 0; j < trajectory.joint_names.size(); ++j) {
            std::string joint_name = trajectory.joint_names[j];
            if (name_to_idx.count(joint_name) == 0) {
                continue;
            }

            int joint_idx = name_to_idx[joint_name];
            // Treat the matrix at knots[i] as a column vector.
            if (i == 0) {
                // Always start moving from the position which we're
                // currently commanding.
                knots[0](joint_idx, 0) = measurement.joint_position[joint_idx];
            } else {
                knots[i](joint_idx, 0) = traj_point.positions[j];
            }
        }

        input_time.push_back(traj_point.time_from_start.toSec());
    }

    // OK
    const Eigen::MatrixXd knot_dot = Eigen::MatrixXd::Zero(num_joints, 1);
    auto plan = std::make_shared<JointTrajectoryPlan>(
            JointTrajectoryPlan::PiecewisePolynomial::Cubic(input_time, knots, knot_dot, knot_dot));

    // Add callback
    return plan;
}