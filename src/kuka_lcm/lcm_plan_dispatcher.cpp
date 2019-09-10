//
// Created by wei on 9/9/19.
//

#include <std_srvs/Trigger.h>

#include "lcm_plan_dispatcher.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "robot_msgs/JointTrajectoryActionGoal.h"


plan_runner::KukaPlanDispatcherLCM::KukaPlanDispatcherLCM(
        ros::NodeHandle &nh,
        std::string lcm_plan_channel, std::string lcm_stop_channel,
        const std::string& joint_trajectory_action, const std::string& stop_plan_service
) : node_handle_(nh),
    lcm_plan_channel_(std::move(lcm_plan_channel)), lcm_stop_channel_(std::move(lcm_stop_channel))
{
    joint_trajectory_client_ = std::make_shared<actionlib::SimpleActionClient<robot_msgs::JointTrajectoryAction>>(
            joint_trajectory_action, true);
    stop_plan_client_ = std::make_shared<ros::ServiceClient>(node_handle_.serviceClient<std_srvs::Trigger>(stop_plan_service));
}


void plan_runner::KukaPlanDispatcherLCM::Start() {
    // Check the server
    joint_trajectory_client_->waitForServer();

    // Invoke lcm
    lcm::LCM lcm;
    lcm.subscribe(lcm_plan_channel_, &KukaPlanDispatcherLCM::handleJointTrajectoryPlan, this);
    lcm.subscribe(lcm_stop_channel_, &KukaPlanDispatcherLCM::handleStopPlan, this);

    // Spin
    while (0 == lcm.handle()) {};
}


void plan_runner::KukaPlanDispatcherLCM::handleJointTrajectoryPlan(
    const lcm::ReceiveBuffer *,
    const std::string &,
    const robotlocomotion::robot_plan_t *tape
) {
    // Basic check
    const auto& lcm_plan = *tape;
    if(lcm_plan.num_states < 2 || lcm_plan.plan.size() < 2) return;

    // Fill the joint name
    auto goal_msg = robot_msgs::JointTrajectoryGoal();
    for(size_t i = 0; i < lcm_plan.plan[0].joint_name.size(); i++) {
        auto name_i = lcm_plan.plan[0].joint_name[i];
        goal_msg.trajectory.joint_names.emplace_back(std::move(name_i));
    }

    // Fill the plan
    for(size_t i = 0; i < lcm_plan.plan.size(); i++) {
        const auto& state_i = lcm_plan.plan[i];
        trajectory_msgs::JointTrajectoryPoint point;
        for(size_t j = 0; j < state_i.joint_position.size(); j++) {
            point.positions.push_back(state_i.joint_position[j]);

            // Other properties are zero
            point.velocities.push_back(0);
            point.accelerations.push_back(0);
            point.effort.push_back(0);
        }

        // Append this point
        point.time_from_start.fromSec(double(state_i.utime) / 1e6);
        goal_msg.trajectory.points.emplace_back(std::move(point));
    }

    // Send to robot
    std::lock_guard<std::mutex> guard(mutex_);
    joint_trajectory_client_->sendGoal(goal_msg);
}

void plan_runner::KukaPlanDispatcherLCM::handleStopPlan(
    const lcm::ReceiveBuffer *,
    const std::string &,
    const robotlocomotion::robot_plan_t *
) {
    std_srvs::Trigger request;
    std::lock_guard<std::mutex> guard(mutex_);
    stop_plan_client_->call(request);
}
