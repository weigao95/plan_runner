//
// Created by wei on 8/28/19.
//

#include "arm_runner/plan_supervisor.h"
#include "arm_runner/time_utils.h"
#include "arm_runner/trajectory_plan.h"

#include <cstring>
#include <chrono>


// The method for switching
void arm_runner::PlanSupervisor::initializeSwitchData() {
    action_to_current_plan_ = ActionToCurrentPlan::NoAction;
    memset(&plan_construction_data_, 0, sizeof(plan_construction_data_));
    plan_construction_data_.valid = false;
    plan_construction_data_.joint_trajectory_goal = nullptr;
    plan_construction_data_.cartesian_trajectory_goal = nullptr;
}


bool arm_runner::PlanSupervisor::shouldSwitchPlan(const RobotArmMeasurement& measurement, const RobotArmCommand& latest_command) const {
    // Commanded to stop
    if (action_to_current_plan_ != ActionToCurrentPlan::NoAction) return true;

    // No new plan, cannot switch
    if (!plan_construction_data_.valid) return false;

    // Now we have a new plan
    if (rbt_active_plan_ == nullptr) return true;

    // Now the active plan is not NULL
    if (is_streaming_plan(rbt_active_plan_->GetPlanType()) || rbt_active_plan_->HasFinished(measurement))
        return true;

    // Need to wait the current plan
    return false;
}


void arm_runner::PlanSupervisor::processPlanSwitch(
        const CommandInput& input,
        const RobotArmCommand& latest_command
) {
    // Use a fixed time lock
    using std::chrono::milliseconds;
    if(switch_mutex_.try_lock_for(milliseconds(LOOP_MUTEX_TIMEOUT_MS))) {
        // Now this method has lock
        // Check should I switch
        if (!shouldSwitchPlan(*input.latest_measurement, latest_command)) {
            switch_mutex_.unlock();
            return;
        }

        // Do switching
        if(rbt_active_plan_ != nullptr)
            rbt_active_plan_->StopPlan();

        // Construct and switch to the new one
        rbt_active_plan_ = constructNewPlan(input, latest_command);
        if(rbt_active_plan_ != nullptr) {
            rbt_active_plan_->InitializePlan();
        }

        // Cleanup
        plan_construction_data_.valid = false;
        action_to_current_plan_ = ActionToCurrentPlan::NoAction;
        plan_start_time_second_ = now_in_second();

        // Release the lock and return
        switch_mutex_.unlock();
        return;
    } else {
        // No lock
        // Just keep current plan
        return;
    }
}


// The handler for joint trajectory action
void arm_runner::PlanSupervisor::initializeServiceActions() {
    // The joint trajectory action
    joint_trajectory_action_ = std::make_shared<
            actionlib::SimpleActionServer<robot_msgs::JointTrajectoryAction>>(
            node_handle_, "JointTrajectory",
            boost::bind(&PlanSupervisor::HandleJointTrajectoryAction, this, _1),
            false);

    // The plan-end service
    plan_end_server_ = std::make_shared<ros::ServiceServer>(
            node_handle_.advertiseService(
                    "/plan_runner/stop_plan",
                    &PlanSupervisor::HandleEndPlanService, this));
}


void arm_runner::PlanSupervisor::HandleJointTrajectoryAction(const robot_msgs::JointTrajectoryGoal::ConstPtr &goal) {
    std::lock_guard<std::timed_mutex> guard(switch_mutex_);
    plan_construction_data_.valid = true;
    plan_construction_data_.type = PlanType::JointTrajectory;
    plan_construction_data_.joint_trajectory_goal = goal;
}

bool arm_runner::PlanSupervisor::HandleEndPlanService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    std::lock_guard<std::timed_mutex> guard(switch_mutex_);
    action_to_current_plan_ = ActionToCurrentPlan::NormalStop;
    res.success = true;
    return true;
}