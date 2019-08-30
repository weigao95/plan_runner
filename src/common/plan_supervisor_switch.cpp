//
// Created by wei on 8/28/19.
//

#include "common/plan_supervisor.h"
#include "common/trajectory_plan.h"
#include <chrono>


// The time for wait for switching lock
constexpr int LOOP_MUTEX_TIMEOUT_MS = 5;

// The method for switching
void arm_runner::PlanSupervisor::initializeSwitchData() {
    action_to_current_plan_ = ActionToCurrentPlan::NoAction;
    plan_construction_data_.valid = false;
    plan_construction_data_.plan_number = -1;
    plan_construction_data_.joint_trajectory_goal = nullptr;
    plan_construction_data_.cartesian_trajectory_goal = nullptr;
}


bool arm_runner::PlanSupervisor::shouldSwitchPlan(
    const RobotArmMeasurement& measurement,
    const RobotArmCommand& latest_command
) const {
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
            rbt_active_plan_->StopPlan(action_to_current_plan_);

        // Construct and switch to the new one
        rbt_active_plan_ = constructNewPlan(input, latest_command);
        if(rbt_active_plan_ != nullptr) {
            rbt_active_plan_->InitializePlan();
        }

        // Cleanup
        plan_construction_data_.valid = false;
        action_to_current_plan_ = ActionToCurrentPlan::NoAction;
        plan_start_time_second_ = input.latest_measurement->time_stamp.absolute_time_second;

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
        actionlib::SimpleActionServer<robot_msgs::JointTrajectoryAction>>(node_handle_, "/plan_runner/iiwa/JointTrajectory",
            boost::bind(&PlanSupervisor::HandleJointTrajectoryAction, this, _1),
            false);
    joint_trajectory_action_->start();
    std::cout << "Start the action" << std::endl;

    // The plan-end service
    plan_end_server_ = std::make_shared<ros::ServiceServer>(
            node_handle_.advertiseService(
                    "/plan_runner/stop_plan",
                    &PlanSupervisor::HandleEndPlanService, this));
}


// Commonly used utils functions
void arm_runner::PlanSupervisor::lockAndCheckPlanStatus(
        int current_plan_number,
        bool& current_plan_in_queue,
        bool& new_plan_in_queue,
        ActionToCurrentPlan& action_to_current_plan
) {
    std::lock_guard<std::mutex> guard(finished_task_queue_.mutex);
    for(const auto& finished_plan : finished_task_queue_.queue) {
        if(finished_plan.plan_number == current_plan_number) {
            current_plan_in_queue = true;
            action_to_current_plan = finished_plan.action_to_plan;
        } else if(finished_plan.plan_number > current_plan_number)
            new_plan_in_queue = true;
    }
}


void arm_runner::PlanSupervisor::lockAndEnQueue(FinishedPlanRecord record) {
    std::lock_guard<std::mutex> guard(finished_task_queue_.mutex);
    finished_task_queue_.queue.emplace_back(record);
}


void arm_runner::PlanSupervisor::HandleJointTrajectoryAction(const robot_msgs::JointTrajectoryGoalConstPtr &goal) {
    // Send to active task
    switch_mutex_.lock();
    plan_construction_data_.valid = true;
    plan_construction_data_.type = PlanType::JointTrajectory;
    plan_construction_data_.joint_trajectory_goal = goal;
    int current_plan_number = plan_construction_data_.plan_number++;
    switch_mutex_.unlock();

    // Wait for the task being accomplished
    do {
        constexpr int WAIT_RESULT_TIME_MS = 200;
        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_RESULT_TIME_MS));

        // Read the status
        bool current_plan_in_queue = false;
        bool new_plan_in_queue = false;
        ActionToCurrentPlan action_to_current_plan = ActionToCurrentPlan::NoAction;
        lockAndCheckPlanStatus(current_plan_number,
                current_plan_in_queue, new_plan_in_queue, action_to_current_plan);

        // Determine the state
        if(current_plan_in_queue) {
            robot_msgs::JointTrajectoryResult result;
            if(action_to_current_plan == ActionToCurrentPlan::NoAction
            || action_to_current_plan == ActionToCurrentPlan::NormalStop) {
                result.status.status = result.status.FINISHED_NORMALLY;
                joint_trajectory_action_->setSucceeded(result);
            } else {
                result.status.status = result.status.STOPPED_BY_SAFETY_CHECK;
                joint_trajectory_action_->setAborted(result);
            }
            return;
        } else if ((!current_plan_in_queue) && new_plan_in_queue) {
            robot_msgs::JointTrajectoryResult result;
            result.status.status = result.status.STOPPED_BY_EXTERNAL_TRIGGER;
            joint_trajectory_action_->setPreempted(result);
            return;
        } else
            continue;

    } while(true);
}


bool arm_runner::PlanSupervisor::HandleEndPlanService(
    std_srvs::Trigger::Request &req,
    std_srvs::Trigger::Response &res
) {
    std::lock_guard<std::timed_mutex> guard(switch_mutex_);
    action_to_current_plan_ = ActionToCurrentPlan::NormalStop;
    res.success = true;
    return true;
}
