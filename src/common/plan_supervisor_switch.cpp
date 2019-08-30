//
// Created by wei on 8/28/19.
//

#include "common/plan_supervisor.h"
#include "common/trajectory_plan.h"
#include <chrono>


// The method for switching
void arm_runner::PlanSupervisor::initializeSwitchData() {
    action_to_current_plan_ = ActionToCurrentPlan::NoAction;
    plan_construction_data_.valid = false;
    plan_construction_data_.plan_number = -1;
    plan_construction_data_.switch_to_plan = nullptr;
}


bool arm_runner::PlanSupervisor::shouldSwitchPlan(
    const RobotArmMeasurement& measurement,
    const RobotArmCommand& latest_command
) const {
    // Commanded to stop
    if (action_to_current_plan_ != ActionToCurrentPlan::NoAction) return true;

    // Current plan has finished
    if (rbt_active_plan_ != nullptr && rbt_active_plan_->HasFinished(measurement))
        return true;

    // Current plan don't finish, and we have new plan
    if (rbt_active_plan_ != nullptr
        && is_streaming_plan(rbt_active_plan_->GetPlanType())
        && plan_construction_data_.valid) {
        return true;
    }

    // Current no plan, a new plan is valid
    if (rbt_active_plan_ == nullptr && plan_construction_data_.valid)
        return true;

    // Other cases, just false
    return false;
}


void arm_runner::PlanSupervisor::processPlanSwitch(
    const CommandInput& input,
    const RobotArmCommand& latest_command,
    bool command_safety
) {
    // It's OK as other operation on this lock is rather small
    std::lock_guard<std::mutex> guard(switch_mutex_);

    // Command is unsafe
    if(!command_safety)
        action_to_current_plan_ = ActionToCurrentPlan::SafetyStop;

    // Now this method has lock
    // Check should I switch
    if (!shouldSwitchPlan(*input.latest_measurement, latest_command)) {
        return;
    }

    // Do switching
    if(rbt_active_plan_ != nullptr)
        rbt_active_plan_->StopPlan(action_to_current_plan_);

    // Construct and switch to the new one
    rbt_active_plan_ = plan_construction_data_.switch_to_plan;
    if(rbt_active_plan_ != nullptr) {
        rbt_active_plan_->InitializePlan(input);
    }

    // Cleanup
    plan_construction_data_.valid = false;
    plan_construction_data_.switch_to_plan = nullptr;
    action_to_current_plan_ = ActionToCurrentPlan::NoAction;
    plan_start_time_second_ = input.latest_measurement->time_stamp.absolute_time_second;
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
    // Construct the plan
    auto plan = ConstructJointTrajectoryPlan(joint_name_to_idx_, num_joint_, goal);
    auto finish_callback = [this](RobotPlanBase* robot_plan, ActionToCurrentPlan action) -> void {
        // Push this task to result
        FinishedPlanRecord record{robot_plan->GetPlanNumber(), action};
        this->lockAndEnQueue(record);
    };
    plan->AddStoppedCallback(finish_callback);

    // Send to active task
    switch_mutex_.lock();
    plan_construction_data_.valid = true;
    plan_construction_data_.type = PlanType::JointTrajectory;
    plan_construction_data_.switch_to_plan = plan;
    plan_construction_data_.plan_number++;
    int current_plan_number = plan_construction_data_.plan_number;
    switch_mutex_.unlock();

    // Update the plan number
    plan->SetPlanNumber(current_plan_number);

    // Wait for the task being accomplished
    do {
        constexpr int WAIT_RESULT_TIME_MS = 300;
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
    std::lock_guard<std::mutex> guard(switch_mutex_);
    action_to_current_plan_ = ActionToCurrentPlan::NormalStop;
    res.success = true;
    return true;
}
