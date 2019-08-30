//
// Created by wei on 8/30/19.
//

#include "common/plan_supervisor.h"
#include "common/joint_trajectory_plan.h"
#include "common/ee_trajectory_plan.h"
#include <chrono>


// The handler for joint trajectory action
void arm_runner::PlanSupervisor::initializeServiceActions() {
    // The joint trajectory action
    joint_trajectory_action_ = std::make_shared<actionlib::SimpleActionServer<robot_msgs::JointTrajectoryAction>>(
        node_handle_, "/plan_runner/JointTrajectory",
        boost::bind(&PlanSupervisor::HandleJointTrajectoryActionTemplate, this, _1), false);
    joint_trajectory_action_->start();

    // The joint trajectory action
    ee_trajectory_action_ = std::make_shared<actionlib::SimpleActionServer<robot_msgs::CartesianTrajectoryAction>>(
        node_handle_, "/plan_runner/CartesianTrajectory",
        boost::bind(&PlanSupervisor::HandleEETrajectoryAction, this, _1), false);
    ee_trajectory_action_->start();

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


void arm_runner::PlanSupervisor::HandleJointTrajectoryAction(
    const robot_msgs::JointTrajectoryGoal::ConstPtr &goal
) {
    // Construct the plan
    auto plan = JointTrajectoryPlan::ConstructFromMessage(joint_name_to_idx_, num_joint_, goal);
    if(plan == nullptr) {
        robot_msgs::JointTrajectoryResult result;
        result.status.status = result.status.STOPPED_BY_SAFETY_CHECK;
        joint_trajectory_action_->setAborted(result);
        return;
    }

    // The construction is OK
    auto finish_callback = [this](RobotPlanBase* robot_plan, ActionToCurrentPlan action) -> void {
        // Push this task to result
        FinishedPlanRecord record{robot_plan->GetPlanNumber(), action};
        this->lockAndEnQueue(record);
    };
    plan->AddStoppedCallback(finish_callback);

    // Send to active task
    switch_mutex_.lock();
    plan_construction_data_.valid = true;
    plan_construction_data_.switch_to_plan = plan;
    plan_construction_data_.plan_number++;
    int current_plan_number = plan_construction_data_.plan_number;
    switch_mutex_.unlock();

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



void arm_runner::PlanSupervisor::HandleJointTrajectoryActionTemplate(
        const robot_msgs::JointTrajectoryGoal::ConstPtr &goal){
    // Construct the plan
    auto plan = JointTrajectoryPlan::ConstructFromMessage(joint_name_to_idx_, num_joint_, goal);
    if(plan == nullptr) {
        robot_msgs::JointTrajectoryResult result;
        result.status.status = result.status.STOPPED_BY_SAFETY_CHECK;
        joint_trajectory_action_->setAborted(result);
        return;
    }

    // Send to queue and wait for the task being accomplished
    constexpr int WAIT_RESULT_TIME_MS = 300;
    appendAndWaitForTrajectoryPlan<robot_msgs::JointTrajectoryAction, robot_msgs::JointTrajectoryResult>(
        joint_trajectory_action_, plan, WAIT_RESULT_TIME_MS);
}


void arm_runner::PlanSupervisor::HandleEETrajectoryAction(
    const robot_msgs::CartesianTrajectoryGoal::ConstPtr& goal
) {
    // Construct the plan
    auto plan = EETrajectoryPlan::ConstructFromMessage(*tree_, goal);
    if(plan == nullptr) {
        robot_msgs::CartesianTrajectoryResult result;
        result.status.status = result.status.STOPPED_BY_SAFETY_CHECK;
        ee_trajectory_action_->setAborted(result);
        return;
    }
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