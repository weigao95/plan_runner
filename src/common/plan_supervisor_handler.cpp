//
// Created by wei on 8/30/19.
//

#include "common/plan_supervisor.h"
#include "common/robot_plan/forceguard_checker.h"
#include "common/robot_plan/joint_trajectory_plan.h"
#include "common/robot_plan/ee_trajectory_velocity_command.h"
#include "common/robot_plan/joint_streaming.h"


// The handler for joint trajectory action
void arm_runner::PlanSupervisor::initializeServiceActions() {
    // The joint trajectory action
    joint_trajectory_action_ = std::make_shared<actionlib::SimpleActionServer<robot_msgs::JointTrajectoryAction>>(
        node_handle_, "/plan_runner/JointTrajectory",
        boost::bind(&PlanSupervisor::HandleJointTrajectoryAction, this, _1), false);
    joint_trajectory_action_->start();

    // The joint trajectory action
    ee_trajectory_action_ = std::make_shared<actionlib::SimpleActionServer<robot_msgs::CartesianTrajectoryAction>>(
        node_handle_, "/plan_runner/CartesianTrajectory",
        boost::bind(&PlanSupervisor::HandleEETrajectoryAction, this, _1), false);
    ee_trajectory_action_->start();

    // The plan-end service
    plan_end_server_ = std::make_shared<ros::ServiceServer>(
        node_handle_.advertiseService("/plan_runner/stop_plan",
        &PlanSupervisor::HandleEndPlanService, this));

    // The plan-end service
    plan_end_server_ = std::make_shared<ros::ServiceServer>(
        node_handle_.advertiseService("/plan_runner/start_streaming",
        &PlanSupervisor::HandleStartStreamingService, this));
}


// The handler function for action and service
void arm_runner::PlanSupervisor::HandleJointTrajectoryAction(
        const robot_msgs::JointTrajectoryGoal::ConstPtr &goal){
    // Construct the plan
    auto plan = JointTrajectoryPlan::ConstructFromMessage(*tree_, goal);
    if( plan == nullptr
    || (plan->LoadParameterFrom(parameter_map_) == LoadParameterStatus::FatalError)) {
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
    auto plan = EETrajectoryVelocityCommandPlan::ConstructFromMessage(*tree_, goal);

    // Check failure
    if( plan == nullptr
    || (plan->LoadParameterFrom(parameter_map_) == LoadParameterStatus::FatalError)) {
        robot_msgs::CartesianTrajectoryResult result;
        result.status.status = result.status.STOPPED_BY_SAFETY_CHECK;
        ee_trajectory_action_->setAborted(result);
        return;
    }

    // Send to queue and wait for the task being accomplished
    constexpr int WAIT_RESULT_TIME_MS = 300;
    appendAndWaitForTrajectoryPlan<robot_msgs::CartesianTrajectoryAction, robot_msgs::CartesianTrajectoryResult>(
        ee_trajectory_action_, plan, WAIT_RESULT_TIME_MS);
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


bool arm_runner::PlanSupervisor::HandleStartStreamingService(
    robot_msgs::StartStreamingPlan::Request &request,
    robot_msgs::StartStreamingPlan::Response &response
) {
    // Construct the plan from msg
    RobotPlanBase::Ptr plan = nullptr;
    auto streaming_type = request.streaming_type;
    switch (streaming_type.value) {
        case streaming_type.JOINT_POSITION_STREAMING:
            plan = std::make_shared<JointPositionStreamingPlan>(
                    node_handle_, "/plan_runner/joint_space_streaming_setpoint");
            break;
        default:
            break;
    }

    // Check failure
    if( plan == nullptr
        || (plan->LoadParameterFrom(parameter_map_) == LoadParameterStatus::FatalError)) {
        response.plan_number = -1;
        response.status.status = response.status.ERROR;
        return false;
    }

    // The force guard
    for(const auto& guard_msg : request.force_guard) {
        auto checker_vec = ExternalForceGuardChecker::ConstructCheckersFromForceGuardMessage(*tree_, guard_msg);
        for(const auto& checker : checker_vec) {
            plan->AddSafetyChecker(checker);
        }
    }

    // En-Queue
    int plan_number;
    lockAndAppendPlan(plan, plan_number);

    // From now on, the plan is valid
    response.status.status = response.status.RUNNING;
    response.plan_number = plan_number;
    return true;
}


void arm_runner::PlanSupervisor::lockAndAppendPlan(const arm_runner::RobotPlanBase::Ptr& plan, int& plan_number) {
    // The construction should be OK
    DRAKE_ASSERT(plan != nullptr);
    auto finish_callback = [this](RobotPlanBase* robot_plan, ActionToCurrentPlan action) -> void {
        // Push this task to result
        FinishedPlanQueue::Record record{robot_plan->GetPlanNumber(), action};
        this->finished_plan_queue_.LockAndAppend(record);
    };
    plan->AddStoppedCallback(finish_callback);

    // Send to active task
    switch_mutex_.lock();
    switch_to_plan_ = plan;
    int current_plan_number = plan_number_;
    plan->SetPlanNumber(current_plan_number);
    plan_number_++;
    switch_mutex_.unlock();

    // Return the plan number
    plan_number = current_plan_number;
}