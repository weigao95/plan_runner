//
// Created by wei on 8/28/19.
//

#include "supervisor/plan_supervisor.h"
#include "robot_plan/joint_trajectory_plan.h"
#include <chrono>


// The method for switching
void arm_runner::PlanSupervisor::initializeSwitchData() {
    action_to_current_plan_ = ActionToCurrentPlan::NoAction;
    switch_to_plan_ = nullptr;
    plan_number_ = 0;
    finished_plan_queue_.Initialize();
}


bool arm_runner::PlanSupervisor::shouldSwitchPlan(
    const RobotArmMeasurement& measurement,
    const RobotArmCommand& latest_command
) const {
    // Null plan is always unsafe
    if(rbt_active_plan_ == nullptr)
        return true;

    // Commanded to stop
    if (action_to_current_plan_ != ActionToCurrentPlan::NoAction) {
        return true;
    }

    // Current plan has finished
    if (rbt_active_plan_ != nullptr && rbt_active_plan_->HasFinished(measurement)) {
        return true;
    }

    // Current plan don't finish, and we have new plan
    if (rbt_active_plan_ != nullptr
        && !(will_plan_stop_internally(rbt_active_plan_->GetPlanType()))
        && switch_to_plan_ != nullptr) {
        return true;
    }

    // Other cases, just false
    return false;
}


void arm_runner::PlanSupervisor::processPlanSwitch(
    const CommandInput& input,
    const RobotArmCommand& latest_command,
    bool command_safety
) {
    // It's OK as other operation on this lock is rather small
    switch_mutex_.lock();

    // Command is unsafe
    if(!command_safety) {
        if(rbt_active_plan_ != nullptr
        && rbt_active_plan_->GetPlanType() != PlanType::KeepCurrentConfigurationPlan) {
            // That assumes keep current config is always safe, which might not be true
            action_to_current_plan_ = ActionToCurrentPlan::SafetyStop;
        }
    }

    // Check should I switch
    if (!shouldSwitchPlan(*input.latest_measurement, latest_command)) {
        switch_mutex_.unlock();
        return;
    }

    // Copy the data and release the lock
    auto switch_to_plan = switch_to_plan_;
    auto current_action = action_to_current_plan_;
    switch_to_plan_ = nullptr;
    action_to_current_plan_ = ActionToCurrentPlan::NoAction;

    // Need to construct keep current config plan
    int kept_config_plan_number = -1;
    if(switch_to_plan == nullptr){
        kept_config_plan_number = plan_number_;
        plan_number_++;
    }

    // We do need plan_construct_data_ anymore
    switch_mutex_.unlock();

    // Do switching
    if(rbt_active_plan_ != nullptr) {
        //ROS_INFO("Stop plan %d at time %f",
        //        rbt_active_plan_->GetPlanNumber(), input.latest_measurement->time_stamp.ToSecond());
        rbt_active_plan_->StopPlan(current_action);
    }

    // Construct and switch to the new one
    // If no switch_to_plan, keep current position
    if(switch_to_plan != nullptr) {
        rbt_active_plan_ = switch_to_plan;
    } else {
        DRAKE_ASSERT(kept_config_plan_number != -1);
        rbt_active_plan_ = std::make_shared<KeepCurrentConfigurationPlan>();
        rbt_active_plan_->SetPlanNumber(kept_config_plan_number);
    }

    // Initialize the new plan
    DRAKE_ASSERT(rbt_active_plan_ != nullptr);
    rbt_active_plan_->InitializePlan(input);
    //ROS_INFO("Start new plan with number %d", rbt_active_plan_->GetPlanNumber());
}