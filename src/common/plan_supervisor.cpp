//
// Created by wei on 8/28/19.
//

#include "arm_runner/plan_supervisor.h"

void arm_runner::PlanSupervisor::processLoopIter() {
    // Get measurement
    rbt_communication_->GetMeasurement(measurement_cache);

    // Might need to switch the plan
    processPlanSwitch(measurement_cache);

    // After switching
    if(rbt_active_plan_ != nullptr) {
        rbt_active_plan_->ComputeCommand(measurement_cache, *rbt_communication_, command_cache);
    } else {
        RobotPlanBase::KeepCurrentConfigurationCommand(measurement_cache, command_cache);
    }

    // Software safety check
    bool command_safe = true;
    if(!command_safe) {
        // Keep the current pose
        RobotPlanBase::KeepCurrentConfigurationCommand(measurement_cache, command_cache);
    }

    // Send to robot
    rbt_communication_->SendCommand(command_cache);

    // Invalidate the command if not correct
    if(!command_safe) {
        rbt_active_plan_->StopPlan();
        rbt_active_plan_.reset();
    }
}


void arm_runner::PlanSupervisor::processPlanSwitch(const RobotArmMeasurement& measurement) {
    // Lock, current not implement
    // Use a fixed time lock

    // Check should I switch
    if (!shouldSwitchPlan(measurement)) return;

    // Do switching
    if(rbt_active_plan_ != nullptr)
        rbt_active_plan_->StopPlan();

    // Start the new one
    rbt_active_plan_ = switch_to_plan_;
    switch_to_plan_.reset();
    if(rbt_active_plan_ != nullptr) 
        rbt_active_plan_->InitializePlan();
}


bool arm_runner::PlanSupervisor::shouldSwitchPlan(const RobotArmMeasurement& measurement) {
    // No new plan, cannot switch
    if (switch_to_plan_ == nullptr) return false;

    // Now we have a new plan
    if (rbt_active_plan_ == nullptr) return true;

    // Now the active plan is not NULL
    if (is_streaming_plan(rbt_active_plan_->GetPlanType()) || rbt_active_plan_->HasFinished())
        return true;

    // Need to wait the current plan
    return false;
}