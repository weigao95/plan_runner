//
// Created by wei on 8/28/19.
//

#include "common/plan_supervisor.h"
#include "common/joint_trajectory_plan.h"
#include <chrono>


// The default keep current config plan number
constexpr int kKeepCurrentConfigPlanNumber = -1;

// The method for switching
void arm_runner::PlanSupervisor::initializeSwitchData() {
    action_to_current_plan_ = ActionToCurrentPlan::NoAction;
    plan_construction_data_.valid = false;
    plan_construction_data_.plan_number = kKeepCurrentConfigPlanNumber;
    plan_construction_data_.switch_to_plan = nullptr;
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
        ROS_INFO("Stop the plan as requested or it is not safe.");
        return true;
    }

    // Current plan has finished
    if (rbt_active_plan_ != nullptr && rbt_active_plan_->HasFinished(measurement)) {
        ROS_INFO("Stop the plan as it finished.");
        return true;
    }

    // Current plan don't finish, and we have new plan
    if (rbt_active_plan_ != nullptr
        && will_plan_stop_internally(rbt_active_plan_->GetPlanType())
        && plan_construction_data_.valid) {
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
    if(!command_safety)
        action_to_current_plan_ = ActionToCurrentPlan::SafetyStop;

    // Check should I switch
    if (!shouldSwitchPlan(*input.latest_measurement, latest_command)) {
        switch_mutex_.unlock();
        return;
    }

    // Copy the data and release the lock
    PlanConstructionData construction_data = plan_construction_data_;
    auto current_action = action_to_current_plan_;
    plan_construction_data_.valid = false;
    plan_construction_data_.switch_to_plan = nullptr;
    action_to_current_plan_ = ActionToCurrentPlan::NoAction;
    plan_start_time_second_ = input.latest_measurement->time_stamp.absolute_time_second;
    switch_mutex_.unlock();

    // Do switching
    if(rbt_active_plan_ != nullptr) {
        ROS_INFO("Stop plan %d at time %f",
                rbt_active_plan_->GetPlanNumber(), input.latest_measurement->time_stamp.absolute_time_second);
        rbt_active_plan_->StopPlan(current_action);
    }

    // Construct and switch to the new one
    // If no switch_to_plan, keep current position
    if(construction_data.valid && (construction_data.switch_to_plan != nullptr)) {
        rbt_active_plan_ = construction_data.switch_to_plan;
        rbt_active_plan_->SetPlanNumber(construction_data.plan_number);
    } else {
        rbt_active_plan_ = std::make_shared<KeepCurrentConfigurationPlan>();
        rbt_active_plan_->SetPlanNumber(kKeepCurrentConfigPlanNumber);
    }

    // Initialize the new plan
    if(rbt_active_plan_ != nullptr) {
        ROS_INFO("Start new plan %d at time %f",
                rbt_active_plan_->GetPlanNumber(), input.latest_measurement->time_stamp.absolute_time_second);
        rbt_active_plan_->InitializePlan(input);
    }
}