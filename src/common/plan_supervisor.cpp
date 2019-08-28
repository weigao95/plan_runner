//
// Created by wei on 8/28/19.
//

#include "arm_runner/plan_supervisor.h"

void arm_runner::PlanSupervisor::processLoopIter() {
    // Get measurement
    rbt_communication_->GetMeasurement(measurement_cache);

    // Might need to switch the plan

    // After switching
    rbt_active_plan_->ComputeCommand(measurement_cache, *rbt_communication_, command_cache);
    rbt_communication_->SendCommand(command_cache);
}