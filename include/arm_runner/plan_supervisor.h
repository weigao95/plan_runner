//
// Created by wei on 8/28/19.
//

#pragma once

#include "arm_runner/plan_base.h"
#include "arm_runner/plan_common_types.h"
#include "arm_runner/robot_communication.h"

namespace arm_runner {

    class PlanSupervisor {
    protected:
        // The method to handle plan switch
        bool shouldSwitchPlan(const RobotArmMeasurement& measurement) const;
        void processPlanSwitch(const RobotArmMeasurement& measurement);

        // The processing loop
        void processLoopIter();

    private:
        // The real state
        // Can only be mutated in processLoopIter()
        std::unique_ptr<RobotCommunication> rbt_communication_;
        RobotPlanBase::Ptr rbt_active_plan_;
        double plan_start_time_second_;

        // The field for switching, might be accessed by other thread
        // Protected by the lock
        bool stop_current_;
        RobotPlanBase::Ptr switch_to_plan_;

        // The cache
        RobotArmMeasurement measurement_cache;
        RobotArmCommand command_cache;
    };
}