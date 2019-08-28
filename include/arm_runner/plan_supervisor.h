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
        bool shouldSwitchPlan(const RobotArmMeasurement& measurement);
        void processPlanSwitch(const RobotArmMeasurement& measurement);

        // The processing loop
        void processLoopIter();
    private:
        // The real state
        std::unique_ptr<RobotCommunication> rbt_communication_;
        RobotPlanBase::Ptr rbt_active_plan_;

        // The field for switch to
        RobotPlanBase::Ptr switch_to_plan_;

        // The cache
        RobotArmMeasurement measurement_cache;
        RobotArmCommand command_cache;
    };

}