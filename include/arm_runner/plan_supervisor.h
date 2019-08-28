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
        void processLoopIter();
    private:
        // The real state
        std::unique_ptr<RobotCommunication> rbt_communication_;
        RobotPlanBase::Ptr rbt_active_plan_;
        RobotPlanBase::Ptr keep_current_config_;

        // The cache
        RobotArmMeasurement measurement_cache;
        RobotArmCommand command_cache;
    };

}