//
// Created by wei on 8/27/19.
//

#pragma once

#include "arm_runner/plan_common_types.h"
#include "arm_runner/communication_types.h"
#include "arm_runner/robot_communication.h"

namespace arm_runner {

    // The base of the plan
    class RobotPlanBase {
    public:
        using Ptr = std::shared_ptr<RobotPlanBase>;
        RobotPlanBase() : status_(PlanStatus::Waiting) {};
        virtual ~RobotPlanBase() = default;

        // The supervisor would call these methods at initialization, preempt and stopping
        // After calling these method, the plan should finish elegantly.
        virtual void InitializePlan() { status_ = PlanStatus::Running; }
        virtual void StopPlan() { status_ = PlanStatus::Stopped; }

        // The management of flags
        virtual PlanType GetPlanType() const = 0;
        virtual bool HasFinished() const { return false; }

        // The accessing interface of supervisor
        void ComputeCommand(
                const RobotArmMeasurement& measurement,
                const RobotCommunication& history,
                RobotArmCommand& command);

        // The command that just stay at current config
        static void KeepCurrentConfigurationCommand(
            const RobotArmMeasurement& measurement, 
            RobotArmCommand& command);

    protected:
        // The only thing here is the status
        PlanStatus status_;

        // Keep current rbt configuration command
        virtual void computeCommand(
                const RobotArmMeasurement& measurement,
                const RobotCommunication& history,
                RobotArmCommand& command) = 0;
    };
}