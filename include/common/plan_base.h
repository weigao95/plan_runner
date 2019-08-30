//
// Created by wei on 8/27/19.
//

#pragma once

#include "common/plan_common_types.h"
#include "common/communication_types.h"
#include "common/robot_communication.h"

namespace arm_runner {

    // The base of the plan
    class RobotPlanBase {
    public:
        using Ptr = std::shared_ptr<RobotPlanBase>;
        explicit RobotPlanBase(int plan_number) : status_(PlanStatus::Waiting), plan_number(plan_number) {};
        virtual ~RobotPlanBase() = default;

        // The supervisor would call these methods at initialization, preempt and stopping
        // After calling these method, the plan should finish elegantly.
        virtual void InitializePlan() { status_ = PlanStatus::Running; }
        virtual void StopPlan(ActionToCurrentPlan action) {
            for(const auto& callback : stop_callbacks_)
                callback(this, action);
            status_ = PlanStatus::Stopped;
        }

        // The management of flags
        virtual PlanType GetPlanType() const = 0;
        virtual bool HasFinished(const RobotArmMeasurement& measurement) const = 0;

        // The accessing interface of supervisor
        void ComputeCommand(
                const CommandInput& input,
                RobotArmCommand& command);

        // The command that just stay at current config
        static void KeepCurrentConfigurationCommand(
            const RobotArmMeasurement& measurement, 
            RobotArmCommand& command);


        // The plan number can be accessed outside
        // While the status is internal
    public:
        const int plan_number;
    protected:
        PlanStatus status_;

        // Keep current rbt configuration command
        virtual void computeCommand(
                const CommandInput& input,
                RobotArmCommand& command) = 0;

        // The callback function
        using StoppedCallbackFunction = std::function<void(RobotPlanBase*, ActionToCurrentPlan latest_action)>;
        std::vector<StoppedCallbackFunction> stop_callbacks_;
    public:
        void AddStoppedCallback(StoppedCallbackFunction func) { stop_callbacks_.emplace_back(std::move(func)); };
    };
}