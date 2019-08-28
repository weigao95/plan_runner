//
// Created by wei on 8/27/19.
//

#pragma once

#include "arm_runner/plan_common_types.h"
#include "arm_runner/communication_types.h"

namespace arm_runner {

    // The base of the plan
    class RobotPlanBase {
    public:
        RobotPlanBase() : status_(PlanStatus::Waiting) {};
        virtual ~RobotPlanBase() = default;

        // The supervisor would call these methods at initialization, preempt and stopping
        // After calling these method, the plan should finish elegantly.
        virtual void InitializePlan() { status_ = PlanStatus::Running; }
        virtual void PreemptPlan() { status_ = PlanStatus::Preempted; }
        virtual void StopPlan() { status_ = PlanStatus::Stopped; }

        // The management of flags
        virtual PlanType GetPlanType() const = 0;

        // The accessing interface of supervisor
        void ComputeCommand(const RobotArmMeasurement& measurement, RobotArmCommand& command);

    protected:
        // The only thing here is the status
        PlanStatus status_;

        // Keep current rbt configuration command
        static void keepCurrentConfigurationCommand(const RobotArmMeasurement& measurement, RobotArmCommand& command);
        virtual void computeCommand(const RobotArmMeasurement& measurement, RobotArmCommand& command) = 0;
    };

    // The default plan, just stay here
    class KeepCurrentConfigurationPlan : public RobotPlanBase {
    public:
        KeepCurrentConfigurationPlan() = default;
        PlanType GetPlanType() const override { return PlanType::KeepCurrentConfiguration; }
    protected:
        void computeCommand(const RobotArmMeasurement& measurement, RobotArmCommand& command) override {
            RobotPlanBase::keepCurrentConfigurationCommand(measurement, command);
        }
    };
}