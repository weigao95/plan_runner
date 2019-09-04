//
// Created by wei on 8/27/19.
//

#pragma once

#include "common/plan_common_types.h"
#include "common/communication_types.h"
#include "common/robot_communication.h"
#include "common/safety_checker_interface.h"


namespace arm_runner {

    // The base of the plan
    class RobotPlanBase {
    public:
        using Ptr = std::shared_ptr<RobotPlanBase>;
        explicit RobotPlanBase() : plan_number_(-1) {};
        virtual ~RobotPlanBase() = default;

        // The supervisor would call these methods at initialization, preempt and stopping
        // After calling these method, the plan should finish elegantly.
        virtual void InitializePlan(const CommandInput& input) {  }
        virtual void StopPlan(ActionToCurrentPlan action) {
            for(const auto& callback : stop_callbacks_)
                callback(this, action);
        }

        // The management of flags
        virtual PlanType GetPlanType() const = 0;
        virtual bool HasFinished(const RobotArmMeasurement& measurement) const = 0;


        // The external and internal processing interface of supervisor
    public:
        virtual void ComputeCommand(
            const CommandInput& input,
            RobotArmCommand& command) = 0;


        // The plan number can be accessed outside
        // While the status is internal
    public:
        void SetPlanNumber(int plan_number) { plan_number_ = plan_number; }
        int GetPlanNumber() const { return plan_number_; }
    protected:
        int plan_number_;


        // The safety checking function
        // Some checkers are specified to plan, which is maintained here
    public:
        void AddSafetyChecker(SafetyChecker::Ptr checker) { safety_checker_stack_.emplace_back(std::move(checker)); };
        bool CheckSafety(const CommandInput& input, const RobotArmCommand& command);
    protected:
        std::vector<SafetyChecker::Ptr> safety_checker_stack_;


        // The callback function
    protected:
        using StoppedCallbackFunction = std::function<void(RobotPlanBase*, ActionToCurrentPlan latest_action)>;
        std::vector<StoppedCallbackFunction> stop_callbacks_;
    public:
        void AddStoppedCallback(StoppedCallbackFunction func) { stop_callbacks_.emplace_back(std::move(func)); };
    };


    // The default plan that keeps the current configuration
    class KeepCurrentConfigurationPlan : public RobotPlanBase {
    public:
        using Ptr = std::shared_ptr<KeepCurrentConfigurationPlan>;
        explicit KeepCurrentConfigurationPlan();
        ~KeepCurrentConfigurationPlan() override = default;

        // The interface
        void InitializePlan(const CommandInput& input) override;
        PlanType GetPlanType() const override { return PlanType::KeepCurrentConfigurationPlan; }
        bool HasFinished(const RobotArmMeasurement& measurement) const override { return false; }

        // Copy the measurement to command
        // This WILL CAUSE DRIFTING FOR A LONG HORIZON
        static void CopyConfigurationToCommand(
            const RobotArmMeasurement& measurement,
            RobotArmCommand& command);

        // The command just copy kept_config
    protected:
        void ComputeCommand(
            const CommandInput& input,
            RobotArmCommand& command) override;
    private:
        RobotArmMeasurement kept_configuration_;
    };
}