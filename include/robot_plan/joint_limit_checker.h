//
// Created by wei on 9/4/19.
//

#pragma once

#include "common/safety_checker_interface.h"
#include "common/processor_interface.h"

#include <memory>


namespace plan_runner {

    // Check the command velocity shouldn't be too large
    // Also check that the command is not nan
    class JointVelocityLimitChecker : public SafetyChecker {
    public:
        using Ptr = std::shared_ptr<JointVelocityLimitChecker>;
        explicit JointVelocityLimitChecker(double max_joint_vel_radian)
        : max_joint_velocity_(max_joint_vel_radian) {};
        ~JointVelocityLimitChecker() override = default;

        // The checking interface
        bool HasRequiredField(const CommandInput& input, const RobotArmCommand& command) override;
        CheckResult CheckSafety(const CommandInput& input, const RobotArmCommand& command) override;
    private:
        double max_joint_velocity_;
    };
}
