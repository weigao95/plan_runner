//
// Created by wei on 8/31/19.
//

#pragma once

#include <memory>

#include "common/communication_types.h"
#include "common/plan_common_types.h"


namespace arm_runner {

    class SafetyChecker {
    public:
        // Types
        using Ptr = std::shared_ptr<SafetyChecker>;
        struct CheckResult {
            bool is_safe;
            double violation; // Potentially not used
        };

        // Virtual class
        virtual ~SafetyChecker() = default;

        // The input and command might not have the measurement/command field required for this checker.
        // For instance, the force guard requires joint torque measurement, which is not provided on many robots
        // This interface asks whether the checker has the required field
        virtual bool HasRequiredField(const CommandInput& input, const RobotArmCommand& command) = 0;

        // The main checking interface
        // When this method is invoked, the HasRequiredField MUST be true.
        virtual CheckResult CheckSafety(const CommandInput& input, const RobotArmCommand& command) = 0;
    };
}
