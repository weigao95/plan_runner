//
// Created by wei on 8/31/19.
//

#pragma once

#include <memory>

#include "common/yaml_serializable.h"
#include "common/communication_types.h"
#include "common/plan_common_types.h"


namespace plan_runner {

    class SafetyChecker : public YamlSerializableParameter {
    public:
        // Types
        using Ptr = std::shared_ptr<SafetyChecker>;
        struct CheckResult {
            bool is_safe;
            double violation; // Potentially not used

            void set_safe() {
                is_safe = true;
                violation = 0;
            }
        };

        // Virtual class
        ~SafetyChecker() override = default;

        // The input and command might not have the measurement/command field required for this checker.
        // For instance, the force guard requires joint torque measurement, which is not provided on many robots
        // This interface asks whether the checker has the required field
        virtual bool HasRequiredField(const CommandInput& input, const RobotArmCommand& command) = 0;

        // The main checking interface
        // When this method is invoked, the HasRequiredField MUST be true.
        virtual CheckResult CheckSafety(const CommandInput& input, const RobotArmCommand& command) = 0;
    };
}
