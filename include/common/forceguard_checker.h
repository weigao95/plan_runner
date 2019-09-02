//
// Created by wei on 9/2/19.
//

#pragma once

#include <memory>

#include "common/safety_checker_interface.h"


namespace arm_runner {

    class TotalForceGuardChecker : public SafetyChecker {
    public:
        using Ptr = std::shared_ptr<TotalForceGuardChecker>;
        explicit TotalForceGuardChecker(double total_force_threshold) : threshold_(total_force_threshold) {};
        ~TotalForceGuardChecker() override = default;

        // Need force measurement
        bool HasRequiredField(const CommandInput& input, const RobotArmCommand& command) override {
            return input.latest_measurement->torque_validity;
        };

        // Just check the norm of the measured force
        CheckResult CheckSafety(const CommandInput& input, const RobotArmCommand& command) override;
    private:
        const double threshold_;
    };

    class ExternalForceGuardChecker : public SafetyChecker {
    public:
        using Ptr = std::shared_ptr<ExternalForceGuardChecker>;
        explicit ExternalForceGuardChecker(
            int body_idx, Eigen::Vector3d point_in_body,
            Eigen::Vector3d force_threshold, int force_expressed_in_frame);
        ~ExternalForceGuardChecker() override = default;

        // The evaluation interface
        bool HasRequiredField(const CommandInput& input, const RobotArmCommand& command) override;
        CheckResult CheckSafety(const CommandInput& input, const RobotArmCommand& command) override;

    private:
        // The force applied on which body
        const int body_idx_;
        Eigen::Vector3d point_on_body_;

        // The force information
        Eigen::Vector3d force_threshold_;
        const int force_expressed_in_frame_;
    };

}
