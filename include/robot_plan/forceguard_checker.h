//
// Created by wei on 9/2/19.
//

#pragma once

#include <memory>
#include <robot_msgs/ForceGuard.h>
#include <robot_msgs/ExternalForceGuard.h>

#include "common/safety_checker_interface.h"


namespace plan_runner {

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
        ExternalForceGuardChecker(
            int body_idx, Eigen::Vector3d point_in_body,
            Eigen::Vector3d force_threshold, int force_expressed_in_frame);
        ~ExternalForceGuardChecker() override = default;

        // The evaluation interface
        bool HasRequiredField(const CommandInput& input, const RobotArmCommand& command) override {
            return input.latest_measurement->torque_validity;
        }
        CheckResult CheckSafety(const CommandInput& input, const RobotArmCommand& command) override;

        // Construct from message
        static ExternalForceGuardChecker::Ptr ConstructFromMessage(
            const RigidBodyTree<double>& tree,
            const robot_msgs::ExternalForceGuard &message);
        static std::vector<SafetyChecker::Ptr> ConstructCheckersFromForceGuardMessage(
            const RigidBodyTree<double>& tree,
            const robot_msgs::ForceGuard& message);
    private:
        // The force applied on which body
        const int body_idx_;
        Eigen::Vector3d point_on_body_;

        // The force information
        Eigen::Vector3d force_threshold_;
        const int force_expressed_in_frame_;

        // The cache types
        Eigen::MatrixXd jacobian_cache;
        Eigen::VectorXd torque_threshold_cache;
    };
}
