//
// Created by wei on 9/9/19.
//

#pragma once

#include "common/plan_base.h"


namespace plan_runner {

    // These types of plan typically output a desired joint-space velocity, which
    // can be further integrated into commanded position.
    // Primary for robots that are NOT torque-control
    class PositionVelocityPlan : public RobotPlanBase {
    protected:
        explicit PositionVelocityPlan(bool use_commaned_fwd_integration = true)
        : use_commanded_fwd_integration_(use_commaned_fwd_integration) {};
        ~PositionVelocityPlan() override = default;

        // Use which position as the one in forward integration
    protected:
        bool use_commanded_fwd_integration_;
        const double* GetForwardIntegrationJointPosition(const plan_runner::CommandInput &input) const;
    public:
        LoadParameterStatus LoadParameterFrom(const YAML::Node& datamap) override;
        void SaveParameterTo(YAML::Node& datamap) const override;
    };
}
